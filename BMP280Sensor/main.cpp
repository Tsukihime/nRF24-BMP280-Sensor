#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "utils.h"

extern "C" {
    #include "bmp280/bmp280.h"
    #include "rf24/nrf_connect.h"
    #include "rf24/RF24.h"
    #include "rf24/RF24MQTTGateway.h"
}

EMPTY_INTERRUPT(WDT_vect); // WDT Interrupt Is Used To Wake Up CPU From Sleep Mode

// Initialize or restore watchdog timer to combined mode (interrupt + reset)
void setupWatchdog() {
    MCUSR &= ~(1 << WDRF);               // Just to be safe since we can not clear WDE if WDRF is set
    cli();                               // disable interrupts so we do not get interrupted while doing timed sequence
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);   // First step of timed sequence, we have 4 cycles after this to make changes to WDE and WD timeout
    WDTCSR = (1 << WDP3) | (1 << WDP0) | // Set timeout to 8 seconds
             (1 << WDIE) | (1 << WDE);   // Enable both interrupt and system reset (combined mode), must be done in one operation
    sei();
}

void enterSleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
}

uint16_t getBatteryVoltage() {
    ADMUX = (0 << REFS1) | (1 << REFS0) |                          // select AVCC as reference
            (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // measure bandgap reference voltage

    ADCSRA = (1 << ADEN) |                               // enable ADC
             (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Prescaler Selections Div8

    _delay_us(500); // a delay rather than a dummy measurement is needed to give a stable reading!
   
    ADCSRA |= (1 << ADSC);                    // start conversion
    while(bit_is_set(ADCSRA, ADSC));          // wait to finish

    uint16_t voltage = BANDGAP_REFERENCE_VOLTAGE * 1024UL / ADC;
    ADCSRA &= ~(1 << ADEN);                   // disable ADC
    return voltage;                           // millivolts
}

uint8_t calculateCR2032BatteryPercentage(uint16_t voltage_mv) {
    if (voltage_mv >= 3000) {
        return 100;
    }

    if (voltage_mv >= 2900) {
        return 100 - (58 * (3000 - voltage_mv)) / 100; // Section 1: 3000mV - 2900mV -> 100% - 42%
    }

    if (voltage_mv >= 2740) {
        return 42 - (24 * (2900 - voltage_mv)) / 160; // Section 2: 2900mV - 2740mV -> 42% - 18%
    }

    if (voltage_mv >= 2440) {
        return 18 - (12 * (2740 - voltage_mv)) / 300; // Section 3: 2740mV - 2440mV -> 18% - 6%
    }

    if (voltage_mv >= 2100) {
        return 6 - (6 * (2440 - voltage_mv)) / 340; // Section 4: 2440mV - 2100mV -> 6% - 0%
    }

    return 0;
}


void nrfSetup() {
    uint8_t gateway_address[6] = { "NrfMQ" };
    const uint8_t gateway_channel = 0x6f;

    RF24_begin();
    RF24_setPALevel(RF24_PA_HIGH);
    RF24_enableDynamicPayloads();
    RF24_setDataRate(RF24_1MBPS);
    RF24_setCRCLength(RF24_CRC_8);
    RF24_setChannel(gateway_channel);
    RF24_setAutoAck(true);
    RF24_openWritingPipe(gateway_address);
    RF24_stopListening();
}

void identify() {
    RF24MQTT_sendMessage_P(temp_conf_topic, temp_conf_payload, true);
    RF24MQTT_sendMessage_P(press_conf_topic, press_conf_payload, true);
    RF24MQTT_sendMessage_P(voltage_conf_topic, voltage_conf_payload, true);
    RF24MQTT_sendMessage_P(batt_conf_topic, batt_conf_payload, true);
}

void initAll() {
    clock_prescale_set(clock_div_8); // switch clock to 1 MHz
    PRR |= (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2) | (1 << PRUSART0); // disable all timers & USART
    ACSR |= (1 << ACD);  // disable Analog Comparator
    sleep_bod_disable(); // disable the BOD while sleeping

    // Enable pull-ups on all unused pins to reduce leakage current
    DDRB = 0x00; PORTB = 0xFF;
    DDRC = 0x00; PORTC = 0xFF;
    DDRD = 0x00; PORTD = 0xFF;

    bmp280_init();
    NRF_connect_init();
    setupWatchdog();
}

int main(void) {
    initAll();

    uint16_t update_counter = START_DELAY_PERIOD;
    uint16_t ident_counter = START_DELAY_PERIOD;
    uint16_t voltage = getBatteryVoltage();
    char value_str[13];

    while (true) {
        if(ident_counter == 0) {
            ident_counter = IDENT_PERIOD;
            nrfSetup();
            identify();
        }

        if (update_counter == 0) {
            update_counter = UPDATE_PERIOD;
            char buff[40] = "";
            bmp280_takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);

            int32ToStrFixedPoint((bmp280_gettemperature() + 5) / 10, value_str, 1);
            strcat(buff,"{\"t\":");
            strcat(buff, value_str);

            int32ToStrFixedPoint(bmp280_getpressure(), value_str, 2);
            strcat(buff,",\"p\":");
            strcat(buff, value_str);

            int32ToStrFixedPoint(voltage, value_str);
            strcat(buff,",\"v\":");
            strcat(buff, value_str);

            uint8_t batt = calculateCR2032BatteryPercentage(voltage);
            int32ToStrFixedPoint(batt, value_str);
            strcat(buff,",\"b\":");
            strcat(buff, value_str);
            strcat(buff,"}");

            nrfSetup();
            RF24MQTT_sendMessage(TOPIC_NAME, buff, false);
            voltage = getBatteryVoltage();
        }

        update_counter--;
        ident_counter--;
        
        enterSleep();
        setupWatchdog(); // After waking up from sleep, restore combined mode
    }
}
