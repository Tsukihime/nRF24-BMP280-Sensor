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
    wdt_enable(WDTO_8S);
    WDTCSR |= (1 << WDIE); // Enable both interrupt and system reset (combined mode)
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
        return 100; // ? 3.0 � � 100%
    }

    if (voltage_mv >= 2900) {
        return 100 - (15 * (3000 - voltage_mv)) / 100; // 3000 �� ? 100%, 2900 �� ? 85%
    }

    if (voltage_mv >= 2700) {
        return 85 - (25 * (2900 - voltage_mv)) / 200; // 2900 �� ? 85%, 2700 �� ? 60%
    }

    if (voltage_mv >= 2500) {
        return 60 - (40 * (2700 - voltage_mv)) / 200; // 2700 �� ? 60%, 2500 �� ? 20%
    }

    if (voltage_mv >= 2100) {
        return 20 - (20 * (2500 - voltage_mv)) / 400; // 2500 �� ? 20%, 2100 �� ? 0%
    }

    return 0; // < 2100 �� � 0%
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
    setupWatchdog();
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
}

// Counter for tracking identification delay, preserved between resets
uint16_t ident_counter __attribute__((section(".noinit")));

int main(void) {
    bool isWatchdogReset = MCUSR & (1 << WDRF);
    MCUSR = 0;

    initAll();

    if (isWatchdogReset) {
        if (ident_counter > IDENT_PERIOD) { // If the counter value is incorrect
            ident_counter = START_DELAY_PERIOD;
        } // else continue count
    } else {                                // Normal reset
        ident_counter = START_DELAY_PERIOD;
    }

    uint16_t update_counter = START_DELAY_PERIOD;
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
            char buff[50] = "";
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
