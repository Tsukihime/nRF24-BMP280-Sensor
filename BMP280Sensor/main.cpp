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

void initWatchdog() {
    MCUSR &= ~(1 << WDRF);               // Just to be safe since we can not clear WDE if WDRF is set
    cli();                               // disable interrupts so we do not get interrupted while doing timed sequence
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);   // First step of timed sequence, we have 4 cycles after this to make changes to WDE and WD timeout
    WDTCSR = (1 << WDP3) | (1 << WDP0) | // timeout in 8 second, disable reset mode,
             (1 << WDIE);                // enable watchdog interrupt only mode, must be done in one operation
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
    RF24MQTT_sendMessage_P(batt_conf_topic, batt_conf_payload, true);
}

void initAll() {
    clock_prescale_set(clock_div_8); // switch clock to 1 MHz
	PRR |= (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2) | (1 << PRUSART0); // disable all timers & USART
    ACSR |= (1 << ACD);  // disable Analog Comparator
    sleep_bod_disable(); // disable the BOD while sleeping

    // Enable pull-ups on unused I/O pins
    DDRB = 0x00;
    PORTB = 0xFF;
    DDRC = 0x00;
    PORTC = 0xFF;
    DDRD = 0x00;
    PORTD = 0xFF;

    bmp280_init();
    NRF_connect_init();

    initWatchdog();
}

int main(void) {
    initAll();

    uint16_t counter = 0;
    uint16_t ident_counter = 0;
    char value_str[13];

    while (true) {
        if(ident_counter == 0) {
            nrfSetup();
            identify();
        }

        if (counter == 0) {
            char buff[40] = "";
            bmp280_takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);

            int32ToStrFixedPoint((bmp280_gettemperature() + 5) / 10, value_str, 1);
            strcat(buff,"{\"t\":");
            strcat(buff, value_str);

            int32ToStrFixedPoint(bmp280_getpressure(), value_str, 2);
            strcat(buff,",\"p\":");
            strcat(buff, value_str);

            int16_t batt = clamp(((int16_t)getBatteryVoltage() - 2000 + 5) / 10, 0, 100);            
            int32ToStrFixedPoint(batt, value_str);
            strcat(buff,",\"b\":");
            strcat(buff, value_str);
            strcat(buff,"}");

            nrfSetup();
            RF24MQTT_sendMessage(TOPIC_NAME, buff, false);
        }

        if (++counter >= UPDATE_PERIOD) counter = 0;
        if (++ident_counter >= IDENT_PERIOD) counter = 0;
        
        enterSleep();
    }
}
