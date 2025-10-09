#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include "utils.h"

#include "bmp280/bmp280.h"
#include "rf24/RF24.h"
#include "rf24/RF24MQTTGateway.h"

BMP280 sensor(0x76);
    
RF24 radio(
    [](uint8_t data) -> uint8_t {
        SPDR = data;
        while (!(SPSR & (1 << SPIF)));
        return SPDR;
    },
    [](bool CSN, bool CE) {
        PORTB = (PORTB & ~((1 << PINB2) | (1 << PINB1))) | (CSN << PINB2) | (CE << PINB1);
    }
);

RF24MQTTGateway mqtt(radio);

uint16_t battery_voltage_mv = 0;

struct SETTINGS {
    uint16_t magic;
    uint16_t bandgap;
    uint8_t power;
    char id[6];
} settings;

EMPTY_INTERRUPT(WDT_vect); // WDT Interrupt Is Used To Wake Up CPU From Sleep Mode

inline void wdtRestoreInterruptAndResetMode() {
    WDTCSR |= (1 << WDIE);
}

void enterSleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
}

uint16_t getBatteryVoltage() {
    #define ADC_REF_AVCC ((0 << REFS1) | (1 << REFS0))
    #define ADC_BANDGAP_MUX ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0))
    #define ADC_PRESCALER_DIV8 ((0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

    power_adc_enable();

    ADMUX = ADC_REF_AVCC | ADC_BANDGAP_MUX;
    ADCSRA = (1 << ADEN) | ADC_PRESCALER_DIV8;

    _delay_us(500);  // a delay is needed to give a stable reading!

    ADCSRA |= (1 << ADSC);            // start conversion
    while (bit_is_set(ADCSRA, ADSC)); // wait to finish

    uint16_t voltage_mv = settings.bandgap * 1024UL / ADC;
    ADCSRA &= ~(1 << ADEN);           // disable ADC

    power_adc_disable();
    return voltage_mv;
}

uint8_t calcBatteryLevel(uint16_t voltage_mv) {
    if (voltage_mv >= 3000) return 100;
    if (voltage_mv >= 2880) return 100 - (3000 - voltage_mv) / 8; // 3000 mV -> 100%, 2880 mV -> 85%
    if (voltage_mv >= 2680) return 85 - (2880 - voltage_mv) / 8;  // 2880 mV ->  85%, 2680 mV -> 60%
    if (voltage_mv >= 2200) return 60 - (2680 - voltage_mv) / 8;  // 2680 mV ->  60%, 2200 mV -> 0%
    return 0;
}

void nrfSetup() {
    static uint8_t gateway_address[6] = {"NrfMQ"};
    static const uint8_t gateway_channel = 0x6f;

    radio.begin();
    radio.setPALevel(settings.power);
    radio.enableDynamicPayloads();
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setChannel(gateway_channel);
    radio.setAutoAck(true);
    radio.openWritingPipe(gateway_address);
    radio.stopListening();
}

uint8_t renderTemplate(const char* _template, uint16_t index) {
    char c = pgm_read_byte(&_template[index]);
    if (c != PLACEHOLDER_CHAR) return c;

    uint8_t offset = 0;
    do {
        char prev = pgm_read_byte(&_template[--index]);
        if (prev != PLACEHOLDER_CHAR) {
            break;
        }
        offset++;
    } while (index);
    
    return settings.id[offset];
}

bool identify() {
    return mqtt.sendToRadio(id_topic, sizeof(id_topic) - 1, id_payload, sizeof(id_payload) - 1, true, true, renderTemplate);
}

void measure() {
    char value_str[13];
    char payload[50] = "";

    sensor.takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);

    int32ToStrFixedPoint((sensor.getTemperature() + 5) / 10, value_str, 1);
    strcat(payload, "{\"t\":");
    strcat(payload, value_str);

    int32ToStrFixedPoint(sensor.getPressurePa(), value_str, 2);
    strcat(payload, ",\"p\":");
    strcat(payload, value_str);

    int32ToStrFixedPoint(battery_voltage_mv, value_str);
    strcat(payload, ",\"v\":");
    strcat(payload, value_str);

    int32ToStrFixedPoint(calcBatteryLevel(battery_voltage_mv), value_str);
    strcat(payload, ",\"b\":");
    strcat(payload, value_str);

    strcat(payload, "}");

    mqtt.publish(state_topic, payload, false);
    battery_voltage_mv = getBatteryVoltage();
}

void generateUID(char uid[6]) {
    struct ENTROPY {
        uint8_t osccal;
        int32_t temp;
        int32_t pressure;
        uint16_t voltage;
        BMP280_CAL_DATA cal_data;
    } entropy;

    sensor.takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);
    entropy.temp = sensor.getTemperature();
    entropy.pressure = sensor.getPressurePa();
    entropy.voltage = getBatteryVoltage();
    entropy.osccal = OSCCAL;
    memcpy(&entropy.cal_data, sensor.getCalibrationData(), sizeof(BMP280_CAL_DATA));

    uint8_t* bytes = (uint8_t*)&entropy;
    uint32_t hash = 5381;
    for (uint8_t i = 0; i < sizeof(entropy); i++) {
        hash = ((hash << 5) + hash) ^ bytes[i];
    }

    for (int8_t i = 5; i >= 0; i--) {
        uint8_t nibble = hash & 0xF;
        uid[i] = (nibble < 10) ? (nibble + '0') : (nibble - 10 + 'A');
        hash >>= 4;
    }
}

void initAll() {
    wdt_enable(WDTO_8S);
    wdtRestoreInterruptAndResetMode();
    clock_prescale_set(clock_div_8);
    ACSR |= (1 << ACD);     // disable Analog Comparator
    ADCSRA &= ~(1 << ADEN); // disable ADC
    power_all_disable();
    sleep_bod_disable();

    // Enable pull-ups on all unused pins to reduce leakage current
    DDRB = 0x00; PORTB = 0xFF;
    DDRC = 0x00; PORTC = 0xFF;
    DDRD = 0x00; PORTD = 0xFF;

    PORTB &= ~(1 << PINB1);
    PORTB |= (1 << PINB2);

    power_spi_enable();
    DDRB |= (1 << PINB1) | (1 << PINB2) | (1 << PINB3) | (0 << PINB4) | (1 << PINB5);

    SPSR = (1 << SPI2X);  // Double SPI speed
    SPCR = 1 << SPE                     /* SPI module enable: enabled */
           | 0 << DORD                  /* Data order: disabled */
           | 1 << MSTR                  /* Master/Slave select: enabled */
           | 0 << CPOL                  /* Clock polarity: disabled */
           | 0 << CPHA                  /* Clock phase: disabled */
           | 0 << SPIE                  /* SPI interrupt enable: disabled */
           | (0 << SPR1) | (0 << SPR0); /* SPI Clock rate selection: fosc/4 */
    
    sei();

    _delay_ms(250); // Preventing EEPROM Corruption

    power_twi_enable();
    sensor.init();
    sensor.setSampling(MODE_FORCED, SAMPLING_X2, SAMPLING_X16, FILTER_OFF, STANDBY_MS_1);

    const uint16_t MAGIC = 0xC0DE;
    eeprom_read_block(&settings, 0, sizeof(settings));
    if (settings.magic != MAGIC) { // load defaults
        settings.magic = MAGIC;
        settings.bandgap = REFERENCE_VOLTAGE;
        settings.power = RF24_PA_MAX;
        generateUID(settings.id);
        eeprom_write_block(&settings, 0, sizeof(settings));
    }

    battery_voltage_mv = getBatteryVoltage();
    // Update state topic id
    memcpy(state_topic + sizeof(state_topic) - sizeof(settings.id) - 1, settings.id, sizeof(settings.id));
}

// Counter for tracking identification delay, preserved between resets
uint16_t ident_counter __attribute__((section(".noinit")));

int main(void) {
    bool isExternalReset = MCUSR & (1 << EXTRF);
    MCUSR = 0;

    initAll();

    uint16_t update_counter = isExternalReset ? 0 : START_DELAY_PERIOD;

    if (isExternalReset) {
        ident_counter = 0;
    } else if (ident_counter == 0 || ident_counter > IDENT_PERIOD) {
        ident_counter = START_DELAY_PERIOD;
    }

    while (true) {
        if (ident_counter == 0) {
            ident_counter = IDENT_PERIOD;
            nrfSetup();
            identify();
        }

        if (update_counter == 0) {
            update_counter = UPDATE_PERIOD;
            nrfSetup();
            measure();
        }

        update_counter--;
        ident_counter--;

        enterSleep();
        wdtRestoreInterruptAndResetMode();
    }
}
