#ifndef CONFIG_H_
#define CONFIG_H_

#include <avr/pgmspace.h>

constexpr uint16_t seconds_to_period(uint32_t seconds) {
    return static_cast<uint16_t>(seconds / 8);
}

constexpr uint16_t minutes(uint32_t m) {
    return seconds_to_period(m * 60);
}

constexpr uint16_t hours(uint32_t h) {
    return seconds_to_period(h * 60 * 60);
}

const uint16_t IDENT_PERIOD = hours(12);
const uint16_t UPDATE_PERIOD = minutes(5);
const uint16_t START_DELAY_PERIOD = minutes(3);
const uint16_t REFERENCE_VOLTAGE = 1100;

#define PLACEHOLDER "\xff\xff\xff\xff\xff\xff"
const uint8_t PLACEHOLDER_CHAR = 255;

char state_topic[] = "home/sensor_" PLACEHOLDER;

const char id_topic[] PROGMEM = "homeassistant/device/" PLACEHOLDER "/config";
const char id_payload[] PROGMEM =
    R"({"dev":{"ids":")" PLACEHOLDER
    R"(","name":"BMP280Sensor","mf":"Tsukihime","mdl":"nRF24/BMP280 Sensor"},"o":{"name":"BMP280Sensor)" PLACEHOLDER
    R"("},"cmps":{"t":{"p":"sensor","uniq_id":")" PLACEHOLDER
    R"(t","name":"Temperature","dev_cla":"temperature","unit_of_meas":"°C","val_tpl":"{{value_json.t}}"},"p":{"p":"sensor","uniq_id":")" PLACEHOLDER
    R"(p","name":"Pressure","dev_cla":"pressure","unit_of_meas":"hPa","val_tpl":"{{value_json.p}}"},"b":{"p":"sensor","uniq_id":")" PLACEHOLDER
    R"(b","name":"Battery","dev_cla":"battery","unit_of_meas":"%","val_tpl":"{{value_json.b}}"},"v":{"p":"sensor","uniq_id":")" PLACEHOLDER
    R"(v","name":"Battery voltage","dev_cla":"voltage","unit_of_meas":"mV","val_tpl":"{{value_json.v}}"}},"stat_t":"home/sensor_)" PLACEHOLDER
    R"("})";

#endif /* CONFIG_H_ */
