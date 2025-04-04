#ifndef CONFIG_H_
#define CONFIG_H_

#include <avr/pgmspace.h>

const uint16_t IDENT_PERIOD = 60 * (60 * 12 / 8); // update every 12 hours (8sec interval)
const uint16_t UPDATE_PERIOD = 60 * 5 / 8; // update every 5 min (8sec interval)
const uint16_t START_DELAY_PERIOD = 60 * 3 / 8;

#if defined(INDOOR)
    #define DEVICE_ID "indoor"
    const uint16_t BANDGAP_REFERENCE_VOLTAGE = 1040; //1067;
#elif defined(OUTDOOR)
    #define DEVICE_ID "outdoor"
    const uint16_t BANDGAP_REFERENCE_VOLTAGE = 1075;
#else    
    #define DEVICE_ID "bmp280"
    const uint16_t BANDGAP_REFERENCE_VOLTAGE = 1100;
#endif

#define TOPIC_NAME "home/BMP280_" DEVICE_ID

const char temp_conf_topic[] PROGMEM  =  "homeassistant/sensor/BMP280_" DEVICE_ID "t/config";
const char temp_conf_payload[] PROGMEM  =  R"({"uniq_id":")" DEVICE_ID R"(t","name":"Temperature","dev_cla":"temperature","stat_t":"home/BMP280_)" DEVICE_ID R"(","unit_of_meas":"°C","val_tpl":"{{value_json.t}}","dev":{"ids":[")" DEVICE_ID R"("],"name":"BMP280 Sensor", "mf":"Tsukihime","mdl":"nRF24/BMP280 Sensor"}})";

const char press_conf_topic[] PROGMEM  = "homeassistant/sensor/BMP280_" DEVICE_ID "p/config";
const char press_conf_payload[] PROGMEM  = R"({"uniq_id":")" DEVICE_ID R"(p","name":"Pressure","dev_cla":"pressure","stat_t":"home/BMP280_)" DEVICE_ID R"(","unit_of_meas":"hPa","val_tpl":"{{value_json.p}}","dev":{"ids":[")" DEVICE_ID R"("]}})";

const char voltage_conf_topic[] PROGMEM  =  "homeassistant/sensor/BMP280_" DEVICE_ID "v/config";
const char voltage_conf_payload[] PROGMEM  =  R"({"uniq_id":")" DEVICE_ID R"(v","name":"Battery voltage","dev_cla":"voltage","stat_t":"home/BMP280_)" DEVICE_ID R"(","unit_of_meas":"mV","val_tpl":"{{value_json.v}}","dev":{"ids":[")" DEVICE_ID R"("]}})";

const char batt_conf_topic[] PROGMEM  =  "homeassistant/sensor/BMP280_" DEVICE_ID "b/config";
const char batt_conf_payload[] PROGMEM  =  R"({"uniq_id":")" DEVICE_ID R"(b","name":"Battery","dev_cla":"battery","stat_t":"home/BMP280_)" DEVICE_ID R"(","unit_of_meas":"%","val_tpl":"{{value_json.b}}","dev":{"ids":[")" DEVICE_ID R"("]}})";

#endif /* CONFIG_H_ */