#ifndef RF24MQTTGATEWAY_H_
#define RF24MQTTGATEWAY_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define RF24_PACKET_SIZE 32

void RF24MQTT_sendData(uint8_t* data, uint16_t size);

void RF24MQTT_sendMessage_P(const char* topic, const char* payload, bool retained);

void RF24MQTT_sendMessage(const char* topic, const char* payload, bool retained);

#endif /* RF24MQTTGATEWAY_H_ */
