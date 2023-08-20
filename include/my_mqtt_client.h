#ifndef MY_MQTT_CLIENT_H
#define MY_MQTT_CLIENT_H

#include "mqtt_client.h"

class MyMqttClient {
private:
    esp_mqtt_client_handle_t client;

    static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

public:
    MyMqttClient(const char* mqtt_server, const char* mqtt_port, const char* mqtt_user, const char* mqtt_password);
    void start();
    void publish(const char* topic, const char* data);
    static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
};

#endif //MY_MQTT_CLIENT_H
