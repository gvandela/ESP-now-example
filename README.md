ESP-now example with sensor ESP8266 device and server ESP8266 device

For ESP32, only the WiFi library needs to be modified *probably*

Sensor is meant to consume as little as possible.
Server is meant to communicate over both ESP-now and WiFi, allowing for relay of MQTT messages to ESP-now and vice versa.
