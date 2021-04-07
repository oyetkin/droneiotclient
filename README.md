# droneiotclient

## Overview

This is the client end of a system designed to measure sensor data from a set of "Data Loggers" deployed in the field, retrieve the sensor data from a "Data Collector" device, and then process and use the data. The system contains 4 main items:

1. esp32_sleep: This code is for the Data Logger. The device is meant to include an ESP32 microcontroller, an SSD1306 display screen, a BME280 temperature/pressure/humidity sensor, and an IR receiver. The device consumes very low power so that it can last for weeks on battery power alone. It wakes up on a configurable interval to collect data, stores that data, and goes back to sleep. When the Data Collector device sends an IR signal to the device, it wakes up, sends its data using point-to-point WiFi (ESP-Now), and goes back to sleep.
The device also has a second Wifi mode, which can be toggled using a push button. In this mode, the device wakes up, connects to a WiFi network, and posts its data to a server.  

2. esp32_receiver. This code is for the Data Receiver. It is also meant to be used with an ESP32 device. 

3. You'll also need a server to collect and store data from the sensors. An open-source server that is compatible with this project available here: https://github.com/oyetkin/droneiotserver

4. Demo notebook. There
