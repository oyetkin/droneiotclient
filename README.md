# droneiotclient

## Overview

This is the client end of a system designed to measure sensor data from a set of "Data Loggers" deployed in the field, retrieve the sensor data from a "Data Collector" device, and then process and use the data. The system contains 4 main items:

1. esp32_sleep: This code is for the Data Logger. The device is meant to include an ESP32 microcontroller, an SSD1306 display screen, a BME280 temperature/pressure/humidity sensor, and an IR receiver. The device consumes very low power so that it can last for weeks on battery power alone. It wakes up on a configurable interval to collect data, stores that data, and goes back to sleep. When the Data Collector device sends an IR signal to the device, it wakes up, sends its data using point-to-point WiFi (ESP-Now), and goes back to sleep.
The device also has a second Wifi mode, which can be toggled using a push button. In this mode, the device wakes up, connects to a WiFi network, and posts its data to a server.  

2. esp32_receiver. This code is for the Data Receiver. It is also meant to be used with an ESP32 device. The receiver uses a 38kHz-modulated infrared signal to wake the Data Logger when one button is pressed. It receives the Logger's data using ESP-Now point-to-point WiFi. It stores data from many sensors, and when it is in range of a WiFi network, you can push another button to make it upload its data. 

3. You'll need a server to collect and store data from the sensors. An open-source server that is compatible with this project available here: https://github.com/oyetkin/droneiotserver

4. Demo notebook. There are myriad ways to use and store the data, but we've created one sample project. It displays the locations of some sensors, and also creates a heatmap of some other sensors. Depending on what data is currently on the server, this demo script may fail to display data, but it should be easy to edit it to display whatever data is on the server.

## Quick start

1. Build the Data Logger and Data Receiver, following the schematic diagrams in the repository. There's also a tutorial article with more details on components and wiring. 
2. Clone this repo. In both esp32_receiver and esp32_sleep, edit the lines of code for your server URL and your WiFI ssid and password. In esp32_sleep, edit the lines of code for your latitude, longitude, and device name. 
3. Clone and run the server repo here: https://github.com/oyetkin/droneiotserver
4. Upload the esp32_receiver and esp32_sleep code to their respective devices. Set the Data Logger to Wifi Mode using one of the two buttons if you want it to upload data directly, or set it to Trigger Mode if you want to collect data through the receiver. 
5. Run the Demo ipy notebook to visualize your data. 

Read the tutorial for a more detailed introduction. See the server repo for more details on how to retrieve data. 

## File guide

1. Tutorial article, schematic
2. PCB
  Bill of materials for PCB
3. code
