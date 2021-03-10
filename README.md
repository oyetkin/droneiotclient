# droneiotclient


This currently just contains several separate Drone IOT related projects. They are listed below:

1. Virginia Demo: Jupyter notebook that gets data from Otto's server and displays it. Meant to exhibit a preview of what our data could look like and how it could be used. 
2. arduino_wifi: Just a testing script to play wiht random functions like IR on the arduino. 
3. esp_32_demo: Code to run the ESP 32 as a weather station. Currently (3/09/21) it assumes the ESP 32 is connected to Wifi and has a DHT11 humidity/temp sensor, BMP 180 pressure sensor, and an OLED. Posts to Otto's server every 2 seconds. 
4. simple server: Very basic server demo. Run server_demo from one terminal to start the server; run sensor_prototype from another terminal and it will post to the server with each call (specify args from the command line) (does not include an actual sensor!).  Sensor_prototype_otto_server was meant to interface with Otto's server but there were a few bugs left to correct still. upload_measurements.sh just runs sensor_protoype many times to post lots of data. 
5. visualize_sensors.py: Draft version of the Virginia Demo notebook. 
6. Barometer_HP20x_HumidityDHTArduino: Oguz's arduino code.  
