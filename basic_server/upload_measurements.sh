#!/bin/bash


while [[ 1 ]];do
	export SENSOR_DATA=`head -1 /dev/cu.usbmodem14201`
	export T=`echo $SENSOR_DATA|cut -d, -f1`
	echo $T
	python3 sensor_prototype.py oguz_sensor temperature $T
done
