##### Runs a few basic commands with Otto's server #####

import requests 
import pandas as pd 
import numpy as np
import datetime
import sys
import ssl

ssl._create_default_https_context = ssl._create_unverified_context


GET_URL = "http://api.is-conic.com/api/v0p1/debug/get_data"
BATCH_POST_URL = "http://api.is-conic.com/api/v0p1/sensor/batch"
POST_URL = "http://api.is-conic.com/api/v0p1/sensor/"
MY_LAT = 38.819991
MY_LON = -112.13512
MY_ID = "abc_123"

def post(meas_type, unit, value, timestamp=True):
    """
    Uses the POST api endpoint to post a single sensor reading. 
    
    meas_type: type of measurement (humidity, temperature, etc)

    unit: unit of measurement (C, F, RF%)

    timestamp: whether to include a timestamp or not
    """
    payload = {"key": meas_type, "unit": unit, "value": value, "lat": MY_LAT, "lon": MY_LON}
    if timestamp:
        payload["timestamp"] = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    
    resp = requests.post(POST_URL, data=payload)
    return resp

def batch_post(id, type, unit, values):
    """
    Uses the batch POST api endpoint to post many sensor readings, all
    from the same sensor with the same type of measurement
    """
    payloads = []
    for v in values:
        payload = {"key": meas_type, "unit": unit, "value": v, "lat": MY_LAT, "lon": MY_LON}        
        paylods.append(payload)
    resp = requests.post(BATCH_POST_URL, data=payloads)
    return resp


if __name__ == "__main__":
    """
    Usage: 
    POST: python sensor_prototype.py abc123 temperature F 28
    GET: python sensor_prototype.py abc123 
    """
    arglen = len(sys.argv)
    if arglen == 2: #one argument --> get sensor values
        x = ""
        #get_sensor_history(name)
    elif arglen >= 4: #more arguments --> post sensor readings
        meas_type = sys.argv[1]
        unit = sys.argv[2]
        if arglen > 4: #more than 4 --> batch post
            meas_values = [sys.argv[i] for i in range(3, arglen)]
        else: #exactly 4 --> single post
            meas_value = float(sys.argv[3])
            #post(meas_type, unit, meas_value)
    else:
        print("Specify arguments correctly...")
    