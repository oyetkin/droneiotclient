import requests 
import pandas as pd 
import numpy as np
from enum import Enum
import datetime
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.pyplot as plt 
import html
from pandas.plotting import register_matplotlib_converters
register_matplotlib_converters()
import sys

POST_URL = "http://localhost:8000/sensor.json"
GET_URL = "http://localhost:8000/sensor.json"

#this would be implemented on the Arduino in the final version
class SensorInstance:
    def __init__(self, mac_addr, measurement_type):
        """
        Manually specify the sensor's mac address (or another unique identifier).
        Measurement should be either 'temperature' or 'humidity'
        """
        self.uid = mac_addr
        self.type = measurement_type

    def post(self, measurement, include_time=True):
        """
        Post a measurement. Timestamp, sensor ID, and sensor type is posted along with the measurement.
        """
        data = {'key': self.type, 'value': measurement, 'sensor_id': self.uid}
        if include_time:
            data['sender_time'] = datetime.datetime.now()
        requests.post(POST_URL, data=data) 

def get_sensor_history(sensor_id):
    """
    Use get to view the history of a sensor's data. Displays a plot of the
    sensor data. 
    """
    dt_format = '%Y-%m-%d+%H:%M:%S.%f'

    response = requests.get(GET_URL)
    if response.status_code != 200:
        raise Exception(response.status_code, response.text)
    data = response.json()['data']
    df = pd.DataFrame.from_dict(data)

    #clean the data types
    #df['sender_time'] = df['sender_time'].apply(lambda x: x.replace("%3A", ":"))
    df['sender_time'] = df['sender_time'].apply(
        lambda x: datetime.datetime.strptime(x.replace("%3A", ":"), dt_format)
    )
    df['value'] = df['value'].astype(float)

    #select your sensor
    df = df[df['sensor_id']==sensor_id]
    with pd.option_context('display.max_rows', None, 'display.max_columns', None):  # more options can be specified also
        print(df['value'])
    #print(df['value'])

if __name__ == "__main__":
    """
    Usage: 
    POST: python sensor_prototype.py abc123 humidity 28
    GET: python sensor_prototype.py abc123
    """
    name = sys.argv[1]
    if len(sys.argv) == 2:
        get_sensor_history(name)
    elif len(sys.argv) == 4:
        meas_type = sys.argv[2]
        meas_value = sys.argv[3]
        temp = SensorInstance(name, meas_type)
        temp.post(float(meas_value))
    else:
        print("Specify arguments correctly...")
    