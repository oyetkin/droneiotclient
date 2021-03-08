############ Visualize sensor data from otto's API ##############

import pandas as pd
import numpy as np
import requests
import os
import json
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.pyplot as plt 
#from pandas.plotting import register_matplotlib_converters
#register_matplotlib_converters()
import datetime

## To use this: 
# Go to https://www.openstreetmap.org/ and edit the display box to the area you want
# Copy the coordinates of the display box here. then either screenshot or export
# (using the share icon on the right + selecting the display box) the map. 
# From this tutorial: https://towardsdatascience.com/easy-steps-to-plot-geographic-data-on-a-map-python-11217859a2db
IMAGE_TOP = 51.33
IMAGE_BOT = 23.53
IMAGE_LEFT = -127.45
IMAGE_RIGHT = -65.15
bbox = (IMAGE_LEFT, IMAGE_RIGHT, IMAGE_BOT, IMAGE_TOP)

GET_URL = "https://api.is-conic.com/api/v0p1/debug/get_data"

json_data = '{"data":[{"timestamp":128,"key":"Arjun_weather_kit","unit":"RF%","value":57.0,"lat":35.12,"lon":-121.84},{"timestamp":130,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":130,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":132,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":133,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":135,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":135,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":138,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":138,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":140,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":140,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":143,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":143,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":145,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":145,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":148,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":148,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":150,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":150,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":153,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":153,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":155,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":155,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":158,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":158,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":160,"key":"Arjun_weather_kit","unit":"Celsius","value":24.6,"lat":35.12,"lon":-121.84},{"timestamp":160,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84},{"timestamp":163,"key":"Arjun_weather_kit","unit":"Celsius","value":24.7,"lat":35.12,"lon":-121.84},{"timestamp":163,"key":"Arjun_weather_kit","unit":"RF%","value":56.0,"lat":35.12,"lon":-121.84}]}'

def visualize_geography(df):
    """
    Show all sensors as points on a USA map; label their names
    """
    #take any location with more than 1 reading
    df = df.drop_duplicates(subset=['lat', 'lon'])

    fig, ax = plt.subplots()
    #plot the points and annotate the station id
    ax.scatter('lon', 'lat', zorder=1, alpha= 0.7, c='C1', data=df)
    for i in range(len(df)):
        ax.annotate(df.iloc[i]['key'], (df.iloc[i]['lon'], df.iloc[i]['lat']))
    #load in the map image in the background
    background_map = plt.imread(os.getcwd() + "/usa_map.png")
    ax.imshow(background_map, zorder=0, extent = bbox, aspect= 'equal')
    #labels and bounds
    ax.set_title('Location of All Sensors')
    ax.set_xlim(IMAGE_LEFT, IMAGE_RIGHT)
    ax.set_ylim(IMAGE_BOT, IMAGE_TOP)
    plt.show()

def visualize_graphs():
    """
    Display all sensors' readings over time
    """
    # get n distinct keys, make subplots.
    #groupby keys, timestamp isna --> drop --> plot remaining


if __name__ == "__main__":
    """
    header = {"Content-Type": "application/json"}
    response = requests.request("GET", GET_URL, headers=header)
    if response.status_code != 200:
        raise Exception(response.status_code, response.text)
    #response.json()
    """
    df = pd.DataFrame.from_dict(json.loads(json_data)['data'])
    visualize_geography(df)
    #visualize_graphs()