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

def visualize_graphs(df):
    """
    Display all sensors' readings over time
    """
    df = df[~df['timestamp'].isna()].sort_values('timestamp', axis=0)
    grouped = df.groupby(['key', 'unit'], as_index=False)

    i = 0 
    for name, grp in grouped:
        fig, ax = plt.subplots()  
        ax.plot('timestamp', 'value', data=grp)
        ax.set_title("{}: {}".format(name[0], name[1]))
        i += 1
    fig.suptitle("All Sensor Data")
    plt.tight_layout()
    plt.show()
    
        plt.show()


if __name__ == "__main__":
    header = {"Content-Type": "application/json"}
    response = requests.request("GET", GET_URL, headers=header, verify=False)
    if response.status_code != 200:
        raise Exception(response.status_code, response.text)
    json_data = response.json()
    df = pd.DataFrame.from_dict(json_data)
    visualize_geography(df)
    visualize_graphs(df)