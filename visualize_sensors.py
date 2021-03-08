############ Visualize sensor data from otto's API ##############

import pandas as pd 
import numpy as np 
import os
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.pyplot as plt 
from pandas.plotting import register_matplotlib_converters
register_matplotlib_converters()
import datetime

IMAGE_TOP = 51.33
IMAGE_BOT = 23.53
IMAGE_LEFT = -127.45
IMAGE_RIGHT = -65.15
bbox = (IMAGE_LEFT, IMAGE_RIGHT, IMAGE_BOT, IMAGE_TOP)



def visualize_geography():
    """
    Show all sensors as points on a USA map
    """
    #read into pandas --> isna --> filter lat/lon
    x = [-117.084493]
    y = [32.638612]

    background_map = plt.imread(os.getcwd() + "/usa_map.png")
    fig, ax = plt.subplots()
    #ax.set_title('Plotting Spatial Data on the Oval')
    ax.scatter(x, y, zorder=1, alpha= 0.7, c='C1')
    ax.set_xlim(IMAGE_LEFT, IMAGE_RIGHT)
    ax.set_ylim(IMAGE_BOT, IMAGE_TOP)
    ax.imshow(background_map, zorder=0, extent = bbox, aspect= 'equal')
    plt.show()

def visualize_graphs():
    """
    Display all sensors' readings over time
    """
    # get n distinct keys, make subplots.
    #groupby keys, timestamp isna --> drop --> plot remaining


if __name__ == "__main__":
    visualize_geography()
    visualize_graphs()