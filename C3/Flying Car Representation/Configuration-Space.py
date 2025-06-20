#!/usr/bin/env python
# coding: utf-8

# ## Confguration Space
# 
# In this notebook you'll create a configuration space given a map of the world and setting a particular altitude for your drone. You'll read in a `.csv` file containing obstacle data which consists of six columns $x$, $y$, $z$ and $\delta x$, $\delta y$, $\delta z$.
# 
# You can look at the `.csv` file [here](/edit/colliders.csv). The first line gives the map center coordinates and the file is arranged such that:
# 
# * $x$ -> NORTH
# * $y$ -> EAST
# * $z$ -> ALTITUDE (positive up, note the difference with NED coords)
# 
# Each $(x, y, z)$ coordinate is the center of an obstacle. $\delta x$, $\delta y$, $\delta z$ are the half widths of the obstacles, meaning for example that an obstacle with $(x = 37, y = 12, z = 8)$ and $(\delta x = 5, \delta y = 5, \delta z = 8)$ is a 10 x 10 m obstacle that is 16 m high and is centered at the point $(x, y) = (37, 12)$ at a height of 8 m.
# 
# Given a map like this, the free space in the $(x, y)$ plane is a function of altitude, and you can plan a path around an obstacle, or simply fly over it! You'll extend each obstacle by a safety margin to create the equivalent of a 3 dimensional configuration space. 
# 
# Your task is to extract a 2D grid map at 1 metre resolution of your configuration space for a particular altitude, where each value is assigned either a 0 or 1 representing feasible or infeasible (obstacle) spaces respectively.

# The end result should look something like this ... (colours aren't important)
# 
# ![title](grid_map.png)

# In[1]:


import numpy as np 
import matplotlib.pyplot as plt

get_ipython().run_line_magic('matplotlib', 'inline')


# In[2]:


plt.rcParams["figure.figsize"] = [12, 12]


# Read the csv file which contains the coordinates of the obstacles. 

# In[3]:


filename = 'colliders.csv'
# Read in the data skipping the first two lines.  
# Note: the first line contains the latitude and longitude of map center
# Where is this??
data = np.loadtxt(filename,delimiter=',',dtype='Float64',skiprows=2)
print(data)


# In[8]:


# Static drone altitude (metres)
drone_altitude = 10

# Minimum distance required to stay away from an obstacle (metres)
# Think of this as padding around the obstacles.
safe_distance = 3


# The given function will take the data from the file describing the obstacles city and will return a 2D grid representation showing open and closed spaces.

# In[15]:


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    cell_halfWidth = 0.5
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # TODO: Determine which cells contain obstacles
        # and set them to 1.
        #
        # Example:
        #
        #    grid[north_coordinate, east_coordinate] = 1
        
        # obstacle is accounted for only if it is present at given drone altitude
        z_min = alt - d_alt - safety_distance
        z_max = alt + d_alt + safety_distance
        if z_min <= drone_altitude <= z_max:
        
            # row(north) and column(east) numbers of the cell containing obstacle centre 
            centre_row = int(np.floor(north - north_min))
            centre_rowOffset = north-north_min-centre_row
            centre_column = int(np.floor(east - east_min))
            centre_columnOffset = east-east_min-centre_column
            
            # extent (half length in terms of number of cells) of the obstacle in row(north) and column(east) directions
            row_northExtent = int(np.ceil(d_north + safety_distance - 1 + centre_rowOffset))
            row_southExtent = int(np.ceil(d_north + safety_distance - centre_rowOffset))
            column_eastExtent = int(np.ceil(d_east + safety_distance - 1 + centre_columnOffset))
            column_westExtent = int(np.ceil(d_east + safety_distance - centre_columnOffset))
            
            # place the obstacle on the grid
            min_row = max(0, centre_row - row_southExtent)
            max_row = min(north_size-1, centre_row + row_northExtent)
            min_column = max(0, centre_column - column_westExtent)
            max_column = min(east_size-1, centre_column + column_eastExtent)
            
            for row in range(min_row, max_row+1):
                for column in range(min_column, max_column+1):
                    grid[row, column] = 1

    return grid


# In[16]:


grid = create_grid(data, drone_altitude, safe_distance)


# In[17]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
# NOTE: we're placing the origin in the lower lefthand corner here
# so that north is up, if you didn't do this north would be positive down
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# Play around with the `drone_altitude` and `safe_distance` values to get a feel for how it changes the map.

# [solution](/notebooks/Configuration-Space-Solution.ipynb)

# In[ ]:




