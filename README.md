# Socially Aware Path Navigation

Code developed at HRI lab, Tufts University.

## Overview
The goal of this project is to program robots using ROS2 to account for social norms while path planning. The repository contains packages that leverage costmaps/occupancy grids to introduce added functionality and awareness of the robot's environment.

## Requirements
- ROS2
- Python 3.7 or higher
- C++11

## Packages
### Custom Social Layer
This package enables socially aware path navigation by marking out personal space around people detected by the robot. These costmap markings demotivate robot path planning within people's personal spaces.

### Zone Marking
This package allows us to mark out zones in the environment and provide some information about them in ROS. In this implementation, zones can be named (for example "Zone-1-Kitchen") and they are then drawn on the map using a custom costmap layer.

### Unity People Interactions
This package pivots off of the previous two mentioned in order to implement socially aware path navigation within the Unity simulation environment. Input to this implementation would be a list of poses of people in the Unity environment, whether or not they're pointing their teleportation wand and if they are, where they're pointing. With this information, we use the custom social layer to mark personal spaces around current and future poses, and draw a line between them using the zone marking package. 

## Credits
This work was developed at HRI lab, Tufts University.
