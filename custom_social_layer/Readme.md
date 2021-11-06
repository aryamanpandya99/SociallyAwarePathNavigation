This package enables socially aware path navigation by marking out personal space
around people detcted by the robot. These costmap markings demotivate robot path
planning within people's personal spaces.

**Custom messages:** 

- Person.msg: Contains a 2D pose, velocity and a header. 
- People.msg: Array of Person objects. 

**Test Node:**

people_publisher.py: Reads in people information from a .yaml file and publishes 
it to the "people_messages" topic to which the custom layer is subscribed. 