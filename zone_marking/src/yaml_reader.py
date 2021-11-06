#!/usr/bin/env python
import sys
import rospy
import yaml
import argparse
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import String
from zone_marking.srv import zoneMarking

#This node reads in zones information from a yaml file and then
#converts it to PolygonStamped messages that can then be used to make
#zone marking service calls every time an update is made

class zoneClientNode:

    def __init__(self, params):

        name = String()
        name.data = params['Name']

        shape = PolygonStamped()
        head = params['Header']
        shape.header.seq = head['seq']
        shape.header.frame_id = head['frame_id']

        pointsList = params['Points']

        for point in params['Points']:
            currentPoint = pointsList[point]
            shape.polygon.points.append(Point32(x = currentPoint['x'], y = currentPoint['y'], z = currentPoint['z']))

        outcome = self.zoneMarkingClient(shape, name)

    def zoneMarkingClient(self, polygon, name):

        rospy.wait_for_service('move_base/zoneMarking')
        try:
            zone_marking = rospy.ServiceProxy('move_base/zoneMarking', zoneMarking)
            resp1 = zone_marking(polygon, name)
            print('Request sent to server')
            return resp1.outcome.data

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



def read_yaml(filename):
    with open(filename, 'r') as file:
        params = yaml.safe_load(file)
    return params

if __name__ == "__main__":
    rospy.myargv(argv=sys.argv)
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--zone_data", type=str, help="path to a yaml file holding zone information"
    )
    args = parser.parse_args(argv[1:])
    params = read_yaml(args.zone_data)
    zoneClientNode(params)
