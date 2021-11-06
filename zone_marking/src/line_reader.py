#!/usr/bin/env python
import sys
import rospy
import yaml
import argparse
from geometry_msgs.msg import Point32
from zone_marking.srv import mark_lines
from zone_marking.msg import segment, Lines

#This node reads in line information from a yaml file and then
#converts it to lines messages that can then be used to make
#line marking service calls every time an update is made 

class lineClientNode:

    def __init__(self, params):
        lines_list = Lines()
        lines_temp = params['Lines']
        for line in params['Lines']:
            current_line = lines_temp[line]
            line_seg = segment()
            line_seg.name = current_line['Name']
            line_seg.thickness = current_line['Thickness']
            point_list = current_line['Points']
            for point in current_line['Points']:
                current_point = point_list[point]
                line_seg.points.append(Point32(x=current_point['x'], y=current_point['y'], z=current_point['z']))
            lines_list.segments.append(line_seg)
            print(lines_list)
        outcome = self.lineMarkingClient(lines_list)

    def lineMarkingClient(self, lines_list):
        rospy.wait_for_service('move_base/mark_lines')
        try:
            line_marking = rospy.ServiceProxy('move_base/mark_lines', mark_lines)
            resp1 = line_marking(lines_list)
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
        "--line_data", type=str, help="path to a yaml file holding line information"
    )
    args = parser.parse_args(argv[1:])
    params = read_yaml(args.line_data)
    lineClientNode(params)
