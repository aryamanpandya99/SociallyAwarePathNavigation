#!/usr/bin/env python
import rospy
import sys
import yaml
import argparse
from custom_social_layer.msg import People, Person
from geometry_msgs.msg import Pose2D

#This node reads in people information from a yaml file and then
#converts it to people messages that can then be published onto
#the people_messages topic

class PeopleNode:
    def __init__(self, params):
        self.people_list = People()
        list1 = params["People"]
        current_person = Person()

        for person in params["People"]:
            rospy.loginfo("Initializing person: " + str(person))
            temp_person = list1[person]
            current_person.velocity = temp_person["Velocity"]
            current_pose = temp_person["Pose"]
            person_pose = Pose2D()
            person_pose.x = current_pose["x"]
            person_pose.y = current_pose["y"]
            person_pose.theta = current_pose["theta"]
            current_person.pose = person_pose

            self.people_list.people.append(current_person)

        self.people_publisher = rospy.Publisher(
            "people_messages", People, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(30), self.publish_people)

    def publish_people(self, people_list):
        self.people_publisher.publish(self.people_list)


def read_yaml(filename):
    with open(filename, "r") as file:
        parameters = yaml.safe_load(file)
        return parameters


if __name__ == "__main__":

    rospy.init_node("people_sender", anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--people_data", type=str, help="path to a yaml file holding people information"
    )
    args = parser.parse_args(argv[1:])
    params = read_yaml(args.people_data)
    PeopleNode(params)
    rospy.spin()
