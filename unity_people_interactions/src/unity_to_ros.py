#!/usr/bin/env python
import rospy
import sys
import yaml
import argparse
from custom_social_layer.msg import People
from unity_people_interactions.msg import UnityPerson, UnityPeople
from zone_marking.srv import mark_lines
from geometry_msgs.msg import Pose2D, Point32
from zone_marking.msg import segment, Lines


# This node reads in unity people information from a yaml file and then
# converts it to unity people messages from which we can call the custom
# social layer service as well as the line marking service (if needed)
# to populate the unity environment with personal spaces and extend that
# to drawing "future pose" personal spaces to which lines will be drawn from
# the current pose if the person's wand is pointing at a target


class UnityPeopleNode:
    def __init__(self, params):

        self.unity_people_list = UnityPeople()
        self.people_list = People()

        self.unity_people_list.people = []
        self.people_list.people = []

        if "People" in params:
            people_temp = params["People"]

            for person in params["People"]:
                rospy.loginfo("Initializing person: " + str(person))
                temp_person = people_temp[person]
                temp_unity_person = UnityPerson()
                temp_current_person = temp_person["Current"]
                temp_unity_person.current_pose.velocity = temp_current_person["Velocity"]
                temp_pose = temp_current_person["Pose"]
                person_pose = Pose2D()
                person_pose.x = temp_pose["x"]
                person_pose.y = temp_pose["y"]
                person_pose.theta = temp_pose["theta"]
                temp_unity_person.current_pose.pose = person_pose

                temp_future_person = temp_person["Future"]
                temp_unity_person.future_pose.velocity = temp_future_person["Velocity"]
                temp_future_pose = temp_future_person["Pose"]
                future_person_pose = Pose2D()
                future_person_pose.x = temp_future_pose["x"]
                future_person_pose.y = temp_future_pose["y"]
                future_person_pose.theta = temp_future_pose["theta"]
                temp_unity_person.future_pose.pose = future_person_pose

                temp_unity_person.wand_pointing.data = temp_person["Wand"]

                self.unity_people_list.people.append(temp_unity_person)

            self.line_marking_client()

        self.people_publisher = rospy.Publisher(
            "people_messages", People, queue_size=10
        )

        self.unity_subscriber = rospy.Subscriber(
            "unity_people_messages", UnityPeople, queue_size=10
        )

        self.timer = rospy.Timer(rospy.Duration(30), self.publish_people)

    def publish_people(self):
        people = self.unity_people_list if self.unity_people_list is not None else []
        for unity_person in people:
            if unity_person.wand_pointing:
                self.people_list.people.append(unity_person.future_pose)
            self.people_list.people.append(unity_person.current_pose)
        self.people_publisher.publish(self.people_list)
        print(self.people_list)

    def line_marking_client(self):
        line = segment()
        lines_list = Lines()
        temp_point = Point32()
        temp_point_future = Point32()
        count = 0
        for unity_person in self.unity_people_list.people:
            count = count + 1
            if unity_person.wand_pointing:
                temp_point.x = unity_person.current_pose.pose.x
                temp_point.y = unity_person.current_pose.pose.y
                temp_point_future.x = unity_person.future_pose.pose.x
                temp_point_future.y = unity_person.future_pose.pose.y
                line.points *= 0
                line.points.append(temp_point)
                line.points.append(temp_point_future)
                temp_name = "moving_person_"
                temp_name = temp_name + str(count)
                line.name = temp_name
                line.thickness = 5
                lines_list.segments.append(line)
        rospy.wait_for_service("move_base/mark_lines")
        try:
            line_marking = rospy.ServiceProxy("move_base/mark_lines", mark_lines)
            resp1 = line_marking(lines_list)
            print("Request sent to server")
            print(lines_list)
            return resp1.outcome.data

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


def read_yaml(filename):

    with open(filename, "r") as file:
        parameters = yaml.safe_load(file)
        return parameters


if __name__ == "__main__":

    rospy.init_node("unity_people_sender", anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--unity_people_data",
        type=str,
        help="path to a yaml file holding unity people information",
    )
    args = parser.parse_args(argv[1:])
    params = read_yaml(args.unity_people_data)
    UnityPeopleNode(params)
    rospy.spin()
