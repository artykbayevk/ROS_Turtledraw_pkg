#!/usr/bin/env python

import os
import sys
import time
import util
import rospy
import argparse
from math import pi
from random import random
from functools import partial
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Point
from turtlesim.srv import Kill, Spawn, SetPen

positions = {}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="file of element")

    args = vars(parser.parse_args())

    with open(args["file"], "r") as f:
        lines = f.readlines()
    coords = []
    for line in lines:
        coords.append([float(i) for i in line.split(",")])


    points = []
    for coord in coords:
        points.append(Point(x=transform_coord(coord[0]), y=transform_coord(coord[1])))


    rospy.init_node("turtle_draw")
    rospy.ServiceProxy("/reset", Empty)()
    rospy.ServiceProxy("/kill", Kill)("turtle1")


    turtle_name = rospy.ServiceProxy("/spawn", Spawn)(random() * 11, random() * 11, random() * 2 * pi, "").name

    subscribe_(turtle_name)

    func_dict = {
        "log"       : lambda s: rospy.loginfo("[%s] %s" % (turtle_name, s)),
        "pen"       : partial(set_pen_on_off, set_pen=rospy.ServiceProxy("/%s/set_pen" % turtle_name, SetPen)),
        "move"      : partial(move_turtle, pub=rospy.Publisher("/%s/cmd_vel" % turtle_name, Twist, queue_size=10)),
        "current" : partial(get_turtle_pose, turtle_name)
    }

    func_dict["pen"](True)

    util.draw(points, func_dict)

def subscribe_(turtle_name):
    rospy.Subscriber("/%s/pose" % turtle_name, Pose, partial(new_pose_callback, turtle_name=turtle_name))

    while turtle_name not in positions: # We need to sleep for a bit to let the subscriber fetch the current pose at least once
        time.sleep(0)

def new_pose_callback(pose, turtle_name):
    global positions
    positions[turtle_name] = pose


def get_turtle_pose(turtle_name):
    pose = positions[turtle_name]
    return util.Pose(pose.x, pose.y, pose.theta)

def move_turtle(linear_speed, angular_speed, pub):
    pub.publish(Twist(linear=Point(linear_speed, 0, 0), angular=Point(0, 0, angular_speed)))


def set_pen_on_off(on, set_pen):
    if on:
        set_pen(255, 255, 255, 3, 0) # On, white, width of 3
    else:
        set_pen(255, 255, 255, 3, 1) # Off

def transform_coord(coord):
    return ((coord + 1) / 2) * 11

main()
