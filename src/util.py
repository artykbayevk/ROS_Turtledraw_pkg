#!/usr/bin/env python

import math
import rospy
from Pose import Pose


def draw(points, func_dict):
    func_dict["pen"](False)
    move_forward(Pose(x=points[0].x, y=points[0].y), 1, 1, 0.1, func_dict)

    func_dict["pen"](True)
    for point in points[1:]:
        move_forward(Pose(x=point.x, y=point.y), 1, 1, 0.1, func_dict)

    func_dict["log"]("Finished!")

def move_forward(target, move_speed, spin_speed, pos_tolerance, func_dict):
    target_theta = _angle_between_points(func_dict["current"](), target)
    while not _are_angles_equal(func_dict["current"]().theta, target_theta, _deg_to_rad(5)):
        spin_func(spin_speed, _is_spin_clockwise(func_dict["current"]().theta, target_theta), func_dict)
        rospy.Rate(10).sleep
    stop_method(func_dict)
    _move(target, move_speed, spin_speed, pos_tolerance, func_dict)
    stop_method(func_dict)

def _move(target, move_speed, spin_speed, pos_tolerance, func_dict):
    while not _are_points_equal(func_dict["current"](), target, pos_tolerance):
        target_theta = _angle_between_points(func_dict["current"](), target)
        angle_correction = _min_angle_between_angles(func_dict["current"]().theta, target_theta)
        correction_spin_speed = _clamp(angle_correction, -spin_speed, spin_speed)
        func_dict["move"](move_speed, correction_spin_speed)
        rospy.Rate(10).sleep

def spin_func(speed, clockwise, func_dict):
    func_dict["move"](0, speed * (-1 if clockwise else 1))

def stop_method(func_dict):
    func_dict["move"](0, 0)


def _angle_between_points(point_a, point_b):
    return math.atan2((point_b.y - point_a.y), (point_b.x - point_a.x))

def _min_angle_between_angles(angle_a, angle_b):
    angle_a = _normalize_rad(angle_a)
    angle_b = _normalize_rad(angle_b)

    possible_angle_as = [angle_a - 2 * math.pi, angle_a, angle_a + 2 * math.pi]
    angle_differences = [angle_b - possible_angle_a for possible_angle_a in possible_angle_as]

    min_abs_diff, idx = min((abs(diff), idx) for (idx, diff) in enumerate(angle_differences))
    return angle_differences[idx]

def _is_spin_clockwise(current_angle, target_angle):
    return _min_angle_between_angles(current_angle, target_angle) < 0

def _are_points_equal(point_a, point_b, tolerance):
    return math.sqrt((point_a.x - point_b.x) ** 2 + (point_a.y - point_b.y) ** 2) < tolerance

def _are_angles_equal(angle_a, angle_b, tolerance):
    return (abs(angle_a - angle_b) < tolerance) or (abs(_normalize_rad(angle_a) - _normalize_rad(angle_b)) < tolerance)

def _normalize_rad(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad <= -math.pi:
        rad += 2 * math.pi
    return rad

def _deg_to_rad(deg):
    return math.pi * deg / 180

def _clamp(val, min_val, max_val):
    if min_val > max_val: raise ValueError
    return max(min_val, min(val, max_val))
