#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler

global height 
height = 0.15

def robot_program():

    th = TrajectoryHandler()

    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)
    pose1 = Pose(
        position=Point(1.0, 0.3, height), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    poses = [start]

    # th.publish_marker_array([pose_l, pose_r])

    for pose in poses:
        th.sequencer.plan(Ptp(goal=pose, vel_scale=0.3, acc_scale=0.3))
        th.sequencer.execute()


if __name__ == '__main__':

    robot_program()