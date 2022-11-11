#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler


def robot_program():

    th = TrajectoryHandler()

    start = th.start
    pose_l = Pose(position=Point(0.6, -0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(0.6, 0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))

    poses = [start, pose_l, pose_r]

    th.publish_marker_array([pose_l, pose_r])

     # attach camera and set new tcp
    th.attach_camera(ee_name)
    th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    rospy.loginfo(
        f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    )

    for pose in poses:
        th.sequencer.plan(Ptp(goal=pose, vel_scale=0.3, acc_scale=0.3))
        th.sequencer.execute()


if __name__ == '__main__':

    robot_program()
