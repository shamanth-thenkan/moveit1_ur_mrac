#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from industrial_reconstruction_msgs.msg import NormalFilterParams
from industrial_reconstruction_msgs.srv import (StartReconstruction,
                                                StartReconstructionRequest,
                                                StopReconstruction,
                                                StopReconstructionRequest)
from move_group_sequence.move_group_sequence import (Circ, Lin, Ptp, Sequence,
                                                     from_euler)
from trajectory_tools.trajectory_handler import TrajectoryHandler

# reconstruction parameters
start_srv_req = StartReconstructionRequest()
start_srv_req.tracking_frame = "tool0"
start_srv_req.relative_frame = "base_link"
start_srv_req.translation_distance = 0.0
start_srv_req.rotational_distance = 0.0
start_srv_req.live = True
start_srv_req.tsdf_params.voxel_length = 0.2
start_srv_req.tsdf_params.sdf_trunc = 0.4
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale =1000
start_srv_req.rgbd_params.depth_trunc = 0.5
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
stop_srv_req.archive_directory = '/dev_ws/src.reconstruction/'

global acl
acl = 0.3
global vel
vel = 0.1
global height
height = 0.15
name = str(height)
stop_srv_req.mesh_filepath = f"/home/usuario/{name}.ply"
stop_srv_req.normal_filters = [NormalFilterParams(
                    normal_direction=Vector3(x=0.0, y=0.0, z=1.0), angle=90)]
stop_srv_req.min_num_faces = 500



def robot_program():

    ee_name = "D405"

    rospy.wait_for_service("/start_reconstruction")
    rospy.loginfo("robot program: waiting for /start_reconstruction srv")
    start_recon = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
    stop_recon = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)

    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)
    pose0 = Pose(
        position=Point(1.0, 0.5 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose1 = Pose(
        position=Point(0.8, 0.5 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose2 = Pose(
        position=Point(0.8, 0.35 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose3 = Pose(
        position=Point(1.0, 0.35 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose4 = Pose(
        position=Point(1.0, 0.2 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose5 = Pose(
        position=Point(0.8, 0.2 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose6 = Pose(
        position=Point(0.8, 0.05 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose7 = Pose(
        position=Point(1.0, 0.05 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose8 = Pose(
        position=Point(1.0, -0.1 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose9 = Pose(
        position=Point(0.8, -0.1 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose10 = Pose(
        position=Point(0.8, -0.25 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose11 = Pose(
        position=Point(1.0, -0.25 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose12 = Pose(
        position=Point(1.0, -0.4 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    pose13 = Pose(
        position=Point(0.8, -0.4 , 0.15), orientation=Quaternion(0.707106781187, 0.707106781187, 0.0, 0.0)
    )
    th = TrajectoryHandler()
    th.publish_marker_array([pose0,pose1, pose2, pose3, pose4, pose5, pose6,pose7,pose8,pose9,pose10,pose11,pose12,pose13])

    # attach camera and set new tcp
    th.attach_camera(ee_name)
    th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    rospy.loginfo(
        f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    )

    # Move into position to start reconstruction
    th.sequencer.plan(Ptp(goal=start, vel_scale=0.2, acc_scale=0.3))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose1, vel_scale=0.2, acc_scale=0.3))
    th.sequencer.execute()

    # Start reconstruction with service srv_req
    resp = start_recon(start_srv_req)

    if resp:
        rospy.loginfo("robot program: reconstruction started successfully")
    else:
        rospy.loginfo("robot program: failed to start reconstruction")

    th.sequencer.plan(Ptp(goal=pose1, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose2, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose3, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose4, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose5, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose6, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose7, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose8, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose9, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose10, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose11, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose12, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose13, vel_scale=vel, acc_scale=acl))
    th.sequencer.execute()

    # Stop reconstruction with service srv_req
    resp = stop_recon(stop_srv_req)

    th.sequencer.plan(Ptp(goal=start, vel_scale=0.2, acc_scale=0.3))
    th.sequencer.execute()

    if resp:
        rospy.loginfo("robot program: reconstruction stopped successfully")
    else:
        rospy.loginfo("robot program: failed to stop reconstruction")


if __name__ == "__main__":

    robot_program()
