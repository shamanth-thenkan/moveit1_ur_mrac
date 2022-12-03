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
start_srv_req.tsdf_params.voxel_length = 0.02
start_srv_req.tsdf_params.sdf_trunc = 0.04
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale = 1000
start_srv_req.rgbd_params.depth_trunc = 0.5
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
# stop_srv_req.archive_directory = '/dev_ws/src.reconstruction/'
global height
height = 0.19
name = str(height)
stop_srv_req.mesh_filepath = f"/home/usuario/{name}.ply"
# stop_srv_req.normal_filters = [NormalFilterParams(
#                     normal_direction=Vector3(x=0.0, y=0.0, z=1.0), angle=90)]
# stop_srv_req.min_num_faces = 1000



def robot_program():

    ee_name = "D405"

    # rospy.wait_for_service("/start_reconstruction")
    # rospy.loginfo("robot program: waiting for /start_reconstruction srv")
    # start_recon = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
    # stop_recon = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)

    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)
    pose1 = Pose(
        position=Point(0.67889286632, 0.451261641105 , 0.169762287405), orientation=Quaternion(-0.521677595724, 0.593610255501, -0.521677595724, -0.321452697613)
    )
    pose2 = Pose(
        position=Point(0.876743353776, 0.451261641105 , 0.169762287405), orientation=Quaternion(0.772679617537, -0.163136888236, 0.0, 0.61347580583)
    )
    pose3 = Pose(
        position=Point(0.876743353776, -0.0673464548037 , 0.169762287405), orientation=Quaternion(0.0, 0.942679959238, 0.0, -0.333698208642)
    )
    pose4 = Pose(
        position=Point(0.876743353776, -0.585954550712 , 0.169762287405), orientation=Quaternion(-0.772679617537, -0.163136888236, 0.0, 0.61347580583)
    )
    pose5 = Pose(
        position=Point(0.67889286632, -0.585954550712 , 0.169762287405), orientation=Quaternion(-0.779040668006, 0.132726361019, 0.0, 0.612763698895)
    )
    pose6 = Pose(
        position=Point(0.67889286632, -0.0673464548037 , 0.169762287405), orientation=Quaternion(0.677723917658, 0.677723917658, 0.201718346797, 0.201718346797)
    )
    th = TrajectoryHandler()
    th.publish_marker_array([pose1, pose2, pose3, pose4, pose5, pose6])

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
    # resp = start_recon(start_srv_req)

    # if resp:
    #     rospy.loginfo("robot program: reconstruction started successfully")
    # else:
    #     rospy.loginfo("robot program: failed to start reconstruction")

    th.sequencer.plan(Lin(goal=pose2, vel_scale=0.1, acc_scale=0.3))
    th.sequencer.execute()
    th.sequencer.plan(Lin(goal=pose3, vel_scale=0.1, acc_scale=0.3))
    th.sequencer.execute()
    th.sequencer.plan(Lin(goal=pose4, vel_scale=0.1, acc_scale=0.3))
    th.sequencer.execute()
    th.sequencer.plan(Lin(goal=pose5, vel_scale=0.1, acc_scale=0.3))
    th.sequencer.execute()
    th.sequencer.plan(Lin(goal=pose6, vel_scale=0.1, acc_scale=0.3))
    th.sequencer.execute()
    # th.sequencer.plan(Ptp(goal=pose1, vel_scale=0.2, acc_scale=0.3))
    # th.sequencer.execute()
    # Stop reconstruction with service srv_req
    # resp = stop_recon(stop_srv_req)

    th.sequencer.plan(Ptp(goal=start, vel_scale=0.2, acc_scale=0.3))
    th.sequencer.execute()


    # if resp:
    #     rospy.loginfo("robot program: reconstruction stopped successfully")
    # else:
    #     rospy.loginfo("robot program: failed to stop reconstruction")


if __name__ == "__main__":

    robot_program()
