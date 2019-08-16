#!/usr/bin/env python

import rospy
import rospkg
import sys
from copy import copy

from gazebo_msgs.srv import(
    SpawnModel,
    SpawnModelRequest,
    DeleteModel,
    DeleteModelRequest,
)

from geometry_msgs.msg import(
    Pose,
    Point,
    Quaternion,
)

from tf.transformations import(
    quaternion_from_euler
)

if __name__=='__main__':

    rospy.init_node("spawn_target_node")
    rospack = rospkg.RosPack()

    try:
        rospy.wait_for_service("gazebo/spawn_urdf_model",5)
        rospy.wait_for_service("gazebo/delete_model",5)
    except rospy.ROSException as e:
        sys.exit(str(e))

    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model",SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model",DeleteModel)

    #Delete if already in scene
    req = DeleteModelRequest("target")
    resp = delete_model(req)

    if not resp.success:
        print(resp.status_message)

    #TODO: randomize pose
    target_pose = Pose(
        Point(1,1,1),
        Quaternion(0,0,0,1)#Quaternion([quaternion_from_euler(0,0,0))
    )

    with open(rospack.get_path("ur5_arm_gazebo") + "/urdf/target.urdf.xacro","r") as urdf:
        target_xml = urdf.read()

    req = SpawnModelRequest("target",target_xml,"",target_pose,"world")
    resp = spawn_model(req)

    if not resp.success:
        print(resp.status_message)