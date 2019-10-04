#!/usr/bin/env python

import rospy
import rospkg
import sys
from copy import copy
from random import randrange

from gazebo_msgs.srv import(
    SpawnModel,
    SpawnModelRequest,
    DeleteModel,
    DeleteModelRequest,
)

from ur5_arm_gazebo.srv import(
    SpawnTarget,
    SpawnTargetResponse
)

from geometry_msgs.msg import(
    Pose,
    Point,
    Quaternion,
)

from tf.transformations import(
    quaternion_from_euler
)

def handle_spawn_target(req):
    """
    Spawns the target in one of 9 prefined locations, picks one random if 0.
    """

    # retrieves locations as a array
    target_locations = rospy.get_param("~locations")

    if len(target_locations) == 0:
        rospy.logerr("target locations list not found in param file!")
        return rospy.ServiceException("target locations list not found in param file!")


    if req.position not in range(0,len(target_locations)):
        rospy.logerr("target position not in valid range!")
        return rospy.ServiceException("target position not in valid range 0-9.")
    else:

        rospack = rospkg.RosPack()
        spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model",SpawnModel)
        delete_model = rospy.ServiceProxy("gazebo/delete_model",DeleteModel)

        # xd = 0.25
        # yd = 0.25
        # table_heigth = 0.83

        # target_locations = {
        #     1:[-xd,yd,table_heigth],  2:[0,yd,table_heigth],  3:[xd,yd,table_heigth],
        #     4:[-xd,0,table_heigth],   5:[0,0,table_heigth],   6:[xd,0,table_heigth],
        #     7:[-xd,-yd,table_heigth], 8:[0,-yd,table_heigth], 9:[xd,-yd,table_heigth]
        # }
        

        if req.position == 0:
            choosen_location = target_locations[randrange(1,len(target_locations))]
        else:
            choosen_location = target_locations[req.position]

        target_spawn_pose = Pose(
            Point(
                choosen_location[0],
                choosen_location[1],
                choosen_location[2]
            ),
            Quaternion(0,0,0,1)
        )

        with open(rospack.get_path("ur5_arm_gazebo") + "/urdf/target.urdf.xacro","r") as urdf:
            target_xml = urdf.read()

        # Delete if already in scene
        req_dm = DeleteModelRequest("target")
        resp_dm = delete_model(req_dm)

        if not resp_dm.success:
            rospy.logwarn(resp_dm.status_message)

        # Spawn urdf in scene
        req_sm = SpawnModelRequest("target",target_xml,"",target_spawn_pose,"world")
        resp_sm = spawn_model(req_sm)

        if not resp_sm.success:
            rospy.logwarn(resp_sm.status_message)
            return rospy.ServiceException(resp_sm.status_message)

        return SpawnTargetResponse(target_spawn_pose)

if __name__=='__main__':

    rospy.init_node("spawn_target_node")

    try:
        rospy.wait_for_service("gazebo/spawn_urdf_model",5)
        rospy.wait_for_service("gazebo/delete_model",5)
    except rospy.ROSException as e:
        sys.exit(str(e))

    server=rospy.Service("spawn_target",SpawnTarget,handle_spawn_target)
    rospy.spin()