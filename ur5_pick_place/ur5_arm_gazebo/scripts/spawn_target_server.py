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

        if req.position not in range(0,10):
            print("target position not in valid range!")
            return SpawnTargetResponse(False)
        else:
            target_locations={
                1:[0,0,0],
                2:[1,0,0],
                3:[2,0,0],
                4:[0,1,0],
                5:[1,1,0],
                6:[2,1,0],
                7:[0,2,0],
                8:[1,2,0],
                9:[2,2,0]
            }

            rospack = rospkg.RosPack()
            spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model",SpawnModel)
            delete_model = rospy.ServiceProxy("gazebo/delete_model",DeleteModel)

            #Delete if already in scene
            req_dm = DeleteModelRequest("target")
            resp_dm = delete_model(req_dm)

            if not resp_dm.success:
                print(resp_dm.status_message)

            if req.position == 0:
                choosen_location = target_locations[randrange(1,10)]
            else:
                choosen_location = target_locations[req.position]

            target_pose = Pose(
                Point(
                    choosen_location[0],
                    choosen_location[1],
                    choosen_location[2]
                ),
                Quaternion(0,0,0,1)#Quaternion([quaternion_from_euler(0,0,0))
            )

            with open(rospack.get_path("ur5_arm_gazebo") + "/urdf/target.urdf.xacro","r") as urdf:
                target_xml = urdf.read()

            req_sm = SpawnModelRequest("target",target_xml,"",target_pose,"world")
            resp_sm = spawn_model(req_sm)

            if not resp_sm.success:
                print(resp_sm.status_message)

            return SpawnTargetResponse(True)

if __name__=='__main__':

    rospy.init_node("spawn_target_node")

    try:
        rospy.wait_for_service("gazebo/spawn_urdf_model",5)
        rospy.wait_for_service("gazebo/delete_model",5)
    except rospy.ROSException as e:
        sys.exit(str(e))

    server=rospy.Service("spawn_target",SpawnTarget,handle_spawn_target)
    rospy.spin()
