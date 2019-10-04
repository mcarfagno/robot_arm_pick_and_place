#!/usr/bin/env python

import rospy
import math
import sys

import moveit_commander

from moveit_msgs.msg import (
    PlanningScene,
    DisplayTrajectory
)

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Quaternion
)

from ur5_arm.srv import(
    CalculateIK,
    CalculateIKRequest,
    CalculateIKResponse
)

from ur5_arm_gazebo.srv import(
    SpawnTarget,
    SpawnTargetResponse,
    SpawnTargetRequest
)

from tf.transformations import (
    quaternion_from_euler
)

class Commander(object):
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._arm_group = moveit_commander.MoveGroupCommander('arm')
        self._hand_group = moveit_commander.MoveGroupCommander('hand')

        # Create a planned path publisher to show the animation in rviz
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    DisplayTrajectory,
                                                    queue_size=20)

        # Create a scene publisher to push changes to the scene
        self._scene_publisher = rospy.Publisher('/planning_scene',
                                            PlanningScene,
                                            queue_size=1)

        # Add the table to the scene
        self._add_table(0,0,0)

    def _add_table(self,x,y,z):
        '''
        Adds the cafe table to the scene as
        collision object
        '''
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
        self._scene.add_box("cafe_table", p, (0.825, 0.825, 0.825))

    def arm_goto_named_target(self,named_pose):
        '''
        '''
        if (named_pose in self._arm_group.get_named_targets()):
            self._arm_group.set_named_target(named_pose)
            plan = self._arm_group.plan()
            self._arm_group.go(wait=True)

    def gripper_goto_named_target(self,named_pose):
        '''
        '''
        if (named_pose in self._hand_group.get_named_targets()):
            self._hand_group.set_named_target(named_pose)
            plan = self._hand_group.plan()
            self._hand_group.go(wait=True)

    def arm_goto_joint_target(self,joints_values):
        '''
        '''
        if len(joints_values) == 6:
            joint_goal = self._arm_group.get_current_joint_values()
            for index,q in enumerate(joints_values):
                joint_goal[index] = q

            self._arm_group.set_joint_value_target(joint_goal)
            plan=self._arm_group.plan()
            self._arm_group.go(wait=True)
     
def main():
    print("Initializing node... ")
    rospy.init_node("moveit_commander_node")
    print("Running Pick and Place Demo. Ctrl-c to quit")
    
    try:
        rospy.wait_for_service("/spawn_target",5)
        rospy.wait_for_service("/calculate_ik",5)
    except rospy.ROSException as e:
        rospy.logerr("Services are unavaiable...")
        sys.exit(str(e))
    
    # 0- START ARM COMMANDER
    move = Commander()

    # 1- MOVE ROBOT TO PREDEFINED STARTING POSE
    move.arm_goto_named_target("start")
    move.gripper_goto_named_target("open")
    raw_input("Robot moved to starting Position. Press Enter to continue...")

    # 2- SPAWN TARGET AT RANDOM LOCATION
    spawn_target = rospy.ServiceProxy("/spawn_target",SpawnTarget)
    target_pose = spawn_target(SpawnTargetRequest(0)).pose
    raw_input("Target Spawed in position [{},{},{}]. Press Enter to continue...".format(target_pose.position.x,
                                                                                        target_pose.position.y,
                                                                                        target_pose.position.z))

    # 3- COMPUTE INVERSE KINEMATICS
    calculate_ik = rospy.ServiceProxy("/calculate_ik",CalculateIK)
    ik_poses = CalculateIKRequest()
    
    # approach
    goal_pose=Pose()
    goal_pose.position.x = target_pose.position.x
    goal_pose.position.y = target_pose.position.y
    goal_pose.position.z = target_pose.position.z + 0.17
    
    # approach angle
    goal_pose.orientation=Quaternion(*quaternion_from_euler(0.0, 0.0,-1.57))

    # send ik request
    ik_poses.poses.append(goal_pose)
    joint_angles = calculate_ik(ik_poses)
    raw_input("Found IK Solution {}. Press Enter to continue...".format(joint_angles.points[0].positions))

    move.arm_goto_joint_target([0,
                                -math.pi/4,
                                0,
                                -math.pi/2,
                                0,
                                math.pi/3])

    rospy.spin()


if __name__ == "__main__":
    main()