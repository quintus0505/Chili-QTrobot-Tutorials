#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
from std_msgs.msg import Bool
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list


class Writting_Control():
    def __init__(self):
        self.eef_step = 0.0001
        argv = ['/home/yujun/catkin_ws/src/gpt_demo/src/writting.py', 'joint_states:=/qt_robot/joints/state']
        moveit_commander.roscpp_initialize(argv)
        try:
            rospy.init_node('writting', anonymous=True)
        except:
            pass
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.plan = None

        self.signal_publisher = rospy.Publisher("/qt_executing_signal", Bool, queue_size=1)
        rospy.sleep(3)

            # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        print("============ Printing robot state")
        print("current pose: \n {}".format(self.group.get_current_pose().pose))
        print("current pose reference frame: {}".format(self.group.get_pose_reference_frame()))

        self.group.allow_replanning(True)
        self.group.set_pose_reference_frame("base_link")
        self.group.set_planning_time(5.0)
        self.group.clear_path_constraints()
        self.group.clear_pose_targets()

    def publish_signal(self, signal):
        if signal == "start":
            print("publishing start")
            self.signal_publisher.publish(Bool(True))  # Publish "start" signal
        elif signal == "finish":
            print("publishing finish")
            self.signal_publisher.publish(Bool(False))  # Publish "finish" signal

    def writting_prepare_arm(self):
        self.group.set_start_state_to_current_state()
        self.group.set_position_target([0.18, -0.25, 0.35])
        self.plan = self.group.go(wait=True)
        self.group.set_position_target([0.23, -0.23, 0.35])
        self.plan = self.group.go(wait=True)
        print("Reached starting point")

    def writting_execution(self):

        # generate waypoints
        waypoints = []
        wpose = self.group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        # Publish a signal on the topic
        self.publish_signal("start")

        for i in range(1):
            wpose.position.y += 0.03
            waypoints.append(copy.deepcopy(wpose))
            wpose.position.x += -0.03
            waypoints.append(copy.deepcopy(wpose))
            wpose.position.y += -0.03
            waypoints.append(copy.deepcopy(wpose))
            wpose.position.x += 0.03
            waypoints.append(copy.deepcopy(wpose))

        # plan trajectory
        self.plan, fraction = self.group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    self.eef_step,        # eef_step
                                    0.0,         # jump_threshold
                                    False)       # avoid_collisions
        
        # execute the plan
        self.group.execute(self.plan, True)
        print("done")
        self.publish_signal("finish")


# if __name__ == "__main__":
#     eef_step = 0.0001
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('draw_rectangle', anonymous=True)

#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group = moveit_commander.MoveGroupCommander("right_arm")
#     # Create a publisher to send the signal
#     signal_publisher = rospy.Publisher("/qt_executing_signal", Bool, queue_size=1)
#     rospy.sleep(3)

#     # We can get the name of the reference frame for this robot:
#     planning_frame = group.get_planning_frame()
#     print ("============ Reference frame: %s" % planning_frame)

#     # We can also print the name of the end-effector link for this group:
#     eef_link = group.get_end_effector_link()
#     print ("============ End effector: %s" % eef_link)

#     # We can get a list of all the groups in the robot:
#     group_names = robot.get_group_names()
#     print ("============ Robot Groups:", robot.get_group_names())

#     # Sometimes for debugging it is useful to print the entire state of the
#     print("============ Printing robot state")
#     print("current pose: \n {}".format(group.get_current_pose().pose))
#     print("current pose reference frame: {}".format(group.get_pose_reference_frame()))

#     group.allow_replanning(True)
#     group.set_pose_reference_frame("base_link")
#     group.set_planning_time(5.0)
#     group.clear_path_constraints()
#     group.clear_pose_targets()

#     # move to starting point
#     group.set_start_state_to_current_state()

#     # # draw on ZY plane
#     # ###################
#     # # go to start pose
#     # group.set_position_target([0.18, -0.25, 0.35])
#     # plan = group.go(wait=True)
#     # print("Reached starting point")
    
#     # # generate waypoints
#     # waypoints = []
#     # wpose = group.get_current_pose().pose
#     # waypoints.append(copy.deepcopy(wpose))
#     # for i in range(5):
#     #     wpose.position.z += -0.05
#     #     waypoints.append(copy.deepcopy(wpose))
#     #     wpose.position.y += 0.05
#     #     waypoints.append(copy.deepcopy(wpose))
#     #     wpose.position.z += 0.05
#     #     waypoints.append(copy.deepcopy(wpose))
#     #     wpose.position.y += -0.05
#     #     waypoints.append(copy.deepcopy(wpose))

#     # draw on XY plane
#     ####################
#     group.set_position_target([0.18, -0.25, 0.35])
#     plan = group.go(wait=True)
#     group.set_position_target([0.23, -0.23, 0.35])
#     plan = group.go(wait=True)
#     print("Reached starting point")

#     # generate waypoints
#     waypoints = []
#     wpose = group.get_current_pose().pose
#     waypoints.append(copy.deepcopy(wpose))

#     # Publish a signal on the topic
#     publish_signal("start", signal_publisher)

#     for i in range(1):
#         wpose.position.y += 0.03
#         waypoints.append(copy.deepcopy(wpose))
#         wpose.position.x += -0.03
#         waypoints.append(copy.deepcopy(wpose))
#         wpose.position.y += -0.03
#         waypoints.append(copy.deepcopy(wpose))
#         wpose.position.x += 0.03
#         waypoints.append(copy.deepcopy(wpose))

#     # plan trajectory
#     plan, fraction = group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    eef_step,        # eef_step
#                                    0.0,         # jump_threshold
#                                    False)       # avoid_collisions
    
#     # execute the plan
#     group.execute(plan, True)
#     print("done")
#     publish_signal("finish", signal_publisher)
