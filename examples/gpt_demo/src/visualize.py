#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
import pygame
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import time
from std_msgs.msg import Bool, Int32
import threading
from writting_control import PEN_RISE, TABLE_HEIGH
import numpy as np
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Global variable to store the finish signal
finish_signal = False

class Visualize:
    def __init__(self):
        self.finish_signal = False
        argv = ['/home/yujun/catkin_ws/src/gpt_demo/src/visualize.py', 'joint_states:=/qt_robot/joints/state']
        moveit_commander.roscpp_initialize(argv)
        try:
            rospy.init_node('qt_visualizer', anonymous=True)
        except:
            pass
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

        rospy.sleep(3)

        self.group.allow_replanning(True)
        self.group.set_pose_reference_frame("base_link")
        self.group.set_planning_time(5.0)
        self.group.clear_path_constraints()
        self.group.clear_pose_targets()
        self.writing = False
        self.running = False
        self.pen_up = False
        self.start_time = 0
        self.initial_x = 0
        self.initial_y = 0
        self.xy_positions = []

        self.smooth_factor = 0.1  # Adjust this value to control the smoothing level
        self.hight = 900
        self.width = 1200
        # Initialize the Pygame
        pygame.init()
        # Set up the display
        self.screen = pygame.display.set_mode((self.width, self.hight))
        pygame.display.set_caption("XY Position Visualization")
        # Initialize Pygame clock
        self.clock = pygame.time.Clock()    

        # Start ROS listener thread
        self.ros_thread = threading.Thread(target=self.ros_listener_thread, daemon=True)
        self.ros_thread.start()

    def handle_signal(self, msg):
        if msg.data==1:
            print("Received signal to start drawing")
            self.running = True
            self.writing = True
            self.clear_trajectory()
        elif msg.data==0:
            print("Received signal to finish drawing")
            self.running = False
            self.writing = False
        elif msg.data==2:
            print("Received signal pen up")
            self.pen_up = True
        elif msg.data==3:
            print("Received signal pen down")
            self.pen_up = False

    def main_loop(self):
        self.finish_signal = False  # Reset the finish signal
        # Clear the screen
        self.screen.fill(WHITE)
        self.running = True
        self.start_time = time.time()  # Get the current time

        self.initial_x = self.group.get_current_pose().pose.position.x
        self.initial_y = self.group.get_current_pose().pose.position.y
        self.initial_z = self.group.get_current_pose().pose.position.z

        try:
            while not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        sys.exit()

                if self.running and self.writing:
                    # Calculate the elapsed time
                    elapsed_time = time.time() - self.start_time

                    # Check if 60 seconds have passed
                    if elapsed_time >= 60:
                        self.running = False
                    if self.finish_signal:
                        self.running = False
                    # Get the current XY position
                    x = self.group.get_current_pose().pose.position.x - self.initial_x
                    y = self.group.get_current_pose().pose.position.y - self.initial_y
                    z = self.group.get_current_pose().pose.position.z
                    # Convert the XY positions from meters to pixel coordinates
                    x_pixel = self.width/2 + int(x * 100 * 8) * 4  # Map -1.0 to 1.0 meters to 0 to 800 pixels
                    y_pixel = self.hight/2 + int(y * 100 * 6) * 4 # Map -1.0 to 1.0 meters to 600 to 0 pixels
                    print(x_pixel, y_pixel, z)
                    if not self.pen_up:
                        self.xy_positions.append((x_pixel, y_pixel))
                    # Clear the screen
                    self.screen.fill(WHITE)
                    # Draw the trajectory in black
                    for i in range(1, len(self.xy_positions)):
                        if np.linalg.norm(np.array(self.xy_positions[i]) - np.array(self.xy_positions[i - 1])) < 50:
                            pygame.draw.line(self.screen, BLACK, self.xy_positions[i - 1], self.xy_positions[i], 8)

                    # Draw the current XY position in red
                    pygame.draw.circle(self.screen, RED, (x_pixel, y_pixel), 14)


                    pygame.display.flip()
                    self.clock.tick(45)
        except Exception as e:
            print(f"Exception occurred: {e}")
            # Graceful exit
            rospy.signal_shutdown("Shutting down")
            self.ros_thread.join()
            print("Done")
            pygame.quit()
            sys.exit()


    def start_drawing_callback(self, msg):
        if msg.data==1:
            print("Received signal to start drawing")
            self.running = True
            self.clear_trajectory()
            self.screen.fill(WHITE)
        elif msg.data==0:
            print("Received signal to finish drawing")
            self.finish_signal = True  # Set the global finish signal
            

    def clear_trajectory(self):
        # Clear the xy_positions list
        self.xy_positions.clear()
        # Fill the screen with white to clear it
        self.screen.fill(WHITE)
        pygame.display.flip()

    def ros_listener_thread(self):
        rospy.Subscriber("/qt_executing_signal", Int32, self.handle_signal)
        rospy.spin()

if __name__ == "__main__":
    visualizer = Visualize()
    visualizer.main_loop()

