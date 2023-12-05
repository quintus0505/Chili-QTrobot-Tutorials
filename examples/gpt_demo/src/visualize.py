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
from std_msgs.msg import Bool
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

        self.running = False
        self.start_time = 0
        self.initial_x = 0
        self.initial_y = 0
        self.xy_positions = []

        self.smooth_factor = 0.1  # Adjust this value to control the smoothing level

    def pygame_init(self):
        # Initialize the Pygame
        pygame.init()
        # Set up the display
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("XY Position Visualization")
        # Initialize Pygame clock
        self.clock = pygame.time.Clock()    
                # Subscribe to the signal topic
        rospy.Subscriber("/qt_executing_signal", Bool, self.start_drawing_callback)
        rospy.spin()  # Keep the program running and listen for the signal

    def handle_signal(self, msg):
        if msg.data:
            print("Received signal to start drawing")
            self.start_drawing()
        else:
            print("Received signal to finish drawing")
            self.running = False

    def start_drawing(self):
        self.finish_signal = False  # Reset the finish signal
        # Clear the screen
        self.screen.fill(WHITE)
        self.running = True
        self.start_time = time.time()  # Get the current time

        self.initial_x = self.group.get_current_pose().pose.position.x
        self.initial_y = self.group.get_current_pose().pose.position.y

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

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

            # Convert the XY positions from meters to pixel coordinates
            x_pixel = 400 + int(x * 100 * 8) * 3  # Map -1.0 to 1.0 meters to 0 to 800 pixels
            y_pixel = 300 + int(y * 100 * 6) * 3 # Map -1.0 to 1.0 meters to 600 to 0 pixels


            self.xy_positions.append((x_pixel, y_pixel))
            # Clear the screen
            self.screen.fill(WHITE)
            # Draw the trajectory in black
            for i in range(1, len(self.xy_positions)):
                pygame.draw.line(self.screen, BLACK, self.xy_positions[i - 1], self.xy_positions[i], 8)

            # Draw the current XY position in red
            pygame.draw.circle(self.screen, RED, (x_pixel, y_pixel), 14)


            pygame.display.flip()
            self.clock.tick(45)

        print("Done")
        # Close Pygame
        pygame.quit()
        sys.exit()


    def start_drawing_callback(self, msg):
        if msg.data:
            print("Received signal to start drawing")
            self.start_drawing()
        else:
            print("Received signal to finish drawing")
            self.finish_signal = True  # Set the global finish signal
            self.clear_trajectory()

    def clear_trajectory(self):
        # Clear the xy_positions list
        self.xy_positions.clear()
        # Fill the screen with white to clear it
        self.screen.fill(WHITE)
        pygame.display.flip()

if __name__ == "__main__":
    visualizer = Visualize()
    visualizer.pygame_init()
    # Subscribe to the signal topic
    # rospy.Subscriber("/qt_executing_signal", Bool, start_drawing_callback, callback_args=visualizer)
    # rospy.spin()  # Keep the program running and listen for the signal

