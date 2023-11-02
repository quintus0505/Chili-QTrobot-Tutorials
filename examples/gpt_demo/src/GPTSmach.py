#!/usr/bin/env python3
import random
import time
import concurrent.futures
import asyncio
import rospy
import text2emotion as te
from utils import aimodel
from std_msgs.msg import String
import nltk
from nltk.tokenize import sent_tokenize, word_tokenize
from nltk.sentiment import SentimentIntensityAnalyzer
import logging
from utils.tools import load_csv_file, save_csv_file
from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *
from GPTBot import Synchronizer, QTChatBot
# from qt_riva_asr_app.srv import *

import smach
import pandas as pd
from datetime import datetime
import sys

class Greeting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['proceed'],
                             input_keys=['GPTBot'],
                             output_keys=['GPTBot'])

    def execute(self, userdata):
        rospy.loginfo("Executing state Greeting")
        userdata.GPTBot.intro()
        userdata.GPTBot.ask_name()
        return 'proceed'

class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye'],
                             input_keys=['GPTBot'], 
                             output_keys=['GPTBot'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
        return 'goodbye'

class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'],
                             input_keys=['GPTBot'], 
                             output_keys=['GPTBot'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Goodbye")
        return 'end'


class GPTSmachManager():
    def __init__(self):
        # Initialize ROS node
        print("GPTSMACH INIT")
        rospy.init_node('qt_gpt_smach', anonymous=True)
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.userdata.gpt_bot = QTChatBot()  # Initialize the GPTBot instance

        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('GREETING', Greeting(),
                                transitions={'proceed':'CONVERSATION'},
                                remapping={'GPTBot':'gpt_bot'})  # Use 'gpt_bot' in remapping
            smach.StateMachine.add('CONVERSATION', Conversation(),
                                transitions={'goodbye':'GOODBYE'},
                                remapping={'GPTBot':'gpt_bot'})  # Use 'gpt_bot' in remapping
            smach.StateMachine.add('GOODBYE', Goodbye(),
                                transitions={'end':'end'},
                                remapping={'GPTBot':'gpt_bot'})  # Use 'gpt_bot' in remapping

        self.sm.set_initial_state(['GREETING'])

        # Execute SMACH plan
        outcome = self.sm.execute()
        rospy.loginfo("OUTCOME: " + outcome)
        rospy.spin()



if __name__ == "__main__":
    print("GPTSMACH TEST")
    try:
        myGPTSmachManager = GPTSmachManager()
    except rospy.ROSInterruptException:
        pass