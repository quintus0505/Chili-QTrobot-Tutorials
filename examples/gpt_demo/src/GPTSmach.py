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
                             input_keys=['GPTBot', 'Get_Name_Result'],
                             output_keys=['GPTBot', 'Get_Name_Result'])    

    def execute(self, userdata):
        rospy.loginfo("Executing state Greeting")
        userdata.GPTBot.intro()
        userdata.GPTBot.ask_name()

        while not rospy.is_shutdown() and not userdata.GPTBot.finish:            
            print('waiting for the name') 
            try:
                start_time = time.time()   
                recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                end_time = time.time()
                api_time = end_time - start_time
                if recognize_result:
                    print("recognize_result: ", recognize_result)
                    print("api_time: ", api_time, "input token num: ", len(recognize_result.transcript))
                    userdata.GPTBot.google_speech_data.append((api_time, len(recognize_result.transcript)))
                    break
                else:
                    print("recognize_result is None")                        
                if not recognize_result or not recognize_result.transcript:
                    userdata.GPTBot.bored()
                    continue
            except:
                continue
        userdata.Get_Name_Result = recognize_result.transcript
        prompt = "We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should greet the person again, \
        if you cannot get the name from the reuslt, you could just use 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and greet the person again by asking what you can do for the person"
        response =  userdata.GPTBot.aimodel.generate(prompt)
        print("response: ", response)
        userdata.GPTBot.talk(response)
        return 'proceed'


class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye'],
                             input_keys=['GPTBot', 'Get_Name_Result'],
                             output_keys=['GPTBot', 'Get_Name_Result'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
        userdata.GPTBot.start()
        return 'goodbye'

class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'],
                             input_keys=['GPTBot', 'Get_Name_Result'],
                             output_keys=['GPTBot', 'Get_Name_Result'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Goodbye")
        prompt = "Now we are at the goodbye stage. We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should express your thanks\
        to the person you are talking to, if you cannot get the name, you can just say 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and express your thanks to the person you are talking to"
        response =  userdata.GPTBot.aimodel.generate(prompt)
        print("response: ", response)
        userdata.GPTBot.talk(response)
        return 'end'


class GPTSmachManager():
    def __init__(self):
        # Initialize ROS node
        print("GPTSMACH INIT")
        rospy.init_node('qt_gpt_smach', anonymous=True)
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.userdata.gpt_bot = QTChatBot(log_data=False)  # Initialize the GPTBot instance
        self.sm.userdata.get_name_result = ''

        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('GREETING', Greeting(),
                                transitions={'proceed':'CONVERSATION'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result'})
            smach.StateMachine.add('CONVERSATION', Conversation(),
                                transitions={'goodbye':'GOODBYE'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result'})
            smach.StateMachine.add('GOODBYE', Goodbye(),
                                transitions={'end':'end'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result'})

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