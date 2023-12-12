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
from writting_control import Writting_Control
# from qt_riva_asr_app.srv import *
import threading
import smach
import pandas as pd
from datetime import datetime
import sys

TEST_WRITING = False  # Flag to just test the writing part, no interaction with the children
Available_Letter = ['F', 'X', 'H', 'Q', 'S']
TEST_LETTER = "F"

class Greeting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['proceed'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize'])    

    def execute(self, userdata):
        rospy.loginfo("Executing state Greeting")
        if not TEST_WRITING:
            userdata.GPTBot.intro()
            userdata.GPTBot.explain_bad_hearing()
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
                        # userdata.GPTBot.bored()
                        continue
                except:
                    continue
            userdata.Get_Name_Result = recognize_result.transcript
            prompt = "We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should greet the person again, \
            if you cannot get the name from the reuslt, you could just use 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and greet the person again by asking what you can do for the person"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
        else:
            userdata.GPTBot.talk("Greeting")
        return 'proceed'


class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye', 'writing_start'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
        if not TEST_WRITING:
            userdata.GPTBot.no_guesture_start()
            if userdata.GPTBot.finish and userdata.GPTBot.start_writing:
                return 'writing_start'
            elif userdata.GPTBot.finish:
                return 'goodbye'
        else:
            userdata.GPTBot.talk("Conversation")
            return 'writing_start'
    
class WritingStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['writing'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag', 'target_letter', 'teaching_times', 'taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag', 'target_letter', 'teaching_times', 'taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state WritingStart")
        # userdata.GPTBot.talk("Writing Start")
        if not TEST_WRITING:
            prompt = "Now teach the children how to write, previously you have taught this children" + str(userdata.teaching_times) + "times, and you the letters you have taught are in this list: " + \
            str(userdata.target_letter) + " .You should first ask which letter the children want to learn by saying 'Which letter do you want to learn?'" + \
            "You should tell the children currently you can only write letters in" + str(Available_Letter) + "You should start with 'I can only write letters'"
            response =  userdata.GPTBot.aimodel.generate(prompt)

            print("response: ", response)
            userdata.GPTBot.talk(response)

            # listen to the children's answer
            userdata.GPTBot.finish = False
            while not rospy.is_shutdown() and not userdata.GPTBot.finish:
                print('waiting for the letter') 
                try:
                    start_time = time.time()   
                    recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                    end_time = time.time()
                    api_time = end_time - start_time
                    if recognize_result:
                        print("recognize_result: ", recognize_result)
                        print("api_time: ", api_time, "input token num: ", len(recognize_result.transcript))
                        userdata.GPTBot.google_speech_data.append((api_time, len(recognize_result.transcript)))
                    else:
                        print("recognize_result is None")                        
                    if not recognize_result or not recognize_result.transcript:
                        # userdata.GPTBot.bored()
                        continue
                except:
                    continue

                print('Human:', recognize_result.transcript)
                prompt = recognize_result.transcript
                # userdata.GPTBot.show_sentiment(userdata.GPTBot.get_sentiment(prompt))
                words = word_tokenize(prompt.lower())
                print("words: ", words)
                # if any of the letter in Available_Letter appears in the words, no matter upper or lower case, we will take it as the target letter
                if any(word in Available_Letter for word in words) or any(word in [letter.lower() for letter in Available_Letter] for word in words):
                    for word in words:
                        if word in Available_Letter:
                            userdata.target_letter = word.upper()
                            break
                        elif word in [letter.lower() for letter in Available_Letter]:
                            userdata.target_letter = word.upper()
                            break

                response = None
                input_prompt = "The speech to text response is: (" + prompt  + ") Your task in this state is to get the letter which is available in" + str(Available_Letter) + \
                "If you get the letter, you should say 'Great, Let us start to write the letter', or you should ask again, explain your bad hearing and verify the answer by the children"

                #     response = results[0]   
                response =  userdata.GPTBot.aimodel.generate(input_prompt)
                print("api_time: ", api_time, "input token num: ", len(words))
                if not response:
                    response = userdata.GPTBot.error_feedback
                if userdata.target_letter:
                    break
                userdata.GPTBot.talk(userdata.GPTBot.refine_sentence(response))
        else:
            userdata.GPTBot.talk("Writing Start")
        userdata.teaching_times+=1
        userdata.WrittingFlag = 2
        return 'writing'


class Writing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['writing_end', "writting"],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag', 'target_letter', 'teaching_times', 'taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag', 'target_letter', 'teaching_times', 'taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state Writing")
        if TEST_WRITING:
            userdata.target_letter = TEST_LETTER

        userdata.GPTBot.talk("Here is the letter you want to learn")
        #TODO: write the letter 
        userdata.WrittingFlag -= 1
        userdata.WrittingControl.writting_prepare_arm()
        userdata.WrittingControl.writting_execution(letter=userdata.target_letter)
        
        if userdata.WrittingFlag == 0:
            userdata.GPTBot.talk("I finished writing")
            userdata.taught_letters.append(userdata.target_letter)
            return 'writing_end'
        else:
            return 'writting'
        
class WritingEnd(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye', 'writing_start'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag', 'target_letter', 'teaching_times', 'taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag', 'target_letter', 'teaching_times', 'taught_letters'])
        
    def execute(self, userdata):
        rospy.loginfo("Executing state WritingEnd")
        userdata.target_letter = None

        if not TEST_WRITING:
            prompt = "Now you have written the letter. Keep the response short and simple, you should ask the children if they want to learn another letter or stop for today, please start with 'Cheers, ' or 'Great, '."
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.GPTBot.finish = False
            # listen to the children's answer
            while not rospy.is_shutdown():
                while not rospy.is_shutdown() and not userdata.GPTBot.finish:
                    print('waiting for the answer') 
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
                            # userdata.GPTBot.bored()
                            continue
                    except:
                        continue

                prompt = recognize_result.transcript
                words = word_tokenize(prompt.lower())

                closing_words = ["bye","goodbye","stop", 'end']
                continue_words = ["continue", "another", "more", "next", "other"]
                if any(word in closing_words for word in words):
                    return 'goodbye'
                elif any(word in continue_words for word in words):
                    prompt = "Now the chidren want to learn another letter, you should say start with 'OK, Let us'"
                    response =  userdata.GPTBot.aimodel.generate(prompt)
                    print("response: ", response)
                    userdata.GPTBot.talk(response)
                    return 'writing_start'
                else:
                    prompt = "The speech to text response is: (" + prompt  + "), it is neither a closing word nor a continue word, you should ask again, explain your bad hearing and verify the answer by the children"
                    response =  userdata.GPTBot.aimodel.generate(prompt)
                    print("response: ", response)
                    userdata.GPTBot.talk(response)

        else:
            if userdata.WrittingFlag == 0:
                userdata.GPTBot.talk("Writing End")
                return 'goodbye'
            else:
                return 'writing_start'
    


class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag','taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WrittingControl', 'Visualize', 'WrittingFlag','taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state Goodbye")
        userdata.WrittingControl.writting_end_arm()
        if not TEST_WRITING:
            prompt = "Now we are at the goodbye stage. We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should express your thanks\
            to the person you are talking to, if you cannot get the name, you can just say 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and express your thanks to the person you are talking to. and conclude today you taugh letters" + \
            str(userdata.taught_letters) + "The conversation will end after you made conclusion and express your thanks"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
        else:
            userdata.GPTBot.talk("Goodbye")
        return 'end'


class GPTSmachManager():
    def __init__(self):
        # Initialize ROS node
        print("GPTSMACH INIT")
        rospy.init_node('qt_gpt_smach', anonymous=True)
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.userdata.gpt_bot = QTChatBot(log_data=False)  # Initialize the GPTBot instance
        self.sm.userdata.get_name_result = ''
        self.sm.userdata.writting_control = Writting_Control()
        self.sm.userdata.writting_flag = 2
        self.sm.userdata.target_letter = None
        self.sm.userdata.teaching_times = 0
        self.sm.userdata.taught_letters = []

        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('GREETING', Greeting(),
                                transitions={'proceed':'CONVERSATION'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WrittingControl':'writting_control', 'WrittingFlag':'writting_flag'})
            smach.StateMachine.add('CONVERSATION', Conversation(),
                                transitions={'goodbye':'GOODBYE', 'writing_start':'WRITTING_START'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WrittingControl':'writting_control', 'WrittingFlag':'writting_flag'})
            smach.StateMachine.add('GOODBYE', Goodbye(),
                                transitions={'end':'end'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WrittingControl':'writting_control', 'WrittingFlag':'writting_flag'})
            smach.StateMachine.add('WRITTING_START', WritingStart(),
                                transitions={'writing':'WRITING'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WrittingControl':'writting_control', 'WrittingFlag':'writting_flag'})
            smach.StateMachine.add('WRITING', Writing(),    
                                transitions={'writing_end':'WRITING_END', 'writting':'WRITING'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WrittingControl':'writting_control', 'WrittingFlag':'writting_flag'})
            smach.StateMachine.add('WRITING_END', WritingEnd(),
                                transitions={'goodbye':'GOODBYE', 'writing_start':'WRITTING_START'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WrittingControl':'writting_control', 'WrittingFlag':'writting_flag'})
            

        self.sm.set_initial_state(['GREETING'])

        # Execute SMACH plan
        outcome = self.sm.execute()
        rospy.loginfo("OUTCOME: " + outcome)
        # rospy.spin()



if __name__ == "__main__":
    print("GPTSMACH TEST")
    try:
        myGPTSmachManager = GPTSmachManager()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Shutting down...")