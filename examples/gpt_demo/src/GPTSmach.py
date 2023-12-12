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
from writing_control import Writing_Control
# from qt_riva_asr_app.srv import *
import threading
import smach
import pandas as pd
from datetime import datetime
import sys
from logger import Logger

TEST_WRITING = True  # Flag to just test the writing part, no interaction with the children
# Available_Letter = ['F', 'X', 'H', 'Q', 'S']
Available_Letter = ['F', 'H', 'Q', 'S']
TEST_LETTER = "F"
WRITTING_REPEAT_TIMES = 2
CONVERSATION_TIME = 120     # set the conversation time to 5 minutes
ADDITIONAL_WRITING_TIME = 60  # set the additional writing time to 5 minutes

class Greeting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['proceed'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger'])    

    def execute(self, userdata):
        rospy.loginfo("Executing state Greeting")
        greeting_start_time = time.time()
        userdata.logger.log_state_time("Greeting", greeting_start_time)
        if not TEST_WRITING:
            intro_response = userdata.GPTBot.intro()
            userdata.logger.log_conversation(time.time(), "QTrobot", intro_response)
            bad_hearing_response = userdata.GPTBot.explain_bad_hearing()
            userdata.logger.log_conversation(time.time(), "QTrobot", bad_hearing_response)
            ask_name_response = userdata.GPTBot.ask_name()
            userdata.logger.log_conversation(time.time(), "QTrobot", ask_name_response)

            while not rospy.is_shutdown() and not userdata.GPTBot.finish:            
                print('waiting for the name') 
                try:
                    start_time = time.time()   
                    recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                    end_time = time.time()
                    api_time = end_time - start_time
                    if recognize_result:
                        userdata.logger.log_conversation(end_time, "Human", recognize_result.transcript)
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
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Greeting")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Greeting")
        return 'proceed'


class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye', 'writing_loop'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'])    
    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
        conversation_start_time = time.time() 
        userdata.logger.log_state_time("Conversation", conversation_start_time)
        if not TEST_WRITING:
            while not rospy.is_shutdown() and not userdata.GPTBot.finish:         
                print('listenting...') 
                try:
                    recognize_result = userdata.GPTBot.recognizeQuestion(language=userdata.GPTBot.defaultlanguage, options='', timeout=0)
                    if recognize_result:
                        print("recognize_result: ", recognize_result)
                    else:
                        print("recognize_result is None")                        
                    if not recognize_result or not recognize_result.transcript:
                        continue
                except:
                    continue
                current_time = time.time()
                used_time = current_time - conversation_start_time
                print('Human:', recognize_result.transcript)
                userdata.logger.log_conversation(current_time, "Human", recognize_result.transcript)
                prompt = recognize_result.transcript
                words = word_tokenize(prompt.lower())
                response = None
                if used_time > CONVERSATION_TIME:
                    prompt = "The children's latest response is: " + prompt + \
                        "Now you should suggest to teach the children how to write letters, you should first response to the children and then start with 'Now let us start to write letters'"
                response = userdata.GPTBot.aimodel.generate(prompt)

                if not response:
                    response = userdata.GPTBot.error_feedback

                refined_response = userdata.GPTBot.refine_sentence(response)
                userdata.logger.log_conversation(current_time, "QTrobot", refined_response)
                userdata.GPTBot.no_guesture_speak(refined_response)

            # userdata.GPTBot.no_guesture_start()
            if userdata.GPTBot.finish and userdata.GPTBot.start_writing:
                return 'writing_loop'
            elif userdata.GPTBot.finish:
                return 'goodbye'
        else:
            userdata.GPTBot.talk("Conversation")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Conversation")
            return 'writing_loop'
        
class WritingLoop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aditional_writing_start'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag'])
    def execute(self, userdata):
        rospy.loginfo("Executing state WritingLoop")
        writing_loop_start_time = time.time()
        userdata.logger.log_state_time("WritingLoop", writing_loop_start_time)
        if not TEST_WRITING:
            prompt = "Now we are going to teach the children how to write letters in " + str(Available_Letter) + "You are going to teach all of them You should start with 'Now let us start' and introducing what you are going to teach, limit in 40 words"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Writing Loop")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Writing Loop")


        for letter in Available_Letter:
            userdata.GPTBot.talk("Here is the letter " + letter)
            userdata.logger.log_conversation(time.time(), "QTrobot", "Here is the letter " + letter)
            if not TEST_WRITING:
                for i in range(WRITTING_REPEAT_TIMES):
                    userdata.WritingControl.writing_prepare_arm()
                    userdata.WritingControl.writing_execution(letter=letter)
            else:
                userdata.WritingControl.writing_prepare_arm()
                userdata.WritingControl.writing_execution(letter=letter)
        return 'aditional_writing_start'
        
    
class AditionalWritingStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['additional_writing'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time'])
    def execute(self, userdata):
        rospy.loginfo("Executing state AditionalWritingStart")
        aditional_writing_start_time = time.time()
        userdata.additional_writing_start_time = aditional_writing_start_time
        userdata.logger.log_state_time("AditionalWritingStart", aditional_writing_start_time)
        # userdata.GPTBot.talk("Writing Start")
        if not TEST_WRITING:
            prompt = "Now teach the children how to write letters in" + str(userdata.teaching_times) + \
            " .You should ask which letter the children want to learn again in " + str(userdata.teaching_times) +  "by saying 'Which letter'"
            response =  userdata.GPTBot.aimodel.generate(prompt)

            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
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
                        userdata.logger.log_conversation(end_time, "Human", recognize_result.transcript)
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
                userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Aditional Writing Start")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Aditional Writing Start")
        userdata.teaching_times+=1
        userdata.WritingFlag = 2
        return 'additional_writing'


class AditionalWriting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['additional_writing_end', "additional_writing"],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state AditionalWriting")
        aditional_writing_time = time.time()
        userdata.logger.log_state_time("AditionalWriting", aditional_writing_time)
        if TEST_WRITING:
            userdata.target_letter = TEST_LETTER

        userdata.GPTBot.talk("Here is the letter you want to learn")
        userdata.logger.log_conversation(time.time(), "QTrobot", "Here is the letter you want to learn")
        #TODO: write the letter 
        userdata.WritingFlag -= 1
        userdata.WritingControl.writing_prepare_arm()
        userdata.WritingControl.writing_execution(letter=userdata.target_letter)
        
        if userdata.WritingFlag == 0:
            userdata.GPTBot.talk("I finished writing")
            userdata.logger.log_conversation(time.time(), "QTrobot", "I finished writing")
            userdata.taught_letters.append(userdata.target_letter)
            return 'additional_writing_end'
        else:
            return 'additional_writing'
        
class AditionalWritingEnd(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goodbye', 'additional_writing_start'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag', 'target_letter', 'teaching_times', 'taught_letters', 'additional_writing_start_time'])
        
    def execute(self, userdata):
        rospy.loginfo("Executing state AditionalWritingEnd")
        aditional_writing_end_time = time.time()
        userdata.logger.log_state_time("AditionalWritingEnd", aditional_writing_end_time)
        userdata.target_letter = None

        if not TEST_WRITING:
            if userdata.additional_writing_start_time - aditional_writing_end_time > ADDITIONAL_WRITING_TIME:
                prompt = "Now you should conclude today you taugh letters" + str(Available_Letter) + "Please only make conclusion at this stage stated with 'Now is time for stop, we have learned' and ended with 'I am sure you have learned a lot'"
                response =  userdata.GPTBot.aimodel.generate(prompt)
                print("response: ", response)
                userdata.GPTBot.talk(response)
                userdata.logger.log_conversation(time.time(), "QTrobot", response)
                return 'goodbye'
            else:
                prompt = "Now you have written the letter. Keep the response short and simple, you should ask the children if they want to learn another letter again or stop for today, please start with 'Cheers, ' or 'Great, '."
                response =  userdata.GPTBot.aimodel.generate(prompt)
                print("response: ", response)
                userdata.GPTBot.talk(response)
                userdata.logger.log_conversation(time.time(), "QTrobot", response)
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
                                userdata.logger.log_conversation(end_time, "Human", recognize_result.transcript)
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
                        userdata.logger.log_conversation(time.time(), "QTrobot", response)
                        return 'additional_writing_start'
                    else:
                        prompt = "The speech to text response is: (" + prompt  + "), it is neither a closing word nor a continue word, you should ask again, explain your bad hearing and verify the answer by the children"
                        response =  userdata.GPTBot.aimodel.generate(prompt)
                        print("response: ", response)
                        userdata.GPTBot.talk(response)
                        userdata.logger.log_conversation(time.time(), "QTrobot", response)

        else:
            if userdata.WritingFlag == 0:
                userdata.GPTBot.talk("Aditional Writing End")
                userdata.logger.log_conversation(time.time(), "QTrobot", "Aditional Writing End")
                return 'goodbye'
            else:
                return 'additional_writing_start'
    


class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'],
                             input_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag','taught_letters'],
                             output_keys=['GPTBot', 'Get_Name_Result', 'WritingControl', 'logger', 'WritingFlag','taught_letters'])
    def execute(self, userdata):
        rospy.loginfo("Executing state Goodbye")
        goodbye_start_time = time.time()
        userdata.logger.log_state_time("Goodbye", goodbye_start_time)
        userdata.WritingControl.writing_end_arm()
        if not TEST_WRITING:
            prompt = "Now we are at the goodbye stage. We are using the google speech to text api to recognize the name of the people you are talking to, the result is" + userdata.Get_Name_Result + "If you get name, you should express your thanks\
            to the person you are talking to, if you cannot get the name, you can just say 'my friend' instead of the name. Please act like you are talking to a person rather than acting based on the command and express your thanks to the person you are talking to. and conclude today you taugh letters" + \
            str(userdata.taught_letters) + "The conversation will end after you made conclusion and express your thanks"
            response =  userdata.GPTBot.aimodel.generate(prompt)
            print("response: ", response)
            userdata.GPTBot.talk(response)
            userdata.logger.log_conversation(time.time(), "QTrobot", response)
        else:
            userdata.GPTBot.talk("Goodbye")
            userdata.logger.log_conversation(time.time(), "QTrobot", "Goodbye")
        userdata.logger.save_state_time_to_csv()
        userdata.logger.save_conversation_to_csv()
        return 'end'


class GPTSmachManager():
    def __init__(self):
        # Initialize ROS node
        print("GPTSMACH INIT")
        rospy.init_node('qt_gpt_smach', anonymous=True)
        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.userdata.gpt_bot = QTChatBot()  # Initialize the GPTBot instance
        self.sm.userdata.get_name_result = ''
        self.sm.userdata.writing_control = Writing_Control()
        self.sm.userdata.writing_flag = 2
        self.sm.userdata.target_letter = None
        self.sm.userdata.teaching_times = 0
        self.sm.userdata.taught_letters = []
        self.sm.userdata.logger = Logger()
        self.sm.userdata.additional_writing_start_time = 0

        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y_%H-%M-%S")

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('GREETING', Greeting(),
                                transitions={'proceed':'CONVERSATION'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('CONVERSATION', Conversation(),
                                transitions={'goodbye':'GOODBYE', 'writing_loop':'WRITING_LOOP'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('GOODBYE', Goodbye(),
                                transitions={'end':'end'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('WRITING_LOOP', WritingLoop(),
                                transitions={'aditional_writing_start':'ADITIONAL_WRITING_START'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('ADITIONAL_WRITING_START', AditionalWritingStart(),
                                transitions={'additional_writing':'ADITIONAL_WRITING'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('ADITIONAL_WRITING', AditionalWriting(),    
                                transitions={'additional_writing_end':'ADITIONAL_WRITING_END', 'additional_writing':'ADITIONAL_WRITING'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            smach.StateMachine.add('ADITIONAL_WRITING_END', AditionalWritingEnd(),
                                transitions={'goodbye':'GOODBYE', 'additional_writing_start':'ADITIONAL_WRITING_START'},
                                remapping={'GPTBot':'gpt_bot', 'Get_Name_Result':'get_name_result', 'WritingControl':'writing_control', 'WritingFlag':'writing_flag'})
            

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