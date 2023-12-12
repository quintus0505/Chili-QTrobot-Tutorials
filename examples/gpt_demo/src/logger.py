#!/usr/bin/env python3
import csv
import os
from datetime import datetime
import time
class Logger:
    def __init__(self):
        home_dir = os.path.expanduser('~')
        self.log_dir = os.path.join(home_dir, "GPTSmach_Log", "state_log")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = os.path.join(self.log_dir, current_time + "_state_log.csv")
        self.log_data = []

    def log_state_time(self, state, start_time):
        """
        Logs the start time of a state.
        :param state: Name of the state.
        :param start_time: Start time of the state.
        """
        # formatted_time = datetime.fromtimestamp(start_time).strftime("%Y-%m-%d %H:%M:%S")
        # self.log_data.append({"state": state, "start_time": formatted_time})
        self.log_data.append({"start_time": start_time, "state": state})

    def save_state_time_to_csv(self):
        """
        Saves the logged data to a CSV file.
        """
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=["start_time", "state"])
            writer.writeheader()
            for data in self.log_data:
                writer.writerow(data)

# Example usage
logger = Logger()

# Logging states with their start time
greeting_start_time = time.time()
logger.log_state_time("Greeting", greeting_start_time)

conversation_start_time = time.time()
logger.log_state_time("Conversation", conversation_start_time)

# Save the log to a CSV file
logger.save_state_time_to_csv()
