/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <tinyxml.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <motors_custom_controller/motors_custom_controller.h>
#include <pluginlib/class_list_macros.hpp>

using namespace boost::filesystem;
using namespace motors_custom_controller;





QTCustomController::QTCustomController():shouldPlay(false) {}


QTCustomController::~QTCustomController() {}

bool QTCustomController::init(QTMotorInterface* hw, ros::NodeHandle& root_nh,
                              ros::NodeHandle& controller_nh) {

    QTCustomController::hw = hw;

    //std::string myparam;
    //if(!root_nh.getParam("/qt_robot/custom_controller/myparam", mypapram)) {
    //    ROS_WARN_STREAM("QTCustomController: Missing param 'myparam'!");
    //}

    std::vector<HerkulexParams>& motors = hw->getMotors();
    for(int i=0; i<motors.size(); i++) {
        if(motors[i].part == "head")
            jointStateHead.push_back(hw->getJointStateInterface().getHandle(motors[i].name));
        else if(motors[i].part == "right_arm")
            jointStateRarm.push_back(hw->getJointStateInterface().getHandle(motors[i].name));
        else if(motors[i].part == "left_arm")
            jointStateLarm.push_back(hw->getJointStateInterface().getHandle(motors[i].name));
    }

    serviceStartStop = controller_nh.advertiseService("startstop", &QTCustomController::startStopCB, this);

    // generate a simple trajectory to be played by QTroobot
    generateMyTrajectory();

    return true;
}

void QTCustomController::update(const ros::Time& time, const ros::Duration& period) {

    bool flag;
    mutexUpdate.lock();
    flag = shouldPlay;
    mutexUpdate.unlock();

    static size_t index = 0;

    if(flag) {
        // a true controller schema: read motors feedback, calculate, co0mmand motors
        // example : read joint values (postion, velocity, effort)
        /*
        for(size_t i=0; i<jointStateHead.size(); i++)
            ROS_INFO_STREAM(jointStateHead[i].getName() << ": " << jointStateHead[i].getPosition());
        for(size_t i=0; i<jointStateRarm.size(); i++)
            ROS_INFO_STREAM(jointStateRarm[i].getName() << ": " << jointStateRarm[i].getPosition());
        for(size_t i=0; i<jointStateLarm.size(); i++)
            ROS_INFO_STREAM(jointStateLarm[i].getName() << ": " << jointStateLarm[i].getPosition());
        */

        // playing my custom trajectory
        QTMotorInterface::JointsRawCommand commmand;
        commmand["RightShoulderPitch"] = my_RightShoulderPitch_cmd[index];
        commmand["LeftShoulderPitch"] = my_LeftShoulderPitch_cmd[index];
        // write to motor
        ros::Duration duration(0.1);
        // make the first move slower
        if(index == 0)
            duration.fromSec(2.0);
        hw->moveMotorsRaw(commmand, duration.toNSec()/1e6); //nsec to milliseconds
        duration.sleep();
        index = (index < my_RightShoulderPitch_cmd.size()-1) ? index+1 : 0;
    }
}


bool QTCustomController::startStopCB(motors_custom_controller::StartStop::Request  &req,
                                      motors_custom_controller::StartStop::Response &res) {
    mutexUpdate.lock();
    // adjust the main loop frequency to 20 hz
    //hw->changeMainLoopFreq(20);
    shouldPlay = req.command;
    mutexUpdate.unlock();

    res.status = true;
    return true;
}

void QTCustomController::generateMyTrajectory(void) {
    for (double value=90.0; value>0.0; value-=5) {
        my_RightShoulderPitch_cmd.push_back(value);
        my_LeftShoulderPitch_cmd.push_back(value);
    }
    for (double value=0.0; value<90.0; value+=5) {
        my_RightShoulderPitch_cmd.push_back(value);
        my_LeftShoulderPitch_cmd.push_back(value);
    }
}

PLUGINLIB_EXPORT_CLASS(motors_custom_controller::QTCustomController,controller_interface::ControllerBase)
