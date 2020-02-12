/*!*******************************************************************************
 *  \brief      This is the command interface package for Tello Interface.
 *  \authors    Rodrigo Pueblas Núñez
 *              Hriday Bavle
 *              Alberto Rodelgo Perales
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/ 

#include <iostream>

//// ROS  ///////
#include "ros/ros.h"
#include "cvg_string_conversions.h"
#include <robot_process.h>

#include "socket_tello.h"

#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/droneCommand.h"

#define ALIVE_INTERVAL 1 //alive interval in seconds

class TelloSocketClient;

class CommandInterface : public RobotProcess
{
//Constructors and destructors
public:
    CommandInterface();
    ~CommandInterface();

protected:
    bool resetValues();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();

    void stay_alive();
    bool alive;
    std::thread* alive_thread;

    std::string drone_namespace;   
    std::string tello_drone_model;
    int tello_drone_id;
    int drone_id;
    CommandSocket* commandSocket;

protected:
    ros::Publisher command_pub;
    ros::Subscriber command_sub;
    void commandCallback(const std_msgs::String &msg);

    // RC COMMAND
    message_filters::Subscriber<geometry_msgs::PoseStamped> roll_pitch_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> altitude_yaw_sub;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sync_policy_velocity;
    message_filters::Synchronizer<sync_policy_velocity> sync;
    void angularVelocityCallback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist);

    // message_filters::Subscriber<droneMsgsROS::dronePitchRollCmd> roll_pitch_dep_sub;
    // message_filters::Subscriber<droneMsgsROS::droneDAltitudeCmd> altitude_dep_sub;
    // message_filters::Subscriber<droneMsgsROS::droneDYawCmd> yaw_dep_sub;
    // typedef message_filters::sync_policies::ApproximateTime<droneMsgsROS::dronePitchRollCmd, droneMsgsROS::droneDAltitudeCmd, droneMsgsROS::droneDYawCmd> sync_policy;
    // message_filters::Synchronizer<sync_policy> synch;
    // void rcCallback(const droneMsgsROS::dronePitchRollCmd& roll_pitch, const droneMsgsROS::droneDAltitudeCmd& altitude, const droneMsgsROS::droneDYawCmd& yaw);
    
    ros::Subscriber command_enum_sub;
    void commandEnumCallback(const droneMsgsROS::droneCommand::ConstPtr& msg);

    std_msgs::String command_msg;
};
