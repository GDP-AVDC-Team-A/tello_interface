/*!*******************************************************************************
 *  \brief      This is the state interface package for Tello Interface.
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
#include <robot_process.h>

#include "socket_tello.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/PointStamped.h>

class StateInterface : public RobotProcess
{
//Constructors and destructors
public:
    StateInterface();
    ~StateInterface();

protected:
    bool resetValues();
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    void get_state();
    std::thread* state_thread;

    std::string drone_namespace;   
    std::string tello_drone_model;
    int tello_drone_id;
    int drone_id;
    //TelloSocketServer* stateSocket;
    StateSocket* stateSocket;
    //Publisher
protected:
    ros::Publisher rotation_pub;
    ros::Publisher speed_pub;
    ros::Publisher accel_pub;
    ros::Publisher imu_pub;
    ros::Publisher battery_pub;
    ros::Publisher temperature_pub;
    ros::Publisher sea_level_pub;
    ros::Publisher altitude_pub;

    geometry_msgs::Vector3Stamped rotation_msg;
    geometry_msgs::TwistStamped speed_msg;
    geometry_msgs::AccelStamped accel_msg;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::BatteryState battery_msg;
    sensor_msgs::Temperature temperature_msg;
    geometry_msgs::PointStamped sea_level_msg;
    geometry_msgs::PointStamped altitude_msg;
};
