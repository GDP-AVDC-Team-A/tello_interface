/*!*******************************************************************************
 *  \brief      This is the IMU interface package for Rotors Simulator.
 *  \authors    Ramon Suarez Fernandez
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
#include <math.h>
#include <cmath>

//tf messages
#include <tf/transform_datatypes.h>

//// ROS  ///////
#include "ros/ros.h"

#include <robot_process.h>
#include "communication_definition.h"

//IMU
#include "sensor_msgs/Imu.h"

//Odometry
#include "nav_msgs/Odometry.h"

#include "std_msgs/Float64.h"
#include "tf/transform_datatypes.h"
#include "eigen_conversions/eigen_msg.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
#include <cmath>
#include <cmath>
#include "mavros/mavros_uas.h"
#include <pluginlib/class_list_macros.h>
#include "cvg_string_conversions.h"
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
class ImuInterface : public RobotProcess
{
    //Constructors and destructors
public:
    ImuInterface();
    ~ImuInterface();

protected:
    bool resetValues();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    std::string drone_namespace;   
    std::string rotors_drone_model;
    int rotors_drone_id; 

protected:
    //Subscribers
    ros::Subscriber rotation_angles_sub;
    ros::Subscriber rotation_angles_sub2;
    void rotationAnglesCallback(const sensor_msgs::Imu &msg);
    // Publishers
    ros::Publisher rotation_angles_pub;
    ros::Publisher imu_data_pub;
    ros::Publisher rotation_angles_radians_pub;
    ros::Publisher imu_data_pub2;    
    bool publishImuData(const sensor_msgs::Imu &ImuData);

    Eigen::Quaterniond quaterniond;
    geometry_msgs::Quaternion quaternion;
    double roll, pitch, yaw;
    ros::Time lastTimeRotation;    
};