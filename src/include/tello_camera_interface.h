/*!*******************************************************************************
 *  \brief      This is the battery interface package for Rotors Simulator.
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
#include <thread>

//// ROS  ///////
#include "ros/ros.h"
#include <robot_process.h>

#include "tello_driver.hpp"

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <libavutil/frame.h>
#include "h264decoder.hpp"
#include <cv_bridge/cv_bridge.h>

class CameraInterface : public RobotProcess
{
//Constructors and destructors
public:
    CameraInterface();
    ~CameraInterface();

protected:
    bool resetValues();
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    void get_camera();
    std::thread* camera_thread;

    std::string drone_namespace;   
    std::string tello_drone_model;
    int tello_drone_id;
    int drone_id;
    //TelloSocketServer* cameraSocket;

    VideoSocket* cameraSocket;

    std::vector<unsigned char> current_frame;
    H264Decoder decoder_;
    ConverterRGB24 converter_;

    bool first_packet;
    size_t seq_buffer_next_ = 0;
    std::vector<unsigned char> raw_packet;
    void decode_frames();
    void process_packet(size_t r);
    //Publisher
protected:
    ros::Publisher camera_pub;
    ros::Subscriber camera_sub;
    void cameraCallback(const sensor_msgs::Image &msg);

    sensor_msgs::Image camera_msg;
};
