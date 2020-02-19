/*!*******************************************************************************
 *  \brief      This is the camera interface package for Tello Interface.
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

#include "tello_camera_interface.h"

using namespace std;

CameraInterface::CameraInterface(){
}   

CameraInterface::~CameraInterface(){
}

void CameraInterface::ownSetUp() {
    ros::param::get("~tello_drone_id", tello_drone_id);
    ros::param::get("~tello_drone_model", tello_drone_model);
}

void CameraInterface::ownStart(){
    //Publisher
    ros::NodeHandle n;
    camera_pub = n.advertise<sensor_msgs::Image>("sensor_measurement/camera", 1, true);

    this->cameraSocket = new VideoSocket(TELLO_CAMERA_PORT);
}

//Stop
void CameraInterface::ownStop()
{
    camera_pub.shutdown();
}

//Reset
bool CameraInterface::resetValues()
{
    return true;
}

//Run
void CameraInterface::ownRun()
{
    image = this->cameraSocket->getImage();
    
    cout << "ownRun() publishing image" << endl;
    camera_pub.publish(image.toImageMsg());

}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "CameraInterface");

    cout<<"[ROSNODE] Starting CameraInterface"<<endl;

    //Vars
    CameraInterface camera_interface;
    camera_interface.setUp();
    camera_interface.start();
    try
    {
        //Read messages
        while (true)
        {   
            // sleep 10ms
            usleep(10000);
            ros::spinOnce();
            camera_interface.run();
        }
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}