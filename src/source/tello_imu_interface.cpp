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

#include "tello_imu_interface.h"
using namespace std;

ImuInterface::ImuInterface()
{
}

ImuInterface::~ImuInterface()
{
}

void ImuInterface::ownSetUp() {
    ros::param::get("~rotors_drone_id", rotors_drone_id);
    ros::param::get("~rotors_drone_model", rotors_drone_model);
}

void ImuInterface::ownStart()
{
    ros::NodeHandle n;
    //Publisher
    rotation_angles_pub = n.advertise<geometry_msgs::Vector3Stamped>("rotation_angles", 1, true);
    imu_data_pub        = n.advertise<sensor_msgs::Imu>("imu",1,true);
    rotation_angles_radians_pub = n.advertise<geometry_msgs::Vector3Stamped>("rotation_angles_radians",1, true);

    //Subscriber
    rotation_angles_sub = n.subscribe("/"+rotors_drone_model+cvg_int_to_string(rotors_drone_id)+"/imu", 1, &ImuInterface::rotationAnglesCallback, this);

    //Standard topics
    rotation_angles_sub2 = n.subscribe("/"+rotors_drone_model+cvg_int_to_string(rotors_drone_id)+"/imu", 1, &ImuInterface::rotationAnglesCallback, this);
    imu_data_pub2         = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu",1,true);    
    //Updating lastTime
    this->lastTimeRotation = ros::Time::now();
}

//Reset
bool ImuInterface::resetValues()
{
    return true;
}

//Run
void ImuInterface::ownRun()
{

}

//Stop
void ImuInterface::ownStop()
{
    rotation_angles_pub.shutdown();
    imu_data_pub.shutdown();
    imu_data_pub2.shutdown();
    rotation_angles_sub2.shutdown();
    rotation_angles_radians_pub.shutdown();
    rotation_angles_sub.shutdown();
}

void ImuInterface::rotationAnglesCallback(const sensor_msgs::Imu &msg)
{
    //deg,   mavwork reference frame
    geometry_msgs::Vector3Stamped RotationAnglesMsgs;
    geometry_msgs::Vector3Stamped RotationAnglesRadiansMsgs;

    RotationAnglesMsgs.header.stamp         = ros::Time::now();
    RotationAnglesRadiansMsgs.header.stamp  = ros::Time::now();

    //convert quaternion msg to eigen
    tf::quaternionMsgToEigen(msg.orientation, quaterniond);

//------ The below code converts ENU (Mavros) frame to NED (Aerostack) frame ------- ///

    //Rotating the frame in x-axis by 180 degrees
    Eigen::Quaterniond BASE_LINK_TO_AIRCRAFT = mavros::UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
    quaterniond = quaterniond*BASE_LINK_TO_AIRCRAFT;

    //Rotating the frame in x-axis by 180 deg and in z-axis by 90 axis (accordance to the new mavros update)
    Eigen::Quaterniond ENU_TO_NED = mavros::UAS::quaternion_from_rpy(M_PI, 0.0, M_PI_2);
    quaterniond = ENU_TO_NED*quaterniond;
//-----------------------------------------------------------------------------------///

    //converting back quaternion from eigen to msg
    tf::quaternionEigenToMsg(quaterniond, quaternion);

    tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Matrix3x3 m(q);

    mavros::UAS::quaternion_to_rpy(quaterniond, roll, pitch, yaw);

   //convert quaternion to euler angels
    m.getEulerYPR(yaw, pitch, roll);

    RotationAnglesRadiansMsgs.vector.x =  roll;
    RotationAnglesRadiansMsgs.vector.y =  pitch;
    RotationAnglesRadiansMsgs.vector.z = -yaw;

    RotationAnglesMsgs.vector.x = (double)(+1)*(roll)*180/M_PI;
    RotationAnglesMsgs.vector.y = (double)(+1)*(pitch)*180/M_PI;
    RotationAnglesMsgs.vector.z = (double)(+1)*(yaw)*180/M_PI;


    //converting the yaw which is in the range from -phi to +phi to 0 to 360 degrees
    if( RotationAnglesMsgs.vector.z < 0 )
        RotationAnglesMsgs.vector.z += 360.0;

    rotation_angles_pub.publish(RotationAnglesMsgs);
    publishImuData(msg);
    rotation_angles_radians_pub.publish(RotationAnglesRadiansMsgs);
}

bool ImuInterface::publishImuData(const sensor_msgs::Imu &ImuData)
{
   sensor_msgs::Imu ImuMsg;

   ImuMsg = ImuData;

   //converting the acceleration and velocity in Mavframe (NED frame)
   ImuMsg.linear_acceleration.y*=-1;
   ImuMsg.linear_acceleration.z*=-1;

   ImuMsg.angular_velocity.y*=-1;
   ImuMsg.angular_velocity.z*=-1;

   
   imu_data_pub.publish(ImuMsg);
   imu_data_pub2.publish(ImuMsg);
   return true;
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "ImuInterface");

    cout<<"[ROSNODE] Starting ImuInterface"<<endl;

    //Vars
    ImuInterface imu_interface;
    imu_interface.setUp();
    imu_interface.start();

    try
    {
        //Read messages
        ros::spin();
        return 1;

    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}