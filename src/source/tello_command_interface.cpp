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

#include "tello_command_interface.h"

using namespace std;

CommandInterface::CommandInterface() :
    sync (sync_policy_velocity(1)),
    synch (sync_policy(1))
{
}

CommandInterface::~CommandInterface()
{
}

void CommandInterface::ownSetUp()
{
    this->commandSocket = new CommandSocket(TELLO_CLIENT_ADDRESS, TELLO_COMMAND_PORT, PC_COMMAND_PORT);
    
    cout<<"[ROSNODE] Command ownSetup"<<endl;

    ros::param::get("~tello_drone_id", tello_drone_id);
    ros::param::get("~tello_drone_model", tello_drone_model);
}

void CommandInterface::ownStart()
{
    cout<<"[ROSNODE] Command ownSart"<<endl;

    this->commandSocket->send_command("command");
    usleep(200);
    this->commandSocket->send_command("streamon");

    ros::NodeHandle n;
    command_pub = n.advertise<std_msgs::String>("command", 1, true);
    command_sub = n.subscribe("command", 1, &CommandInterface::commandCallback, this);

    // STANDARD
    roll_pitch_sub.subscribe(n, "actuator_command/roll_pitch", 1);
    altitude_yaw_sub.subscribe(n, "actuator_command/altitude_rate_yaw_rate", 1);
    sync.connectInput(roll_pitch_sub, altitude_yaw_sub);
    sync.registerCallback(&CommandInterface::angularVelocityCallback, this);

    //DEPRECATED
    roll_pitch_dep_sub.subscribe(n, "command/pitch_roll", 1);
    altitude_dep_sub.subscribe(n, "command/dAltitude", 1);
    yaw_dep_sub.subscribe(n, "command/dYaw", 1);
    synch.connectInput(roll_pitch_dep_sub, altitude_dep_sub, yaw_dep_sub);
    synch.registerCallback(&CommandInterface::rcCallback, this);

    command_enum_sub = n.subscribe("command/high_level", 1, &CommandInterface::commandEnumCallback, this);

    this->alive_thread = new std::thread(&CommandInterface::stay_alive, this);
}

void CommandInterface::stay_alive()
{
    this->alive = true;
    while (this->alive)
    {
        sleep(ALIVE_INTERVAL);
        command_msg.data = "rc 0 0 0 0";
        command_pub.publish(command_msg);
    }
}

//Stop
void CommandInterface::ownStop()
{
    command_pub.shutdown();
    command_sub.shutdown();
}

//Reset
bool CommandInterface::resetValues()
{
    return true;
}

//Run
void CommandInterface::ownRun()
{

}

void CommandInterface::commandCallback(const std_msgs::String &msg)
{
    cout << "Command: " << msg.data << endl;
    this->commandSocket->send_command(msg.data.c_str());
    command_msg.data = msg.data;
}

void CommandInterface::angularVelocityCallback(const geometry_msgs::PoseStamped& pose_stamped, const geometry_msgs::TwistStamped& twist_stamped)
{
    cout << "[angularVelocityCallback] Starting" << endl;
    double roll = 0, pitch = 0, yaw = 0, altitude = 0;
    tf::Matrix3x3 orientation(tf::Quaternion(pose_stamped.pose.orientation.x,pose_stamped.pose.orientation.y,pose_stamped.pose.orientation.z,pose_stamped.pose.orientation.w));

    orientation.getRPY(roll, pitch, yaw);
/*
    float roll, pitch, yaw, altitude, t0, t1, t2, x, y, z, w;

    x = pose_stamped.pose.orientation.x;
    y = pose_stamped.pose.orientation.y;
    z = pose_stamped.pose.orientation.z;
    w = pose_stamped.pose.orientation.w;

    //roll/pitch: PoseStamped.Pose.Quaternion
    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0, t1);
    t2 = 2.0 * (w * y - z * x);
    if (t2 > 1) t2 = 1;
    if (t2 < -1) t2 = -1;
    pitch = asin(t2);
    */

    //yaw: TwistStamped.Twist.angular
    yaw = twist_stamped.twist.angular.z;
    //altitude: TwistStamped.Twist.linear
    altitude = twist_stamped.twist.linear.z;


    std::ostringstream rc;
    rc << "rc " << static_cast<int>(round(roll * 100))
    << " " << static_cast<int>(round(-pitch * 100))
    << " " << static_cast<int>(round(altitude * 100))
    << " " << static_cast<int>(round(yaw * 100));

    this->commandSocket->send_command(rc.str().c_str());
    cout << "[angularVelocityCallback] Command: " << rc.str() << endl;
}

void CommandInterface::rcCallback(const droneMsgsROS::dronePitchRollCmd& roll_pitch, const droneMsgsROS::droneDAltitudeCmd& altitude, const droneMsgsROS::droneDYawCmd& yaw)
{
    cout << "[rcCallback] Starting" << endl;
    std::ostringstream rc;
    rc << "rc " << static_cast<int>(round(roll_pitch.rollCmd * 100))
    << " " << static_cast<int>(round(-roll_pitch.pitchCmd * 100))
    << " " << static_cast<int>(round(altitude.dAltitudeCmd * 100))
    << " " << static_cast<int>(round(yaw.dYawCmd * 100));

    this->commandSocket->send_command(rc.str().c_str());
    cout << "[rcCallback] Command: " << rc.str() << endl;
}

void CommandInterface::commandEnumCallback(const droneMsgsROS::droneCommand::ConstPtr& msg)
{
    std::string response;
    switch(msg->command)
    {
    case droneMsgsROS::droneCommand::TAKE_OFF:
        cout << "[commandEnumCallback] Taking off" << endl;
        this->commandSocket->send_command("takeoff");
        break;
    case droneMsgsROS::droneCommand::LAND:
        cout << "[commandEnumCallback] Landing" << response << endl;
        this->commandSocket->send_command("land");
        break;
    case droneMsgsROS::droneCommand::HOVER:
        cout << "[commandEnumCallback] Hovering" << response << endl;
        this->commandSocket->send_command("rc 0 0 0 0");
        break;
    case droneMsgsROS::droneCommand::EMERGENCY_STOP:
        cout << "[commandEnumCallback] Emergency" << response << endl;
        this->commandSocket->send_command("emergency");
        break;
    default:
        break;
    }

    return;
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "CommandInterface");

    cout<<"[ROSNODE] Starting CommandInterface"<<endl;

    //Vars
    CommandInterface command_interface;
    command_interface.setUp();
    command_interface.start();
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