#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <ros/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "control_functions.hpp"
#include <errno.h>
#include <string.h>
using namespace std;
// Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
float tol = 2.0;
vector<float> oldDist = {tol, tol, tol, tol};
vector<float> *oldDistPtr = &oldDist;
int avoiding = 0;
// vector<float> getSonars(){
//     vector<float> dist;
//     //opening i2c channel
//     const char *filename = "/dev/i2c-0";
//     int file_i2c;
//     if ((file_i2c = open(filename, O_RDWR)) < 0) {
//         printf("Failed to open the i2c bus");
//     cout << strerror(errno) << endl;
//     }
//     int addr = 0x70;
//     for(int i = addr; i<=0x73; i++){
//         if (ioctl(file_i2c, I2C_SLAVE, i) < 0) {
//             printf("Failed to acquire bus access and/or talk to slave.\n");
//         }
//         unsigned char buffer[2];
//         buffer[0] = 0x51;
//         //writing to sensor to make it take a reading
//         if (write(file_i2c, buffer, 1) != 1) {
//                 printf("Failed to write to the i2c bus.\n");
//         }
//         buffer[0] = 0xe1;
//         //reading in the data from the sensor
//         int readRes = read(file_i2c, buffer, 2);
//         while (readRes != 2) {
//             buffer[0] = 0xe1;
//             readRes = read(file_i2c, buffer, 2);
//             ros::Duration(.05).sleep();
//         }
//         long val = buffer[1];
//         val = (((val >> 8) & 0xff) | (val & 0xff));
//         //if sonars are close to each other, add more sleep duration between readings
//         dist.push_back(val/100.f); //converting ditance from cm to m and long to float
//         ros::Duration(.3).sleep();
//     }
//     return dist;
// }
int avoid(vector<float> input){
    
    
    vector<float> dist; //order: north, east. south, west
    //vector<float> input; //proportion (how close is the object we need to avoid)
    //input = getSonars();
    //int sum = 0;
    float propGain = 5.0;
    float derGain = 2.0;
    float h = 0.0;
    int avoidCounter = 0;

    for(float i : input)
    {
        if(i<tol){
            dist.push_back(i);
            avoidCounter++;
        }
        else
            dist.push_back(tol);
    }
    if(avoidCounter == 0){
        avoiding = 0;
        set_destination(current_waypoint_x, current_waypoint_y, current_waypoint_z, 0);
        return 1;
    }
    else{
        avoiding = 1;
        float netXDist = dist[1] - dist[3];
        float netYDist = dist[0] - dist[2];
        vector<float> der;
        for (int i = 0; i < dist.size(); i++) {
            der.push_back(dist[i] - oldDist[i]);
        }
        float netXDer = der[1] - der[3];
        float netYDer = der[0] - der[2];
        vector<float> velocities = {propGain * netXDist + derGain * netXDer, propGain * netYDist + derGain * netYDer};
        //forward,right,down,left
        //pre-transformation
        if (velocities[0] == 0) {
            set_destination_avoid(current_pose_g.pose.pose.position.x + waypoint_g.pose.position.x,
                            current_pose_g.pose.pose.position.y + velocities[1],
                            current_pose_g.pose.pose.position.z + h, 0);
        } else if (velocities[1] == 0) {
            set_destination_avoid(current_pose_g.pose.pose.position.x + velocities[1],
                            current_pose_g.pose.pose.position.y + waypoint_g.pose.position.y,
                            current_pose_g.pose.pose.position.z + h, 0);
        } else {
            set_destination_avoid(current_pose_g.pose.pose.position.x + velocities[0],
                            current_pose_g.pose.pose.position.y + velocities[1],
                            current_pose_g.pose.pose.position.z + h, 0);
        }
        *oldDistPtr = dist;
        return 1;
    }
    
}

void avoid_cb(const std_msgs::String::ConstPtr& move){
    //vector<float> sonarStuff = {1,3,4,5};
    // avoid(sonarStuff);

    //Constant motion one direction
    set_destination(10,10,1,0);
    ros::Duration(3.0).sleep();
    vector<float> sonarStuff = {.3,1.2,5.0,5.0};
    avoid(sonarStuff);
    ros::Duration(4.0).sleep();
    sonarStuff = {5.0,5.0,5.0,5.0};
    avoid(sonarStuff);
//     avoid(sonarStuff);
//     ros::Duration(0.5).sleep();
//     avoid(sonarStuff);
//     ros::Duration(0.5).sleep();
//     avoid(sonarStuff);
//     ros::Duration(0.5).sleep();
//     avoid(sonarStuff);

//     //accelerating one direction
//     ros::Duration(3).sleep();
//     sonarStuff = {7,7,7,7};
//     avoid(sonarStuff);
//     ros::Duration(0.5).sleep();
//     sonarStuff = {6,7,7,7};
//     avoid(sonarStuff);
//     ros::Duration(0.5).sleep();
//     sonarStuff = {5,7,7,7};
//     avoid(sonarStuff);
//     ros::Duration(0.5).sleep();
//     sonarStuff = {4,7,7,7};
//     avoid(sonarStuff);
// }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "outtaControls");
    ros::NodeHandle nh;
    ros::Time runStart = ros::Time::now();
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    init_publisher_subscriber(nh);
    ros::Subscriber avoidNow = nh.subscribe<std_msgs::String>("AvoidNow", 10, avoid_cb);

    wait4connect();
    cout << "connected" << endl;
    wait4start();
    cout << "started" << endl;
    initialize_local_frame();
    cout << "local frame" << endl;

    ros::spinOnce();
    takeoff(1);
    ros::spin();

   //  setMode(pylonPath); //increased PD gains and decreased tolerance
   //  while(flyingAroundPath)
   //  avoid(getSonars());
   //  setMode(commSwitch); //normal PD gains increased tolerance
   //  while(!modSwitched)
   //  avoid(getSonars());
   // setMode(pylonPath); //increased PD gains and decreased tolerance
   // while(flyingBackAroundPath)
   //  avoid(getSonars());
    
    
    return 0;
}
