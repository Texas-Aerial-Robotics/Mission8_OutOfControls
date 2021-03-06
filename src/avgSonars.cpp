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
std_msgs::String qr;
bool moving = false;
std_msgs::String msg;
vector<float> c;
std::vector<std_msgs::UInt16> plist;
std_msgs::UInt16 cv_points;
vector<float> simSonar;
int up;

void voice_cb(const std_msgs::String::ConstPtr& voice)
{
    std_msgs::String v = *voice;
    msg = v;
}

void qr_cb(const std_msgs::String::ConstPtr& codes){

    qr = *codes;
}

void point_cb(const std_msgs::UInt16::ConstPtr& numPoints){
    std_msgs::UInt16 nPoints = *numPoints;
    int max = 0;
    int min = 0;
    //dropping highest and lowest point
    if(plist.size() == 5){
    	for(int i = 1; i < 5; i++){
    		if(plist[max].data < plist[i].data){
    			max = i;
    		}
    		else if(plist[min].data > plist[i].data){
    			min = i;
    		}
    	}
    	plist[max].data = 0;
    	plist[min].data = 0;
        std_msgs::UInt16 sum;
        sum.data = 0;
        for(int i = 0; i < 5; i++){
            sum.data+=plist[i].data;
            plist.pop_back();
        }
        cv_points.data = sum.data/3;
    }
    plist.push_back(nPoints);
}

vector<float> getSonars(){
	vector<float> dist;
	//opening i2c channel
	const char *filename = "/dev/i2c-0";
	int file_i2c;
	if ((file_i2c = open(filename, O_RDWR)) < 0) {
       	printf("Failed to open the i2c bus");
	}
	int addr = 0x70;
	for(int i = addr; i<=0x70; i++){
	    if (ioctl(file_i2c, I2C_SLAVE, i) < 0) {
	    	printf("Failed to acquire bus access and/or talk to slave.\n");
		cout << strerror(errno) << endl;
		}
		unsigned char buffer[2];
		buffer[0] = 0x51;
		//writing to sensor to make it take a reading
		if (write(file_i2c, buffer, 1) != 1) {
          		printf("Failed to write to the i2c bus.\n");
		}
		buffer[0] = 0xe1;
		//reading in the data from the sensor
		int readRes = read(file_i2c, buffer, 2);
		while (readRes != 2) {
			buffer[0] = 0xe1;
			readRes = read(file_i2c, buffer, 2);
			ros::Duration(.1).sleep();
        }
		long val = buffer[1];
		val = (((val >> 8) & 0xff) | (val & 0xff));
		//if sonars are close to each other, add more sleep duration between readings
		dist.push_back(val/100.f); //converting ditance from cm to m and long to float
		ros::Duration(.1).sleep();
	}
	return dist;
}

int avoid(){
  int readings_to_avg = 3;
	float sum = 0;
	for(i = 0; i < readings_to_avg; i++){
		sum += getSonars()[0];
	}
	vector<float> d; //order: north, east. south, west
	vector<int> m; //whether we need to avoid in a certain direction or not
	vector<float> p; //proportion (how close is the object we need to avoid)
	d.push_back(sum/readings_to_avg);
	int sum = 0;
	float tol = 1.5;
	float min_aviod_distance = .4;
	float k = 1.0;//this is how much you want the drone to move
	float h = 0.0;
	float back_up_mag = -1.5;

	ROS_INFO("Front sonar return %f m", d[0]);

	if(d[0] < tol && d[0] > min_aviod_distance){
		ROS_INFO("\n Obstacle detected on sonar number 1 range %f \n AVIODING \n SKRT SKRT \n", d[0]);
		float alt = 1;
		set_destination(current_pose_g.pose.pose.position.x + back_up_mag*sin(current_heading_g) , current_pose_g.pose.pose.position.y + back_up_mag*cos(current_heading_g) , alt, 0);
		while(ros::ok() && !(check_waypoint_reached(.2))) {
			ros::Duration(.2).sleep();
		}
	}
	else{
		m.push_back(0);
		p.push_back(0);
	}

	if(sum == 0){
		return 0;
	}
	//if surrounded in all 4 directions or in 2 opposite directions

	// if(sum == 4 || (m[3]-m[1] == 0 && m[2]-m[0] != 0) || (m[2]-m[0] == 0 && m[3]-m[1] != 0)){
	// 	if(current_pose_g.pose.pose.position.z >= 1.5){
	// 		h = -.5;
	// 	}
	// 	else{
	// 		h = .5;
	// 	}
	// }


	return 1;

}

float deltaZ(const std_msgs::UInt16 n){
	return .0000000005*[](float x){return x * x * x;}((.2*(n.data-6400)));
}

//to be changed: function will accept a column of a 2D vector
void flyTo(float x, float y, float z){
	set_destination(x,y,z, 0);
	float tol = .2;
	ros::Time start = ros::Time::now();
	while(ros::ok() && !(check_waypoint_reached(tol)) && (ros::Time::now().toSec() - start.toSec() < 60)){
		if(msg.data == "stop"){
			break;
		}
		if(!(avoid())){
			set_destination(x,y,z,0);
			ros::Duration(.5).sleep();
		}
		ros::spinOnce();
		ros::Duration(0.3).sleep();
	}

    ROS_INFO("Done moving to (%f, %f, %f).", x, y, z);

}

//to be changed: function will accept a column of a 2D vector
void QRcode(float x, float y, float z){
	float tol = .2;
	float r = .1;
    float t = 0;

   ros::Time start = ros::Time::now();
   while(ros::ok() && qr.data == "null" && (ros::Time::now().toSec() - start.toSec() < 60)){
   	if(msg.data == "stop"){
		break;
	}
	if(z + deltaZ(cv_points) >= .5 && z + deltaZ(cv_points) <= 1){
		if(!(avoid())){
			ros::Duration(.2).sleep();
			set_destination(x + r*cos(t), y + r*sin(t), z + deltaZ(cv_points), 0);
			t+=.1;
		}
        }
        else{
        	if(!(avoid())){
			ros::Duration(.5).sleep();
			set_destination(x,y,z,0);
		}
        }
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Got QR Code.");
}



int main(int argc, char** argv)
{

	qr.data = "null";
	msg.data = "nothin";
	up = 0;
	bool drone1 = true;
    ros::init(argc, argv, "outtaControls");
    ros::NodeHandle nh;
    ros::Time runStart = ros::Time::now();

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    init_publisher_subscriber(nh);
    ros::Subscriber voiceRecognition = nh.subscribe<std_msgs::String>("Android", 10, voice_cb);
    ros::Subscriber QR = nh.subscribe<std_msgs::String>("CV", 10, qr_cb);
    ros::Subscriber points = nh.subscribe<std_msgs::UInt16>("Points", 5, point_cb);

    wait4connect();
    cout << "connected" << endl;
    wait4start();
    cout << "started" << endl;
    initialize_local_frame();
    cout << "local frame" << endl;

    ros::spinOnce();

    float takeoff_alt = 1;
    takeoff(takeoff_alt);

    set_destination(0, 0, takeoff_alt, 0);

    while(ros::ok()){
    	ros::spinOnce();
    	avoid();
      ros::Duration(.5).sleep();
    }

    return 0;
}
