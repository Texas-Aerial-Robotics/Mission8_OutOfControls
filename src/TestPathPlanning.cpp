#include <vector>
#include<ros/ros.h>


using namespace std;

vector<vector<float>> relWpts = {{1, 1, 0}, {1, 1, 0}, {5, 0, 0}, {1, -1, 0}, {-1, -1, 0}, {-5, 0, 0}};
vector<vector<float>> absWpts = {{1, 1, 0}, {2, 2, 0}, {7, 2, 0}, {8, 1, 0}, {7, 0, 0}, {2, 0, 0}};

void flyToLocal(vector<float>& in){
    set_destination_local(in[0], in[1], in[2]);
    float tol = 1;
    while(ros::Ok() && !destReached(tol))
    {
        spinOnce();
        ros::Duration(.1).sleep();
    }
}

void pathPlanning(vector<vector<float>>& wptVect, int numLoops)
{
    for(int i=0; i<numLoops;++i)
    {
        for(vector<float> wp : wptVect)
        {
            flyToLocal(wp);
        }
    }
    ros::Duration.sleep(1);
    flyTo(0,0,1);
}

/**
 * Choose which ever set of waypoints we want inside this function
 * Not sure whether to use relative waypoints or absolute waypoints
 * */

void path_cb(const std_mesgs::String::ConstPtr& str)
{
    pathPlanning(relWpts, 4);
}

int main()
{
    ros::init(argc, argv, "outtaControls");
    ros::NodeHandle nh;
    ros::Time runStart = ros::Time::now();
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    init_publisher_subscriber(nh);
    ros::Subscriber pathPlanner = nh.subscribe<std_msgs::String>("start flight path", 10, path_cb);

    wait4connect();
    cout << "connected" << endl;
    wait4start();
    cout << "started" << endl;
    initialize_local_frame();
    cout << "local frame" << endl;

    ros::spinOnce();
    takeoff(1);
    ros::spin();

    return 0;    
}
/**
 * 
 * change to relative points
 * publish through the local_pos_publisher
 * set_dest_local will take a vector of floats <x,y,z>
 * check waypoint reached in control functions.hpp
 * PoseStamped will have position and angles
 * https://github.com/Texas-Aerial-Robotics/Mission8_OutOfControls/blob/master/src/obstacle_avoidance.cpp
 * 
 * */
