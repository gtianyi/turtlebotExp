#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h> 

#include "helper_functions.h"

class MakeMap{
private:
	//tf Transform Listener
	tf::TransformListener listener;

	// GridSize information (width, height)
	float grid_sizeX = 500;
	float grid_sizeY = 500;
	float grid_resolution = 0.05; // in meters/cell (5cm)

	//Sensor Meta data
	float min_range = 0.5;
	float max_range = 6.0;

	//reliability
	float p_measurement_given_occupied = 0.9;
	float p_measurement_given_notOccupied = 0.3;

	//Initialize some varables to store the objects to be published/subscribed
	Position position, camera_pos, origin;

	//map
	geometry_msgs::Twist velocity;

	nav_msgs::OccupancyGrid current_map;
    nav_msgs::MapMetaData meta;
    sensor_msgs::LaserScan scan;

    // Rotation Booleans
    bool start_spin = true;
    bool done_spinning = true;
    bool new_cmd = false;

    // ROS Topics
    ros::Publisher pub_map;
    ros::Subscriber cmd_vel, scan_sub, base_pos_sub;
    ros::Rate* r;

public:
	MakeMap();
	~MakeMap();

	void vel_callback(geometry_msgs::Twist vel_msg);
	void scan_callback(sensor_msgs::LaserScan scan_msg);
	void pose_callback(geometry_msgs::Pose2D pose_msg);

	Position setOrigin();
	void setOccupancy(unsigned index, float* prior, bool Free);
	void updateNeighbors(float scanX, float scanY);
	void updateMap();
	void publishNewMap();
};

/*
rosrun reed_final makeMap.py 
roslaunch reed_final map_maker.launch 
roslaunch reed_final rviz.launch 
roslaunch turtlebot_teleop keyboard_teleop.launch 
roslaunch turtlebot_gazebo turtlebot_world.launch 


http://wiki.ros.org/laser_scan_matcher
*/


