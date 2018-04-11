#include "makeMap_1.h"


MakeMap::MakeMap()
//Constructor
{
    
    ros::NodeHandle n;
        
    n.param<float>("x_size", grid_sizeX, 500);
    n.param<float>("y_size", grid_sizeY, 500);
    n.param<float>("grid_size", grid_resolution, 0.05); // in meters/cell (5cm)
    
    // Sensor Meta data
    n.param<float>("min_range", min_range, 0.4);
    n.param<float>("max_range", max_range, 6.0);

    // reliability
    n.param<float>("p_z|occ", p_measurement_given_occupied, 0.9);
    n.param<float>("p_z|notOcc", p_measurement_given_notOccupied, 0.3);
  
    
    // Initialize the map as unknown (-1)
    for(int i = 0; i < grid_sizeX*grid_sizeY; i++) // row-major order
    	current_map.data.push_back(-1);
    
    origin = setOrigin();

	
	r = new ros::Rate(50);
	r->sleep();
	
    
    // Publishers
    pub_map = n.advertise<nav_msgs::OccupancyGrid>("/frontier_map", 100);

    // Subscribers
    cmd_vel = n.subscribe("/cmd_vel_mux/input/navi", 100, &MakeMap::vel_callback, this);
    scan_sub = n.subscribe("/scan", 100, &MakeMap::scan_callback, this);
    base_pos_sub = n.subscribe("/pose2D", 100, &MakeMap::pose_callback, this);
    
    listener.waitForTransform("/map", "/camera_depth_frame", ros::Time(0), ros::Duration(1.0) );

    while (ros::ok()){
        updateMap();
        r->sleep();
    }
}

MakeMap::~MakeMap(){
	delete r;
}

//// Callbacks ////

void MakeMap::vel_callback(geometry_msgs::Twist vel_msg)
{
	
}

void MakeMap::scan_callback(sensor_msgs::LaserScan scan_msg)
{
	
}

void MakeMap::pose_callback(geometry_msgs::Pose2D pose_msg)
{
	
}

//// Map Making Functions ////

Position MakeMap::setOrigin()
// set the origin variable 
{
	origin = Position();
    origin.x = grid_sizeX/2*grid_resolution;
    origin.y = grid_sizeY/2*grid_resolution;
    origin.theta = 0;
    
    current_map.info.height = grid_sizeX;
    current_map.info.width = grid_sizeY;
    current_map.info.resolution = grid_resolution;
    
    return origin;
}

void MakeMap::setOccupancy(unsigned index, float* prior, bool Free)
// This function defines the occupancy of the cells
{
	
}

void MakeMap::updateNeighbors(float scanX, float scanY)
//Update the free occupancy grid cells between the robot and the   
//obstacle found using the laser scan.
{
	
}

void MakeMap::updateMap()
//This function updates the occupancy grid cells with the current 
//scan.
{
	
}

void MakeMap::publishNewMap()
//This function publishes an occupancy grid to the ROS topic 
// "/frontier_map".
{
	
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Make_Map");

    MakeMap m();
}

/*
rosrun reed_final makeMap.py 
roslaunch reed_final map_maker.launch 
roslaunch reed_final rviz.launch 
roslaunch turtlebot_teleop keyboard_teleop.launch 
roslaunch turtlebot_gazebo turtlebot_world.launch 


http://wiki.ros.org/laser_scan_matcher
*/


