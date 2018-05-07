/**
 * File Name:   frontierExploration.cpp
 * Author:      Tianyi Gu, Zhuo Xu
 * Assignment:  Final Project - Frontier Based Exploration
 * Date:        April-12-2018 
 * Descroption: A node that implement frontier exploration algorithm 
 *              to find best next position to scan the world 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <cmath>
#include <vector>

#define MAX_LIN_VEL = .7
#define MAX_ROT_VEL = 3.14

#define BASE_DIAMETER .23
#define WHEEL_RADIUS .035
#define WHEEL_WIDTH .021
#define TICKS_PER_REVOLUTION 52
#define PULSES_PER_REVOLUTION 13
#define TICK_TO_METER .000085292090497737556558
#define METER_TO_TICK 11724.41658029856624751591
#define MILLIMITER_TO_TICK 11.72441658029856624751591
#define TICK_TO_RADIAN .002436916871363930187454

#define PI 3.141592653589793238462643383279502884197

#define TURN_THRESHOLD 0.02 // about 1 degree
#define POINT_DISTANT_THRESHOLD 0.05

#define LOOP_RATE 50
#define DURATION_SHORT 1
#define DURATION_MIDDLE 3
#define DURATION_LONG 15

// parameter for mapping
#define GRID_SIZEX 480
#define GRID_SIZEY 480
#define GRID_RESOLUTION 0.05 //in meters/cell(5cm)

#define LASER_MIN_RANG 0.5
#define LASER_MAX_RANG 6

#define SENSOR_MODEL_TRUE_POSITIVE 0.9
#define SENSOR_MODEL_FALSE_POSITIVE 0.3

#define OCCUPIED_PROB 0.5

#define UTIL_DIS_WEIGHT 0.7
#define UTIL_LEN_WEIGHT 0.3

// parameter for frontier exploration
#define FRONTIER_FILTER_MIN 20

double rad2degree(double rad){
    return rad / PI * 180;
}

double degree2rad(double degree){
    return degree / 180 * PI;
}

double normalize_degree(double degree){
    return fmod(degree + 360, 360);
}

double normalize_rad(double rad){
    double degree = rad2degree(rad);
    degree = normalize_degree(degree);
    return degree2rad(degree);
}

double distanceBetween(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0));
}

Eigen::Matrix3d getRotationMatrix(double theta){
    Eigen::Matrix3d retMatrix;
    retMatrix << cos(theta), -sin(theta), 0.0, 
            sin(theta), cos(theta), 0.0, 
            0.0, 0.0, 1.0;
    return retMatrix;
}

Eigen::Matrix3d getTranslationMatrix(double dx, double dy){
    Eigen::Matrix3d retMatrix;
    retMatrix << 1.0, 0.0, dx, 
            0.0, 1.0, dy, 
            0.0, 0.0, 1.0;
    return retMatrix;
}

geometry_msgs::Point pointTranform(geometry_msgs::Point &point,
                                   Eigen::Matrix3d R,
                                   Eigen::Matrix3d T){
    Eigen::Vector3d v(point.x, point.y, 1);
    v = T * R * v;
    geometry_msgs::Point retPoint;
    retPoint.x = v[0];
    retPoint.y = v[1];
    return retPoint;
}

struct GridCell{
		unsigned int x;
		unsigned int y;
		int data;
		bool onFrontier;
		int frontierID;
};

class FrontierExploration
{
  public:
      FrontierExploration();

  private:
      void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
      void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
      void move_base_status_callback(
              const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

      void run();
	  void updateGoalPose();
	  void updateFrontiers();
	  void checkFrontiers();
	  void connectedComponentLabeling();
	  bool checkFrontier(const int x, const int y);
	  double getUtil(geometry_msgs::Point centroid, int frontierLength);
      void rotate360();
      void fullStop();
      void vel_from_wheels(double vl, double vr, double sec);
      void visualizeCentroid(const geometry_msgs::Point& centroid,
              unsigned int ID,
              int action = 0,
              bool newMarker = true);
      void visualizeFrontier(const std::vector<GridCell>& frontier,
              unsigned int ID,
              int action = 0,
              bool newMarker = true);
      void removeAllMarkers();

      geometry_msgs::Point grid2point(const int x, const int y);

      ros::NodeHandle nodeHandle;

      ros::Publisher pub_goal;
      ros::Publisher pub_vel;
      ros::Publisher pub_marker_frontier;
      ros::Publisher pub_marker_centorid;

      ros::Subscriber sub_odom;
      ros::Subscriber sub_map;
      ros::Subscriber sub_move_base_status;

	  std::vector<std::vector<GridCell>> map2D;
	  std::vector<std::vector<GridCell>> frontiers;
	  
	  geometry_msgs::Point mapOrigin;
      bool mapInitialized = false;
      geometry_msgs::PoseStamped goalPose;
      actionlib_msgs::GoalStatus goalStatus; 
      double roll, pitch, yaw;
	  geometry_msgs::Point robotPosition;
      bool terminate = false;
      visualization_msgs::MarkerArray markerArray, lineMarker;
	  geometry_msgs::Twist curVel;
};

FrontierExploration::FrontierExploration(){
    
	//publishers
    pub_marker_centorid = nodeHandle.advertise<visualization_msgs::MarkerArray>(
            "/centroid_marker", 100);
    pub_marker_frontier = nodeHandle.advertise<visualization_msgs::MarkerArray>(
            "/frontier_marker", 100);
    pub_goal = nodeHandle.advertise<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 100);
    pub_vel = nodeHandle.advertise<geometry_msgs::Twist>(
            "cmd_vel_mux/input/teleop", 100);

    // subscribers
    sub_map = nodeHandle.subscribe(
            "map", 100, &FrontierExploration::map_callback, this);
    sub_move_base_status = nodeHandle.subscribe("/move_base/status",
            100,
            &FrontierExploration::move_base_status_callback,
            this);
    sub_odom = nodeHandle.subscribe<nav_msgs::Odometry>(
            "odom", 1, &FrontierExploration::odom_callback, this);

    goalPose.header.frame_id = "/map";
    goalPose.pose.orientation.w = 1;
    ros::spinOnce();

    run();
}

void FrontierExploration::odom_callback(
        const nav_msgs::Odometry::ConstPtr& msg) {
    yaw = tf::getYaw(msg->pose.pose.orientation);
    robotPosition = msg->pose.pose.position;
}

void FrontierExploration::map_callback(
        const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    int mapIndex = 0;
    if (!mapInitialized) {
        mapInitialized = true;
		mapOrigin=msg->info.origin.position;
        for (int i = 0; i < GRID_SIZEX; i++) {
            std::vector<GridCell> row;
            for (int j = 0; j < GRID_SIZEY; j++) {
                GridCell cell;
                cell.data = msg->data[mapIndex];
                cell.x = i;
                cell.y = j;
                row.push_back(cell);
				mapIndex++;
            }
            map2D.push_back(row);
		}
    }
	else{
        for (int i = 0; i < GRID_SIZEY; i++) {
            for (int j = 0; j < GRID_SIZEX; j++) {
                map2D[i][j].data = msg->data[mapIndex];
				mapIndex++;
            }
        }
    }
}

void FrontierExploration::move_base_status_callback(
        const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    if (!msg->status_list.empty()) {
        goalStatus = msg->status_list[0];
    }
}

void FrontierExploration::run(){
    ros::Rate loop_rate(LOOP_RATE);
    loop_rate.sleep();

    while (ros::ok()) {
        rotate360();
        ros::spinOnce();
        ros::Duration(DURATION_MIDDLE).sleep();

        ros::spinOnce();
        updateFrontiers();
        updateGoalPose();

        if (terminate) {
            break;
        }

        pub_marker_frontier.publish(lineMarker);
        ros::spinOnce();
		ros::Duration(DURATION_SHORT).sleep();

        pub_marker_centorid.publish(markerArray);
		ros::spinOnce();
		ros::Duration(DURATION_SHORT).sleep();

        pub_goal.publish(goalPose);
		ros::spinOnce();
		ros::Duration(DURATION_LONG).sleep();
		ros::spinOnce();

        while (goalStatus.status == actionlib_msgs::GoalStatus::ACTIVE) {
            ROS_INFO("moving to centroid....");
			ros::spinOnce();
			loop_rate.sleep();
        }

		removeAllMarkers();
    }
}

void FrontierExploration::updateFrontiers(){
    checkFrontiers();
    connectedComponentLabeling();	
}

void FrontierExploration::checkFrontiers() {
    for (int i = 0; i < GRID_SIZEY; i++) {
        for (int j = 0; j < GRID_SIZEX; j++) {
            if (map2D[i][j].data == 0) {
                map2D[i][j].onFrontier =
                        checkFrontier(map2D[i][j].x, map2D[i][j].y);
            }
            map2D[i][j].frontierID = -1;
        }
    }
}

bool FrontierExploration::checkFrontier(const int x, const int y){
    for (int i = x - 1; i <= x + 1; ++i) {
        for (int j = y - 1; j <= y + 1; ++j) {
            if (i >= 0 && i <= GRID_SIZEX && 
				j >= 0 && j <= GRID_SIZEY &&
                map2D[i][j].data == -1) {
                return true;
            }
        }
    }
    return false;
}

void FrontierExploration::connectedComponentLabeling() {
    std::vector<std::vector<GridCell>> tempfrontiers;

    for (int i = 0; i < GRID_SIZEY; i++) {
        for (int j = 0; j < GRID_SIZEX; j++) {
            if (!map2D[i][j].onFrontier) {
                continue;
            } else if (i - 1 >= 0 && j - 1 >= 0 &&
                    map2D[i - 1][j - 1].frontierID != -1) {
                map2D[i][j].frontierID = map2D[i - 1][j - 1].frontierID;
				tempfrontiers[map2D[i][j].frontierID].push_back(map2D[i][j]);
            } else if ((i - 1 >= 0 && j - 1 >= 0 &&
                               map2D[i - 1][j].frontierID == -1 &&
                               map2D[i][j - 1].frontierID == -1) ||
                    (i - 1 < 0 && j - 1 >= 0 &&
                               map2D[i][j - 1].frontierID == -1) ||
                    (i - 1 >= 0 && j - 1 < 0 &&
                               map2D[i - 1][j].frontierID == -1)) {
                map2D[i][j].frontierID = tempfrontiers.size();
                std::vector<GridCell> newfrontier;
                newfrontier.push_back(map2D[i][j]);
                tempfrontiers.push_back(newfrontier);
            } else if (i - 1 >= 0 && map2D[i - 1][j].frontierID != -1) {
                map2D[i][j].frontierID = map2D[i - 1][j].frontierID;
            } else if (j - 1 >= 0 && map2D[i][j - 1].frontierID != -1) {
                map2D[i][j].frontierID = map2D[i][j - 1].frontierID;
            }
        }
    }

    frontiers.clear();
    for (auto f : tempfrontiers) {
        if (f.size() > FRONTIER_FILTER_MIN) {
            frontiers.push_back(f);
        }
    }
}

void FrontierExploration::updateGoalPose(){
    double bestUtil = 0;
    int id = 0;

    geometry_msgs::Point bestCentroid;
    for (const auto& frontier : frontiers) {
        int x_c = 0;
        int y_c = 0;
        for (const auto& cell : frontier) {
            x_c += cell.x;
            y_c += cell.y;
        }
        x_c = x_c / frontier.size();
        y_c = y_c / frontier.size();

        geometry_msgs::Point centroid = grid2point(x_c, y_c);
        double util = getUtil(centroid, frontier.size());
        if (util > bestUtil) {
            bestUtil = util;
            goalPose.pose.position = centroid;
			bestCentroid=centroid;
        }

        visualizeCentroid(centroid, id);
        visualizeFrontier(frontier, id);
        id++;
    }

    visualizeCentroid(bestCentroid, id, 0, false);

    if (id == 0) {
        terminate = true;
    }
}

double FrontierExploration::getUtil(geometry_msgs::Point centroid, const int frontierLength){
	double dis = distanceBetween(centroid, robotPosition);
    //double util = frontierLength/dis;
    double util = frontierLength;
    return util;
}

geometry_msgs::Point FrontierExploration::grid2point(const int x, const int y) {
    geometry_msgs::Point retPosition;

    retPosition.y = (float)x * GRID_RESOLUTION + mapOrigin.x;
    retPosition.x = (float)y * GRID_RESOLUTION + mapOrigin.y;

    return retPosition;
}

 
////Map Markers////
void FrontierExploration::visualizeCentroid(
        const geometry_msgs::Point& centroid,
        unsigned int ID,
        int action,
        bool newMarker) {
    // Creates a marker on RVIZ for the centroid of the frontier
        
    visualization_msgs::Marker RVIZmarker = visualization_msgs::Marker();
    
    RVIZmarker.header.frame_id = "/map";
    RVIZmarker.header.stamp = ros::Time(0);
    
    RVIZmarker.ns = "Frontier";
    RVIZmarker.id = ID;
    RVIZmarker.type = RVIZmarker.CUBE;
    RVIZmarker.action = action;
    
    // Position with respect to the frame
	RVIZmarker.pose.position=centroid;
    
    // Define the scale (meter scale)
    RVIZmarker.scale.x = 0.3;
    RVIZmarker.scale.y = 0.3;
    RVIZmarker.scale.z = 0.3;
    
    // Set the color 
    RVIZmarker.color.a = 1.0;
    RVIZmarker.color.g = 0.0;
    
    if (newMarker){
        RVIZmarker.color.r = 0.0;
        RVIZmarker.color.b = 1.0;
    } else {
        RVIZmarker.color.r = 1.0;
        RVIZmarker.color.b = 0.0;
    }
    
    // Store the marker
    if (markerArray.markers.size() <= ID){
        markerArray.markers.push_back(RVIZmarker);
    } else{
        markerArray.markers[ID] = RVIZmarker;
    }
}

void FrontierExploration::visualizeFrontier(const std::vector<GridCell>& XY,
        unsigned int ID,
        int action,
        bool newMarker) {
    // Creates a line marker on RVIZ for the entire frontier
    
	visualization_msgs::Marker RVIZmarker = visualization_msgs::Marker();
    
    RVIZmarker.header.frame_id = "/odom";
    RVIZmarker.header.stamp = ros::Time(0);
    
    RVIZmarker.ns = "Frontier";
    RVIZmarker.id = ID;
    RVIZmarker.type = RVIZmarker.POINTS;
    RVIZmarker.action = action;
    
    // Define the scale (meter scale)
    RVIZmarker.scale.x = 0.05;
    RVIZmarker.scale.y = 0.05;
    
    // Set the color 
    RVIZmarker.color.a = 1.0;
    RVIZmarker.color.g = 0.0;
    
    if (newMarker){
        RVIZmarker.color.r = 0.0;
        RVIZmarker.color.b = 1.0;
    } else {
        RVIZmarker.color.r = 1.0;
        RVIZmarker.color.b = 0.0;
    }
    
    // Store all of the real world XY coordinates
    for (const auto &cell : XY) {
        geometry_msgs::Point pnt = grid2point(cell.x, cell.y);
        RVIZmarker.points.push_back(pnt);
    }

    // Store the marker
    if (lineMarker.markers.size() <= ID){
        lineMarker.markers.push_back(RVIZmarker);
    } else{
        lineMarker.markers[ID] = RVIZmarker;
    }
}

void FrontierExploration::removeAllMarkers(){
	// This function removes all markers from rviz
	visualization_msgs::Marker m = visualization_msgs::Marker();
	for (unsigned int i = 0; i < markerArray.markers.size(); i++){
		markerArray.markers[i].action = m.DELETE;
		lineMarker.markers[i].action = m.DELETE;
	}
        
    pub_marker_centorid.publish(markerArray);
    pub_marker_frontier.publish(lineMarker);
    
    markerArray = visualization_msgs::MarkerArray();
    lineMarker = visualization_msgs::MarkerArray();
}

void FrontierExploration::rotate360(){
    double vl, vr;
    vl = -0.05;
    vr = 0.05;
    vel_from_wheels(vl, vr, 14);
}

void FrontierExploration::vel_from_wheels(double vl, double vr, double sec){
    double lin_vel = 0.5 * (vl + vr);
    double ang_vel = (vr - vl) / BASE_DIAMETER;
    curVel.linear.x = lin_vel;
    curVel.angular.z = ang_vel;
    ros::Rate loop_rate(LOOP_RATE);
    loop_rate.sleep();
    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = ros::Duration(sec); 
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    while(ros::Time::now() < endTime && ros::ok())
    {
        this->pub_vel.publish(curVel);
		ros::spinOnce();
        loop_rate.sleep();
		ROS_INFO_STREAM("rotate 360...");
    }
}

void FrontierExploration::fullStop(){
    curVel.linear.x = 0;
    curVel.angular.z = 0;
    ros::Rate loop_rate(LOOP_RATE);
    loop_rate.sleep();
    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = ros::Duration(0.5); 
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    while(ros::Time::now() < endTime && ros::ok())
    {
        this->pub_vel.publish(curVel);
		ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "frontier_exploration");
    FrontierExploration frontierExploration;
}
