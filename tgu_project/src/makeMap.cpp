/**
 * File Name:   makeMap.cpp
 * Author:      Tianyi Gu, Zhuo Xu
 * Assignment:  Final Project - Frontier Based Exploration
 * Date:        April-8-2018 
 * Descroption: A mapping node that update an occupancy map
 */

#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/console.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <iostream>

#include <cmath>
#include <vector>

#include "helper.hpp"

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

// parameter for mapping
#define GRID_SIZEX 500
#define GRID_SIZEY 500
#define GRID_RESOLUTION 0.05 //in meters/cell(5cm)

#define LASER_MIN_RANG 0.5
#define LASER_MAX_RANG 6

#define SENSOR_MODEL_TRUE_POSITIVE 0.9
#define SENSOR_MODEL_FALSE_POSITIVE 0.3

#define OCCUPIED_PROB 0.5

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

class MakeMap
{
  public:
      MakeMap();

  private:
      void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
      void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
      void laserInitialize(const sensor_msgs::LaserScan::ConstPtr& msg);
      void initializeMap();
      void updateMap();
      void updateMapByOneScan(double range, int scanIndex);
      geometry_msgs::Point scan2map(geometry_msgs::Point p);
      std::pair<int, int> point2grid(geometry_msgs::Point p);
      void updateCell(const int x, const int y);
      void updateObstacleCell(const int mapIndex);
      void updateFreeCell(const int mapIndex);
      int gridXY2mapIndex(const int x, const int y);

      ros::NodeHandle nodeHandle;

      ros::Publisher pub_map;
      ros::Subscriber odom_sub;
      ros::Subscriber sub_laserScan;
      ros::Subscriber sub_base_pos;

      // tf Transform Listener
      tf::TransformListener listener;

      geometry_msgs::Pose curPose;
      geometry_msgs::Pose lockedPose;
      double roll, pitch, yaw;
	  double lockedYaw;
      std::vector<double> curRanges;
      std::vector<double> lockedRanges;
      bool laserInitialed = false;
      double angle_min;
      double angle_increment;
      int range_size;

      nav_msgs::OccupancyGrid current_map;
	  std::vector<double> prior;
	  double originOrentation;
      nav_msgs::MapMetaData meta;
      sensor_msgs::LaserScan scan;

};

MakeMap::MakeMap(){
    initializeMap();

    pub_map =
            nodeHandle.advertise<nav_msgs::OccupancyGrid>("/frontier_map", 100);

    sub_laserScan = nodeHandle.subscribe(
            "/scan", 100, &MakeMap::laserScan_callback, this);
    odom_sub = nodeHandle.subscribe<nav_msgs::Odometry>(
            "odom", 1, &MakeMap::odom_callback, this);

    listener.waitForTransform(
            "/map", "/camera_depth_frame", ros::Time(0), ros::Duration(1.0));

    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()){
        updateMap();
        loop_rate.sleep();
    }
}

void MakeMap::initializeMap(){
    // Initialize the map as unknown (-1) 
	// Initiallize prior as 0.5
    // row-major order
    for (int i = 0; i < GRID_SIZEX * GRID_SIZEY; i++) {
        current_map.data.push_back(-1);
        prior.push_back(0.5);
    }

    current_map.info.height = GRID_SIZEX; 
    current_map.info.width = GRID_SIZEY;
    current_map.info.resolution = GRID_RESOLUTION;
    current_map.info.origin = curPose;

    originOrentation = yaw;
}

void MakeMap::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    curPose = msg->pose.pose;
}

void MakeMap::laserScan_callback(
    const sensor_msgs::LaserScan::ConstPtr& msg){
    if( !laserInitialed) {
        laserInitialize(msg);
        return;
    }
    for (int i = 0; i < range_size; i++){
        curRanges[i] = msg->ranges[i];
    }
}

void MakeMap::laserInitialize(
    const sensor_msgs::LaserScan::ConstPtr& msg){
    for (const auto &p : msg -> ranges){
        curRanges.push_back(p);
    }
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;
    range_size = msg->ranges.size();
    laserInitialed = true;
}

void MakeMap::updateMap() {
    lockedPose = curPose;
    lockedYaw = yaw;
    lockedRanges = curRanges;
    for (int i = 0; i < range_size; ++i) {
        if (!std::isnan(lockedRanges[i])) {
            updateMapByOneScan(lockedRanges[i], i);
        }
    }
	pub_map.publish(current_map);
}

void MakeMap::updateMapByOneScan(double range,int scanIndex){
    geometry_msgs::Point scanPoint;
    scanPoint.x = range * cos(angle_min + scanIndex * angle_increment);
    scanPoint.y = range * sin(angle_min + scanIndex * angle_increment);
    geometry_msgs::Point mapPoint = scan2map(scanPoint);
	std::pair<int,int> gridXY = point2grid(mapPoint);
	updateCell(gridXY.first,gridXY.second);
}

geometry_msgs::Point MakeMap::scan2map(geometry_msgs::Point p){
    Eigen::Matrix3d R = getRotationMatrix(lockedYaw);
    Eigen::Matrix3d T =
            getTranslationMatrix(lockedPose.position.x, lockedPose.position.y);
    return pointTranform(p, R, T);
}

std::pair<int,int> MakeMap::point2grid(geometry_msgs::Point p){
    geometry_msgs::Point originPosition = current_map.info.origin.position;

    int gridX = int(round((p.x-originPosition.x)/GRID_RESOLUTION));
    int gridY = int(round((p.y-originPosition.y)/GRID_RESOLUTION));
    assert(gridX < GRID_SIZEX);
    assert(gridY < GRID_SIZEY);
    return std::pair<int, int> (gridX,gridY);
}

void MakeMap::updateCell(const int x, const int y){
    // update obstacle cell
    int obstacleCellIndex = gridXY2mapIndex(x, y);
    updateObstacleCell(obstacleCellIndex);

    // update line of sight
    std::pair<int, int> robotXY = point2grid(lockedPose.position);
    std::vector<std::pair<int, int>> lineOfSight =
            getLine(x, y, robotXY.first, robotXY.second);

    for (auto cell : lineOfSight) {
        int freeCellIndex = gridXY2mapIndex(cell.first, cell.second);
        updateFreeCell(freeCellIndex);
	}
}

void MakeMap::updateObstacleCell(const int mapIndex){
    double p_z = SENSOR_MODEL_TRUE_POSITIVE * prior[mapIndex] +
            SENSOR_MODEL_FALSE_POSITIVE * (1 - prior[mapIndex]);
    prior[mapIndex] = SENSOR_MODEL_TRUE_POSITIVE * prior[mapIndex] / p_z;
    current_map.data[mapIndex] = prior[mapIndex] > OCCUPIED_PROB ? 1 : 0;
}

void MakeMap::updateFreeCell(const int mapIndex){
    if (prior[mapIndex] == 0.5 || prior[mapIndex] == 0.3) {
        prior[mapIndex] = 0.3;
	}	
	else{
        prior[mapIndex] = 0.3;// update Bayes rule if this doesn't work
	}
    current_map.data[mapIndex] = prior[mapIndex] > OCCUPIED_PROB ? 1 : 0;
}

int MakeMap::gridXY2mapIndex(const int x, const int y){
    return y * GRID_SIZEY + x;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "make_map");
    MakeMap makeMap;
}