/**
 * File Name:   lab5.cpp
 * Author:      Tianyi Gu, Zhuo Xu
 * Assignment:  Lab5
 * Date:        Mar-1-2018 
 * Descroption: A ROS node for obstacle avoidance using Bug2 algorithm.  
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include "nav_msgs/Odometry.h"
#include <boost/thread/thread.hpp>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <iostream>

#include <math.h>
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

// parameter for bug2
#define GOAL_DISTANCE 3.5
#define INITIAL_DISTANCE 0.5
#define CLEARANCE_DISTANCE 0.6 // 0.45 limit for gazebo
#define LAEERSCAN_MIN_RANG 0.46 // 0.45 limit for gazebo

#define SCAN_INTERVAL 10

#define MLine_DEVIANCE 0.1
#define GOAL_SEEK_SPEED 0.3
#define GOAL_SEEK_STEP_SEC 0.1
#define WALL_FOLLOW_SPEED 0.2
#define WALL_FOLLOW_STEP_SEC_REPLUSE 1
#define WALL_FOLLOW_STEP_SEC_ATTRACT 0.2
#define ARC_OMEGA 0.5
#define WALL_FOLLOW_REPULSE_ANGLE 0.35 // 20degree;
#define CORNER_LASERPOINT_PERCENT 0.6; // less than this nuber is corner

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

struct M_Line{
    double orentation;
    geometry_msgs::Point point;
    double disFrom(geometry_msgs::Point p){
        double angle = atan2(p.y - point.y, p.x - point.x) - orentation;
        double retDis = distanceBetween(p, point) * fabs(sin(angle));
    }
};

enum RobotStatus{AtGoal, AtMLine, AtHitPoint, LostWall, Collision, Overshoot, OK};

class TurtleBotLab5
{
  public:
    TurtleBotLab5();

  private:
    void odom_thread();
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void bumper_thread();
    void bumper_callback(const kobuki_msgs::BumperEventConstPtr msg);
    void laserScan_thread();
    void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void laserInitialize(const sensor_msgs::LaserScan::ConstPtr& msg);
    void velPub_thread();
    void move();
    void drive_straight(double speed, double distance);
    void drive_arc(double radius, double omega, double angle);
    void initializeMLine();
    void setGoalPoint();
    void goal_seek();
    bool obstacleDetected();
    void robotStatusCheck_goal_seek();
    void wall_follow();
    void robotStatusCheck_wall_follow();
    bool cornerDetected();
    bool doubleLaserDetected();
    double calcWallAngleInRobotCoord();
    void rotate(double angle);
    void adjustToMLine();
    void fullStop();
    void vel_from_wheels(double vl,double vr, double sec);

    ros::NodeHandle nodeHandle_pub, nodeHandle_sub_odom,
        nodeHandle_sub_laserScan, nodeHandle_sub_bumper;

    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber laserScan_sub;
    ros::Subscriber bumper_sub;

    geometry_msgs::Twist curVel;
    geometry_msgs::Point curPosition;
    double roll, pitch, yaw;
    std::vector<double> curRanges;
    std::vector<double> prevRanges;
    bool laserInitialed = false;
    double angle_min;
    double range_min;
    double angle_increment;
    int range_size;
    geometry_msgs::Point goalPoint;
    int obstacleIndex = -1;
    double minDis = 100;
    bool rightContact;
    M_Line mLine;
    geometry_msgs::Point hitPoint;
    RobotStatus robotStatus;
    bool collide = false;
    double goalDis;
};

TurtleBotLab5::TurtleBotLab5(){
    boost::thread t_odom( &TurtleBotLab5::odom_thread, this);
    boost::thread t_bumper( &TurtleBotLab5::bumper_thread, this);
    boost::thread t_velPub( &TurtleBotLab5::velPub_thread, this);
    boost::thread t_laserScan( &TurtleBotLab5::laserScan_thread, this);
    ros::spin();
    t_odom.join();
    t_bumper.join();
    t_velPub.join();    
    t_laserScan.join();
}

void TurtleBotLab5::odom_thread(){
    odom_sub = nodeHandle_sub_odom.subscribe<nav_msgs::Odometry>(
        "odom", 1, &TurtleBotLab5::odom_callback, this);
}

void TurtleBotLab5::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    curPosition = msg->pose.pose.position;
}

void TurtleBotLab5::bumper_thread(){
    bumper_sub = nodeHandle_sub_bumper.subscribe
            <kobuki_msgs::BumperEventConstPtr>(
        "/mobile_base/events/bumper", 1, &TurtleBotLab5::bumper_callback, this);
}

void TurtleBotLab5::bumper_callback(const kobuki_msgs::BumperEventConstPtr msg){
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED){
        collide = true;
        robotStatus = RobotStatus::Collision;
        ROS_INFO("bump...  ");
    }
}

void TurtleBotLab5::laserScan_thread(){
    laserScan_sub = nodeHandle_sub_laserScan.subscribe
            <sensor_msgs::LaserScan>(
        "scan", 1, &TurtleBotLab5::laserScan_callback, this);
}

void TurtleBotLab5::laserScan_callback(
    const sensor_msgs::LaserScan::ConstPtr& msg){
    if( !laserInitialed) {
        laserInitialize(msg);
        return;
    }
    minDis = 100;
    obstacleIndex = -1;
    for (int i = 0; i < range_size; i++){
        curRanges[i] = msg->ranges[i];
        if(minDis > curRanges[i]){
            obstacleIndex = i;
            minDis = curRanges[i];
        }
    }
    if(obstacleIndex != -1)
        rightContact = obstacleIndex < range_size / 2;
}

void TurtleBotLab5::laserInitialize(
    const sensor_msgs::LaserScan::ConstPtr& msg){
    for (const auto &p : msg -> ranges){
        curRanges.push_back(p);
        prevRanges.push_back(p);
    }
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;
    range_size = msg->ranges.size();
    laserInitialed = true;
}

void TurtleBotLab5::velPub_thread(){
    vel_pub =  nodeHandle_pub.advertise<geometry_msgs::Twist>(
        "cmd_vel_mux/input/teleop", 1, true);
    move();
}

void TurtleBotLab5::move(){
    drive_straight(0.3, INITIAL_DISTANCE);
    initializeMLine();
    setGoalPoint();
    robotStatus = RobotStatus::AtMLine;
    while(robotStatus == RobotStatus::AtMLine){
        goal_seek();
        wall_follow();
    }
    switch(robotStatus){
        case RobotStatus::AtGoal:
            ROS_INFO("Achieve The Goal ! ! ! !");
            break;
        case RobotStatus::AtHitPoint:
            ROS_INFO("No Solution Found ! ! !");
            break;
        case RobotStatus::Collision:
            ROS_INFO("Robot Collide Obstacal !");
            break;
    }
}

void TurtleBotLab5::drive_straight(double speed, double distance){
    geometry_msgs::Point start_position = curPosition;
    double curDis = 0;
    while(curDis < distance){
        vel_from_wheels(speed, speed, 0.1);
        curDis = distanceBetween(curPosition, start_position);
    }
    fullStop();   
}

void TurtleBotLab5::initializeMLine(){
    mLine.orentation = yaw;
    mLine.point = curPosition;
}

void TurtleBotLab5::setGoalPoint(){
    goalPoint = mLine.point;
    goalPoint.x += GOAL_DISTANCE * cos(mLine.orentation);
    goalPoint.y += GOAL_DISTANCE * sin(mLine.orentation);
    ROS_INFO("goal: [%f], [%f]", goalPoint.x, goalPoint.y);
}

void TurtleBotLab5::goal_seek(){
    ROS_INFO("Goal seeking...  ");
    robotStatus = RobotStatus::OK;
    while( !obstacleDetected() && robotStatus == RobotStatus::OK){
        vel_from_wheels(GOAL_SEEK_SPEED,
                        GOAL_SEEK_SPEED ,
                        GOAL_SEEK_STEP_SEC);
        robotStatusCheck_goal_seek();
        if(robotStatus == RobotStatus::Overshoot)
        {
            adjustToMLine();
            robotStatus = RobotStatus::OK;
        }
        goalDis = distanceBetween(curPosition, goalPoint);
    }
    fullStop();
    if(obstacleDetected())
    {
        hitPoint = curPosition;
        ROS_INFO("hit at m: [%f], [%f]", hitPoint.x, hitPoint.y);
        robotStatus = RobotStatus::AtMLine;
    }
}

bool TurtleBotLab5::obstacleDetected(){
    return obstacleIndex != -1 && minDis < CLEARANCE_DISTANCE;
}

void TurtleBotLab5::robotStatusCheck_goal_seek(){
    if(collide) return;
    if(distanceBetween(curPosition, goalPoint) <= POINT_DISTANT_THRESHOLD)
        robotStatus = RobotStatus::AtGoal;
    else if (distanceBetween(curPosition, goalPoint) > goalDis){
        robotStatus = RobotStatus::Overshoot;
    }
    else
        robotStatus = RobotStatus::OK;
}

void TurtleBotLab5::wall_follow(){
    if(robotStatus != RobotStatus::AtMLine) return;
    ROS_INFO("Wall following...  ");
    robotStatus = RobotStatus::OK;
    double prevWallAngleInRobotCoord;
    while( robotStatus == RobotStatus::OK){
        double wallAngleInRobotCoord = calcWallAngleInRobotCoord();
        // always trun right
        if(wallAngleInRobotCoord == 0){ continue;}
        double repluseAngle = wallAngleInRobotCoord -
                WALL_FOLLOW_REPULSE_ANGLE;
        double attractAngle = WALL_FOLLOW_REPULSE_ANGLE + PI / 2;
        ROS_INFO("Repluse...  ");
        rotate(repluseAngle);
        vel_from_wheels( WALL_FOLLOW_SPEED,
                         WALL_FOLLOW_SPEED,
                         WALL_FOLLOW_STEP_SEC_REPLUSE);
        ROS_INFO("Attract...  ");
        rotate(attractAngle);
        while( !obstacleDetected()){
            vel_from_wheels( WALL_FOLLOW_SPEED,
                             WALL_FOLLOW_SPEED,
                             WALL_FOLLOW_STEP_SEC_ATTRACT);
        }
        if(robotStatus != RobotStatus::AtMLine && cornerDetected()){
            ROS_INFO("Corner detected, adjusting...  ");
            rotate( -PI / 2);
            drive_arc(CLEARANCE_DISTANCE, 0.3, PI / 2);
            rotate( PI / 2);
            while( !obstacleDetected()){
                vel_from_wheels(GOAL_SEEK_SPEED,
                                GOAL_SEEK_SPEED ,
                                WALL_FOLLOW_STEP_SEC_ATTRACT);
            }
        }
        robotStatusCheck_wall_follow();
        while(robotStatus == RobotStatus::LostWall){
            ROS_INFO("Lost Wall, finding...  ");
            rotate(degree2rad(10));
            robotStatusCheck_wall_follow();
        }
    }
    if (robotStatus == RobotStatus::AtMLine)
        adjustToMLine();
}

double TurtleBotLab5::calcWallAngleInRobotCoord(){
    double retAngle;
    geometry_msgs::Point p1, p2;
    int p1_index, p2_index;
    // we always go right along the wall;
    if(rightContact){
        p2_index = obstacleIndex;
        p1_index = obstacleIndex + SCAN_INTERVAL;
    }
    else{
        p1_index = obstacleIndex;
        p2_index = obstacleIndex - SCAN_INTERVAL;
    }
    if(obstacleIndex == -1 ||
       isnan(curRanges[p1_index]) || isnan(curRanges[p2_index]))
        return 0;
    p1.x = curRanges[p1_index] * cos(angle_min + p1_index *angle_increment);
    p1.y = curRanges[p1_index] * sin(angle_min + p1_index *angle_increment);
    p2.x = curRanges[p2_index] * cos(angle_min + p2_index *angle_increment);
    p2.y = curRanges[p2_index] * sin(angle_min + p2_index *angle_increment);
    retAngle = atan2(p2.y - p1.y, p2.x - p1.x);
    ROS_INFO("p1i, p2i: [%d], [%d]", p1_index, p2_index);
    ROS_INFO("p1, p2: [%f], [%f]", curRanges[p1_index], curRanges[p2_index]);
    ROS_INFO("p1: [%f], [%f]", p1.x, p1.y);
    ROS_INFO("p2: [%f], [%f]", p2.x, p2.y);
    ROS_INFO("angle: [%f]", retAngle);
    return retAngle;
}

void TurtleBotLab5::rotate(double angle){
    if(fabs(angle) < TURN_THRESHOLD) return;
    double vl, vr;
    if (angle < 0){
        vl = 0.05;
        vr = -0.05;
    }
    else{
        vl = -0.05;
        vr = 0.05;
    }
    double curAngle = 0;
    while(fabs(curAngle) < fabs(angle)){
        double start_orentation = yaw;
        vel_from_wheels(vl, vr, 0.05);
        double deltaAngle;
        if(angle > 0){
            if(yaw < 0 && start_orentation > 0){
                deltaAngle = 2 * PI + (yaw - start_orentation);
            }
            else{
                deltaAngle = yaw - start_orentation;
            }
        }
        else{
            if(start_orentation < 0 && yaw > 0){
                deltaAngle = 2 * PI + (start_orentation - yaw);
            }
            else{
                deltaAngle = start_orentation - yaw;
            }
        }
        if(fabs(deltaAngle) > TURN_THRESHOLD){
            curAngle += deltaAngle;
        }
    }
    fullStop();
}

void TurtleBotLab5::robotStatusCheck_wall_follow(){
    if(collide) return;
    if(distanceBetween(curPosition, hitPoint) <= POINT_DISTANT_THRESHOLD)
    {
        robotStatus = RobotStatus::AtHitPoint;
    }
    else if(mLine.disFrom(curPosition) <= MLine_DEVIANCE){
        ROS_INFO("cur: [%f], [%f]", curPosition.x, curPosition.y);
        ROS_INFO("mLine dis: [%f]", mLine.disFrom(curPosition));
        robotStatus = RobotStatus::AtMLine;
    }
    else if(distanceBetween(curPosition, goalPoint) <= POINT_DISTANT_THRESHOLD){
        robotStatus = RobotStatus::AtGoal;
    }
    else
        robotStatus = RobotStatus::OK;
    if( !doubleLaserDetected() )
    {
        robotStatus = RobotStatus::LostWall;
    }
}

bool TurtleBotLab5::cornerDetected(){
    double pcount = 0.0;
    for (const auto &p : curRanges){
        if(!isnan(p)) pcount+= 1.0;
    }
    if(pcount < range_size * 0.6) return true;
    return false;
}

bool TurtleBotLab5::doubleLaserDetected(){
    if( !obstacleDetected() ) return false;
    int obstacleIndex2 = rightContact ?
            obstacleIndex - SCAN_INTERVAL:
            obstacleIndex + SCAN_INTERVAL;
    return !isnan(curRanges[obstacleIndex2]);
}

void TurtleBotLab5::adjustToMLine(){
    mLine.orentation = atan2(goalPoint.y - curPosition.y,
                             goalPoint.x - curPosition.x);
    mLine.point = curPosition;
    rotate(mLine.orentation - yaw);
}

void TurtleBotLab5::fullStop(){
    curVel.linear.x = 0;
    curVel.angular.z = 0;
    ros::Rate loop_rate(LOOP_RATE);
    loop_rate.sleep();
    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = ros::Duration(0.5); 
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    while(ros::Time::now() < endTime)
    {
        this->vel_pub.publish(curVel);
        loop_rate.sleep();
    }
}

void TurtleBotLab5::vel_from_wheels(double vl, double vr, double sec){
    double lin_vel = 0.5 * (vl + vr);
    double ang_vel = (vr - vl) / BASE_DIAMETER;
    curVel.linear.x = lin_vel;
    curVel.angular.z = ang_vel;
    ros::Rate loop_rate(LOOP_RATE);
    loop_rate.sleep();
    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = ros::Duration(sec); 
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    while(ros::Time::now() < endTime)
    {
        this->vel_pub.publish(curVel);
        loop_rate.sleep();
        if(collide) break;
    }
}

void TurtleBotLab5::drive_arc(double radius, double omega, double angle){
    double vl, vr;
    if(angle > 0){
        vl= omega *(radius - BASE_DIAMETER / 2.0);
        vr = omega *(radius + BASE_DIAMETER / 2.0);
    }
    else{
        vl= omega *(radius + BASE_DIAMETER / 2.0);
        vr = omega *(radius - BASE_DIAMETER / 2.0);
    }
    double curAngle = 0;
    while(fabs(curAngle) < fabs(angle)){
        double start_orentation = yaw;
        vel_from_wheels(vl, vr, 0.1);
        double deltaAngle;
        if(angle > 0){
            deltaAngle = yaw - start_orentation;
        }
        else{
            deltaAngle = start_orentation - yaw;
        }
        if(fabs(deltaAngle) > TURN_THRESHOLD){
            if(curAngle < 0) deltaAngle += 2 * PI;
            curAngle += deltaAngle;
        }
        curAngle = normalize_rad(curAngle);
    }
    fullStop();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lab5");
    TurtleBotLab5 turtleBotLab5;
}
