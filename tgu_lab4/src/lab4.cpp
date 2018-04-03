/**
 * File Name:   lab4.cpp
 * Author:      Tianyi Gu, Zhuo Xu
 * Assignment:  Lab4
 * Date:        Feb-16-2018 
 * Descroption: A ROS node include a kinematic model for the
 * TurtleBot and program the TurtleBot for differential
 * drive motion, simple trajectory generation and odometry
 * reporting.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include "nav_msgs/Odometry.h"
#include <boost/thread/thread.hpp>
#include <math.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/BumperEvent.h>

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

class TurtleBotLab4
{
  public:
    TurtleBotLab4();

  private:
    void odom_thread();
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void bumper_thread();
    void bumper_callback(const kobuki_msgs::BumperEventConstPtr msg);
    void velPub_thread();
    void executeTrajectory();
    void drive_straight(double speed, double distance);
    void rotate(double angle);
    void drive_arc(double radius, double omega, double angle);
    void fullStop();
    void vel_from_wheels(double vl,double vr, double sec);
    

    ros::NodeHandle nodeHandle_pub, nodeHandle_sub_odom, nodeHandle_sub_bumper;

    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber bumper_sub;

    geometry_msgs::Twist curVel;
    nav_msgs::Odometry curOdom;
    double roll, pitch, yaw;
    bool trig = false;
    
};

TurtleBotLab4::TurtleBotLab4(){
    boost::thread t_odom( &TurtleBotLab4::odom_thread, this);
    boost::thread t_velPub( &TurtleBotLab4::velPub_thread, this);
    boost::thread t_bumper( &TurtleBotLab4::bumper_thread, this);
    t_odom.join();
    t_bumper.join();
    t_velPub.join();
}

void TurtleBotLab4::odom_thread(){
    odom_sub = nodeHandle_sub_odom.subscribe<nav_msgs::Odometry>(
        "odom", 1, &TurtleBotLab4::odom_callback, this);
    ros::spin();
}

void TurtleBotLab4::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    curOdom.pose.pose.position.x = msg->pose.pose.position.x;
    curOdom.pose.pose.position.y = msg->pose.pose.position.y;
    curOdom.pose.pose.position.z = msg->pose.pose.position.z;
    curOdom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    curOdom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    curOdom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    curOdom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    curOdom.twist.twist.linear.x = msg->twist.twist.linear.x;
    curOdom.twist.twist.linear.y = msg->twist.twist.linear.y;
    curOdom.twist.twist.linear.z = msg->twist.twist.linear.z;
    curOdom.twist.twist.angular.x = msg->twist.twist.angular.x;
    curOdom.twist.twist.angular.y = msg->twist.twist.angular.y;
    curOdom.twist.twist.angular.z = msg->twist.twist.angular.z;
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
    //          msg->pose.pose.position.x,
    //          msg->pose.pose.position.y,
    //          msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",
    //          msg->pose.pose.orientation.x,
    //          msg->pose.pose.orientation.y,
    //          msg->pose.pose.orientation.z,
    //          msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",
    //             msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    // ROS_INFO("roll: [%f], pitch: [%f], yaw: [%f]", roll, pitch,yaw);
}

void TurtleBotLab4::bumper_thread(){
    bumper_sub = nodeHandle_sub_bumper.subscribe
            <kobuki_msgs::BumperEventConstPtr>(
        "/mobile_base/events/bumper", 1, &TurtleBotLab4::bumper_callback, this);
}

void TurtleBotLab4::bumper_callback(const kobuki_msgs::BumperEventConstPtr msg){
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED){
        trig = true;
        ROS_INFO("bump...  ");
    }
}

void TurtleBotLab4::velPub_thread(){
    vel_pub =  nodeHandle_pub.advertise<geometry_msgs::Twist>(
        "cmd_vel_mux/input/teleop", 1, true);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        if (trig){
            executeTrajectory();
            trig = false;
        }
        loop_rate.sleep();
    }
    // executeTrajectory();
}

void TurtleBotLab4::executeTrajectory(){
    drive_straight(0.5, 0.6);
    rotate(-PI / 2.0);
    drive_arc(0.15, 0.5, -PI);
    rotate(3 * PI / 4.0);
    drive_straight(0.5, 0.42);
}

void TurtleBotLab4::drive_straight(double speed, double distance){
    geometry_msgs::Point start_position = curOdom.pose.pose.position;
    double curDis = 0;
    while(curDis < distance){
        ROS_INFO("curDis: [%f]",curDis);
        vel_from_wheels(speed, speed, 0.1);
        geometry_msgs::Point cur_position = curOdom.pose.pose.position;
        curDis = sqrt(pow(cur_position.x - start_position.x, 2.0) +
                      pow(cur_position.y - start_position.y, 2.0));
    }
    fullStop();
    
}

void TurtleBotLab4::rotate(double angle){
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
        ROS_INFO("curAngle: [%f]",curAngle);
        double start_orentation = yaw;
        // ROS_INFO("startOren: [%f]",yaw);
        vel_from_wheels(vl, vr, 0.1);
        // ROS_INFO("curOren: [%f]",yaw);
        double deltaAngle;
        if(angle > 0){
            deltaAngle = yaw - start_orentation;
        }
        else{
            deltaAngle = start_orentation - yaw;
        }
        if(fabs(deltaAngle) > 0.02){
            if(curAngle < 0) deltaAngle += 2 * PI;
            curAngle += deltaAngle;
        }
    }
    fullStop();
}

void TurtleBotLab4::drive_arc(double radius, double omega, double angle){
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
        // ROS_INFO("startOren_arc: [%f]",yaw);
        double start_orentation = yaw;
        vel_from_wheels(vl, vr, 0.1);
        // ROS_INFO("curOren_arc: [%f]",yaw);
        double deltaAngle;
        if(angle > 0){
            deltaAngle = yaw - start_orentation;
        }
        else{
            deltaAngle = start_orentation - yaw;
        }
        if(fabs(deltaAngle) > 0.02){
            if(curAngle < 0) deltaAngle += 2 * PI;
            curAngle += deltaAngle;
        }
        curAngle = normalize_rad(curAngle);
        ROS_INFO("curAngle_arc: [%f]",curAngle);
    }
    fullStop();
}

void TurtleBotLab4::fullStop(){
    curVel.linear.x = 0;
    curVel.angular.z = 0;
    ros::Rate loop_rate(50);
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

void TurtleBotLab4::vel_from_wheels(double vl, double vr, double sec){
    double lin_vel = 0.5 * (vl + vr);
    double ang_vel = (vr - vl) / BASE_DIAMETER;
    curVel.linear.x = lin_vel;
    curVel.angular.z = ang_vel;
    ros::Rate loop_rate(50);
    loop_rate.sleep();
    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = ros::Duration(sec); 
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    // ROS_INFO_STREAM("begintime:" << beginTime);
    while(ros::Time::now() < endTime)
    {
        // ROS_INFO_STREAM("now:" << ros::Time::now());
        this->vel_pub.publish(curVel);
        loop_rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lab4");
    TurtleBotLab4 turtleBotLab4;
}
