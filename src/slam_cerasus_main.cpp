#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include "cerasus_slam_class.h"
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
void Callback(std_msgs::Float64 ang);
void realcall();
void Callback2(std_msgs::Float64 _rpm);
void Callback3(sensor_msgs::LaserScan lidar);
cerasus_slam_class CSC;

int main(int argc, char** argv){
	ros::init(argc, argv, "slam_cerasus");
    //ROS_INFO_STREAM("1");
	ros::NodeHandle nh;
    //ROS_INFO_STREAM("2");
	ros::Subscriber sub = nh.subscribe("/imu", 1, Callback);

    ros::Subscriber sub2 = nh.subscribe("/motor/rpm", 1, Callback2);
	ros::spin();
	return 0;
}

void Callback(std_msgs::Float64 ang){
    //ROS_INFO_STREAM("3");
    CSC.Update_Slam_imu(ang.data);
}

void Callback2(std_msgs::Float64 _rpm){
    CSC.Update_Slam_rpm(_rpm.data);
}

void Callback3(sensor_msgs::LaserScan lidar){
    
}
    
  //  static tf::TransformBroadcaster br;
//    br.sendTransform(tf::StampedTransform(transform,
  //                                        ros::Time::now(),
    //                                      "odom",
	//				"laser"
      //                                    ));

                                          