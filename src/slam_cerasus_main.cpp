#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include "cerasus_slam_class.h"
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include "slam_cerasus_const.h"

void Callback(std_msgs::Float64 ang);
void realcall();
void Callback2(std_msgs::Float64 _rpm);
void Callback3(sensor_msgs::LaserScan lidar);
static cerasus_slam_class CSC;
static ros::Publisher slam_pub;
int main(int argc, char** argv){
	ros::init(argc, argv, "slam_cerasus");
    //ROS_INFO_STREAM("1");
	ros::NodeHandle nh;
    //ROS_INFO_STREAM("2");
	ros::Subscriber sub = nh.subscribe("/motor/steering_angle", 1, Callback);

    ros::Subscriber sub2 = nh.subscribe("/motor/rpm", 1, Callback2);

    ros::Subscriber sub3 = nh.subscribe("/scan", 1, Callback3);
    slam_pub = nh.advertise<geometry_msgs::Vector3>("/slam", 1000);//Root Commandros::Publisher root_pub = nh.advertise<std_msgs::Int16>("Cerasus/root", 1000);//Root Command

    ros::spin();
	return 0;
}

void Callback(std_msgs::Float64 ang){
    //ROS_INFO_STREAM("3");
    static int countCall=0;
    std::cout<<"\n1:"<<countCall++;
    CSC.Update_Slam_imu(ang.data/2000);
}

void Callback2(std_msgs::Float64 _rpm){

    static int countCall=0;
    std::cout<<"\n2:"<<countCall++;
    CSC.Update_Slam_rpm(_rpm.data*0.00045);

    double x,y,th;
    geometry_msgs::Vector3 output;
    CSC.Get_Slam(&x,&y,&th);
    output.x=x;
    output.y=y;
    output.z=th;
    slam_pub.publish(output);
}

void Callback3(sensor_msgs::LaserScan lidar){
    //lidar.ranges[]

    static int countCall=0;
    std::cout<<"\n3**********:"<<countCall++;
    double Data[360];
    int count=0;
    for(int i=0;i<360;i++){
        if(lidar.ranges[i]>=CONST_LIDARMIN&&lidar.ranges[i]<=CHARCLASS_NAME_MAX){
            Data[count]=lidar.ranges[i];
        }
    }
    //double x,y,th;
    //geometry_msgs::Vector3 output;
    //CSC.Update_Slam(Data,count);
   // output.x=x;
   // output.y=y;
   // output.z=th;
   // slam_pub.publish(output);
   }
    
  //  static tf::TransformBroadcaster br;
//    br.sendTransform(tf::StampedTransform(transform,
  //                                        ros::Time::now(),
    //                                      "odom",
	//				"laser"
      //                                    ));

                                          