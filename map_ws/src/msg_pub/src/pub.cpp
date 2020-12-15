#include<iostream>
#include<fstream>
#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher_node");
  ros::start();

  ros::NodeHandle nodehandle;

  ros::Publisher pub_kf_and_pts = nodehandle.advertise<geometry_msgs::PoseArray>("pts_and_pose", 100);

  std::string filePath = "/home/slam/map_ws/src/p_occupany_mapping/data/data3.txt";

  std::ifstream f;
  f.open(filePath.c_str());

  std::string s;
  while(getline(f, s)){
    geometry_msgs::PoseArray kf_and_pts;
    std::stringstream ss;

    // frame_id
    uint32_t frame_id;
    ss << s;
    ss >> frame_id;
    kf_and_pts.header.seq = frame_id;
    ROS_INFO("Frame%u", kf_and_pts.header.seq);

    // 相机位置
    geometry_msgs::Pose cam_pose;
    double x, y, z;
    ss << s;
    ss >> x;
    ss << s;
    ss >> y;
    ss << s;
    ss >> z;
    cam_pose.position.x = x;
    cam_pose.position.y = y;
    cam_pose.position.z = z;
    // Pose[0]
    kf_and_pts.poses.push_back(cam_pose);
    ROS_INFO("cam_pose : x = %lf, y = %lf, z = %lf", 
    cam_pose.position.x, cam_pose.position.y, cam_pose.position.z);

    // 前帧的地图点个数
    geometry_msgs::Pose pt_num;
    int num;
    ss << s;
    ss >> num;
    pt_num.position.x = pt_num.position.y = pt_num.position.z = num;
    ROS_INFO("Point_num = %lf", pt_num.position.x);
    // Pose[1]
    kf_and_pts.poses.push_back(pt_num);

    // 所有地图的位置
    for (int i = 0; i < num; ++i) {
      geometry_msgs::Pose pt_pose;
      ss << s;
      ss >> x;
      ss << s;
      ss >> y;
      ss << s;
      ss >> z;
      // Pose[n]
      pt_pose.position.x = x;
      pt_pose.position.y = y;
      pt_pose.position.z = z;
      kf_and_pts.poses.push_back(pt_pose);
      ROS_INFO("point_pose : x = %lf, y = %lf, z = %lf", pt_pose.position.x, pt_pose.position.y, pt_pose.position.z);
    }
    pub_kf_and_pts.publish(kf_and_pts);
    ros::Duration(0.1).sleep();
    // ros::spin();
    // }
  }

  ROS_INFO("END");
  ros::spin();

  return 0;
}
