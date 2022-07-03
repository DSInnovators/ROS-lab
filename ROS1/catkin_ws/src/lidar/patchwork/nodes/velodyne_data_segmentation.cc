//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "patchwork/patchwork.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <cstdlib>
#include <iostream>


using PointType = pcl::PointXYZ;
using namespace std;

ros::Publisher CloudPublisher;
ros::Publisher PositivePublisher;
ros::Publisher NegativePublisher;
ros::Subscriber VelodyneSubscriber;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
bool        save_flag;


pcl::PointCloud<PointType> pc_ground;
pcl::PointCloud<PointType> pc_non_ground;
static double time_taken;

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void laserCallBack(const sensor_msgs::PointCloud2::ConstPtr& data) {
  
  sensor_msgs::PointCloud2 laser_data = sensor_msgs::PointCloud2(*data);

  pcl::PointCloud<PointType> new_data;
  new_data = cloudmsg2cloud<PointType>(laser_data);
  PatchworkGroundSeg->estimate_ground(new_data, pc_ground, pc_non_ground, time_taken);
  CloudPublisher.publish(cloud2msg(new_data));
  PositivePublisher.publish(cloud2msg(pc_ground));
  NegativePublisher.publish(cloud2msg(pc_non_ground));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "patchwork_segmentation");
    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchwork");
    ros::Rate loop_rate(10);
    
    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));
    // std::cout<<"Hereee\n";
    VelodyneSubscriber = nh.subscribe("velodyne_points", 10, laserCallBack);
    CloudPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100);
    PositivePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/P", 100);
    NegativePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/N", 100);


    //pcl::PointCloud<PointType> pc_curr;

    
    cout << "Operating patchwork..." << endl;


    ros::spin();

    return 0;
}
