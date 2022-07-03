// Type: sensor_msgs/LaserScan
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "lidar_tutorial/lidar_visualizer.h"

void Lidar_visualizer::cmdVelCallback(const sensor_msgs::PointCloud2::ConstPtr& cmd_msg) {
  this->mutex_.lock();
  this->rec_msg = sensor_msgs::PointCloud2(*cmd_msg);
  
  this->mutex_.unlock();
  //std::cout<<this->rec_msg.row_step<<"\n";
}

void Lidar_visualizer::publishData(/*sensor_msgs::PointCloud2 data*/) {
  //this->rec_msg.data[4] = static_cast<uint8_t>(5);
  //auto *ptr = this->rec_msg.data.data();
  //*((uint8_t*)(ptr+4)) = 5;
  //std::cout<<this->rec_msg.data[0]<<"\n";
  node_pub_.publish(this->rec_msg);
  std::cout<<this->rec_msg.row_step<<"\n";
}

int main(int argc, char **argv){
  Lidar_visualizer lidar_v(argc, argv);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {

    lidar_v.publishData();
    std::cout<<"Publish count: "<<count<<"\n";
    //ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}