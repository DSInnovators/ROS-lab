#ifndef LT_LIDAR_VISUALIZER_H_
#define LT_LIDAR_VISUALIZER_H_
#include <thread>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>

class Lidar_visualizer
{
private:

  // ROS Variables
  std::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Subscriber node_sub_;
  ros::Publisher node_pub_;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  std::mutex mutex_;
  
  // Lidar Variable
  sensor_msgs::PointCloud2 rec_msg;

  void QueueThread() {     
    while (this->rosnode_->ok()) {
      this->rosQueue.callAvailable();
      //printf("QueueThread!\n");
    }
  }

public:
  Lidar_visualizer(int argc, char** argv);
  ~Lidar_visualizer();

  void cmdVelCallback(const sensor_msgs::PointCloud2::ConstPtr& cmd_msg);
  void publishData(/*sensor_msgs::PointCloud2 data*/);
};

Lidar_visualizer::Lidar_visualizer(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_visualizer",ros::init_options::NoSigintHandler);
  this->rosnode_.reset(new ros::NodeHandle());
  this->rosnode_->setCallbackQueue(&(this->rosQueue));
  this->node_sub_ = this->rosnode_->subscribe("velodyne_points", 10, &Lidar_visualizer::cmdVelCallback,this);
  this->rosQueueThread = std::thread(std::bind(&Lidar_visualizer::QueueThread, this));

  this->node_pub_ = this->rosnode_->advertise<sensor_msgs::PointCloud2>("laser_scan", 10, this);
}

Lidar_visualizer::~Lidar_visualizer()
{
}

#endif
