#include <sensor_msgs/PointCloud2.h>
//#include <pcl_ros/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/passthrough.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>

using PointType = pcl::PointXYZ;

class PCL_segmentation
{
private:
  //std::shared_ptr<ros::NodeHandle> node_handler_;
  ros::NodeHandle node_handler_;
  ros::Subscriber node_sub_;
  ros::Publisher node_pub_;

  sensor_msgs::PointCloud2 rec_pointcloud2;
  sensor_msgs::PointCloud2 pub_pointcloud2;
  pcl::PointCloud<PointType> pcl_pointcloud;
  pcl::PointCloud<PointType> filtered_pcl_pointcloud;
  pcl::PointCloud<PointType>::Ptr filtered_pcl_pointcloud_ptr;
  //std::shared_ptr<pcl::PointCloud<PointType>>(&t_author);
  pcl::PointCloud<PointType>::Ptr pcl_pointcloud_ptr;

public:
  PCL_segmentation() {
    // this->node_handler_.reset(new ros::NodeHandle());
    this->node_sub_ = this->node_handler_.subscribe("velodyne_points", 10, &PCL_segmentation::cmdVelCallback,this);
    this->node_pub_ = this->node_handler_.advertise<sensor_msgs::PointCloud2>("laser_scan", 10, this);
  }
  //~PCL_segmentation();

  void cmdVelCallback(const sensor_msgs::PointCloud2::ConstPtr& cmd_msg) {
    this->rec_pointcloud2 = sensor_msgs::PointCloud2(*cmd_msg);
    pcl::fromROSMsg(this->rec_pointcloud2,this->pcl_pointcloud);
    //std::cout<<this->rec_pointcloud2.row_step<<"\n";

    this->segment_points();
    this->publishData();
  }

  void publishData() {
    pcl::toROSMsg(*(this->filtered_pcl_pointcloud_ptr), this->pub_pointcloud2);
    std::cout<<this->pub_pointcloud2.row_step<<"\n";
    node_pub_.publish(this->pub_pointcloud2);
  }

  void segment_points(){
    filtered_pcl_pointcloud_ptr = boost::make_shared<pcl::PointCloud<PointType> >(this->pcl_pointcloud);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (this->filtered_pcl_pointcloud_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.3, 1.0);
    pass.filter(*(this->filtered_pcl_pointcloud_ptr));

    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // // Create the segmentation object
    // pcl::SACSegmentation<PointType> segmentation;

    
    // // Optional
    // segmentation.setOptimizeCoefficients(true);

    // segmentation.setModelType(pcl::SACMODEL_PLANE );

    // segmentation.setMethodType(pcl::SAC_RANSAC );

    // segmentation.setDistanceThreshold(20.20);

    // segmentation.setInputCloud(&this->pcl_pointcloud);
    // segmentation.segment (*inliers, *coefficients);
  }

};



int main(int argc, char** argv){
  ros::init(argc,argv, "pcl_segmentation");

  PCL_segmentation pcl_seg;

  ros::spin();

  return 0;
}
