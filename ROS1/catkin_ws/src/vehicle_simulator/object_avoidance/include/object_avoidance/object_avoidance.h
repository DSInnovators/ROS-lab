// "Copyright 2022 <Name>"
#ifndef SRC_VEHICLE_SIMULATOR_OBJECT_AVOIDANCE_INCLUDE_OBJECT_AVOIDANCE_OBJECT_AVOIDANCE_H_
#define SRC_VEHICLE_SIMULATOR_OBJECT_AVOIDANCE_INCLUDE_OBJECT_AVOIDANCE_OBJECT_AVOIDANCE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

float forward_speed = 0.2;
float turn_speed = 0.5;


// Constants.
// float INPUT_WIDTH = 640.0;
// float INPUT_HEIGHT = 640.0;
const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.45;
const float CONFIDENCE_THRESHOLD = 0.45;


// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;


// Colors.
cv::Scalar BLACK = cv::Scalar(0, 0, 0);
cv::Scalar BLUE = cv::Scalar(255, 178, 50);
cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
cv::Scalar RED = cv::Scalar(0, 0, 255);


class ObjectAvoidance{
 private:
  std::vector<std::string> class_list;
  float robot_direction = 0;

 public:
  cv::dnn::Net net;  // Model
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher teleop_pub;

  ObjectAvoidance();
  void addClassList(std::string filePath);
  void loadModel(std::string filePath);
  void draw_label(cv::Mat& input_image, std::string label, int left, int top);
  std::vector<cv::Mat> pre_process(cv::Mat &input_image);
  cv::Mat post_process(cv::Mat &input_image, std::vector<cv::Mat> &outputs);
  void update_direction();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

ObjectAvoidance::ObjectAvoidance() {
  sub = nh.subscribe("my_camera/image_raw", 10, &ObjectAvoidance::imageCallback, this);
  teleop_pub = nh.advertise<geometry_msgs::Twist>("vehicle/cmd_vel", 10);
}

#endif  // SRC_VEHICLE_SIMULATOR_OBJECT_AVOIDANCE_INCLUDE_OBJECT_AVOIDANCE_OBJECT_AVOIDANCE_H_
