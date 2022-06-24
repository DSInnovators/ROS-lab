#ifndef VS_VIDEO_STREAM_H_
#define VS_VIDEO_STREAM_H_

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class VideoStream{
  private:
  cv::VideoCapture camera;
  ros::NodeHandle nh;
  ros::Publisher video_pub;
  int counter=0;

  public:
  VideoStream(std::string topic_){
    camera.open(0);
    video_pub = nh.advertise<sensor_msgs::Image>(topic_, 10);
  }
  ~VideoStream(){
    camera.release();
  }

  void capture_img();
};


#endif

//std::string hdr_gst_str ("udpsrc port=5601 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! queue ! videoconvert ! video/x-raw ! appsink");
//const char *pipeline = " tcambin serial=15810833 ! video/x-raw, format=BGRx, width=1280,height=960, framerate=25/1 ! videoconvert ! appsink";
//cv::VideoCapture camera(hdr_gst_str, cv::CAP_GSTREAMER);