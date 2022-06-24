#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#include "video_stream/video_stream.h"


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "video_stream");

    VideoStream vs("my_camera/image_raw");
    
    ros::Rate loop_rate(10);

    while (ros::ok()) {

      vs.capture_img();
      
      ros::spinOnce();
      loop_rate.sleep();
      
    }
    return 0;
}

void VideoStream::capture_img(){
  if(camera.isOpened()){
    cv::Mat frame;
    camera >> frame;
    //cv::imshow("window", frame);
    //cv::waitKey(10);

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header;

    header.seq = counter; 
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);
    img_bridge.toImageMsg(img_msg);

    video_pub.publish(img_msg);
    counter++;
  }
}

// https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros
// https://answers.ros.org/question/9765/how-to-convert-cvmat-to-sensor_msgsimageptr/
