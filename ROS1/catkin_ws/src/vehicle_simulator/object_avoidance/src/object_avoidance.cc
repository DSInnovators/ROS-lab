// "Copyright 2022 <Name>"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "object_avoidance/object_avoidance.h"


void ObjectAvoidance::addClassList(std::string filePath) {
  std::ifstream ifs(filePath);
  std::string line;
  while (getline(ifs, line)) {
    class_list.push_back(line);
  }
}

void ObjectAvoidance::loadModel(std::string filePath) {
  // net = readNet("YOLOv5s.onnx");
  net = cv::dnn::readNetFromONNX(filePath);
}

void ObjectAvoidance::draw_label(cv::Mat& input_image,
                                 std::string label, int left, int top) {
  // Display the label at the top of the bounding box.
  int baseLine;
  cv::Size label_size = cv::getTextSize(label, FONT_FACE, FONT_SCALE,
                                        THICKNESS, &baseLine);
  top = cv::max(top, label_size.height);
  // Top left corner.
  cv::Point tlc = cv::Point(left, top);
  // Bottom right corner.
  cv::Point brc = cv::Point(left + label_size.width, top + label_size.height + baseLine);
  // std::cout << "Top: " << top << " | Left: " << left << " || Bottom: " <<
  // (top + label_size.height + baseLine) << " Right: " <<
  // (left + label_size.width) << "\n";
  // Draw white rectangle.
  rectangle(input_image, tlc, brc, BLACK, cv::FILLED);
  // Put the label on the black rectangle.
  putText(input_image, label, cv::Point(left, top + label_size.height),
                                    FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

std::vector<cv::Mat> ObjectAvoidance::pre_process(cv::Mat &input_image) {
  // cv::Mat newimg;
  // cv::resize(input_image,newimg, cv::Size(640, 640), cv::INTER_LINEAR);
  // std::cout<<"Row: "<<frame.rows<<" Col: "<<frame.cols<<"\n";
  cv::resize(input_image, input_image, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::INTER_LINEAR);

  // std::cout<<"ImgSize: "<<input_image.rows <<" "<<input_image.cols<<std::endl;
  // Convert to blob.
  cv::Mat blob;
  cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);

  net.setInput(blob);
  // std::cout<<"blob: "<<blob.rows <<" "<<blob.cols<<std::endl;
  // Forward propagate.
  std::vector<cv::Mat> outputs;
  net.forward(outputs, net.getUnconnectedOutLayersNames());
  return outputs;
}

cv::Mat ObjectAvoidance::post_process(cv::Mat &input_image,
                                      std::vector<cv::Mat> &outputs) {
  // Initialize vectors to hold respective outputs while unwrapping     detections.
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  // Resizing factor.
  float x_factor = input_image.cols / INPUT_WIDTH;
  float y_factor = input_image.rows / INPUT_HEIGHT;
  float *data = reinterpret_cast<float *> (outputs[0].data);
  const int dimensions = 85;
  // 25200 for default size 640.
  const int rows = 25200;
  // Iterate through 25200 detections.
  for (int i = 0; i < rows; ++i) {
    float confidence = data[4];
    // Discard bad detections and continue.
    if (confidence >= CONFIDENCE_THRESHOLD) {
      float * classes_scores = data + 5;
      // Create a 1x85 Mat and store class scores of 80 classes.
      cv::Mat scores(1, class_list.size(), CV_32FC1, classes_scores);
      // Perform minMaxLoc and acquire the index of best class  score.
      cv::Point class_id;
      double max_class_score;
      minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
      // Continue if the class score is above the threshold.
      if (max_class_score > SCORE_THRESHOLD) {
        // Store class ID and confidence in the pre-defined respective vectors.
        confidences.push_back(confidence);
        class_ids.push_back(class_id.x);
        // Center.
        float cx = data[0];
        float cy = data[1];
        // Box dimension.
        float w = data[2];
        float h = data[3];
        // Bounding box coordinates.
        int left = static_cast<int>((cx - 0.5 * w) * x_factor);
        int top = static_cast<int>((cy - 0.5 * h) * y_factor);
        int width = static_cast<int>(w * x_factor);
        int height = static_cast<int>(h * y_factor);
        // std::cout<<"Top: "<<top<<" | Left: "<<left<<" || \
        // width: "<<width<<" height: "<<height<<"\n";
        // Store good detections in the boxes vector.
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }
    // Jump to the next row.
    data += 85;
  }

  // Perform Non-Maximum Suppression and draw predictions.
  std::vector<int> indices;
  robot_direction = 0;
  cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
  for (int i = 0; i < indices.size(); i++) {
    int idx = indices[i];
    cv::Rect box = boxes[idx];
    int left = box.x;
    int top = box.y;
    int width = box.width;
    int height = box.height;
    // Draw bounding box.
    rectangle(input_image, cv::Point(left, top), cv::Point(left + width, top + height), BLUE, 3*THICKNESS);
    // Get the label for the class name and its confidence.
    std::string label = cv::format("%.2f", confidences[idx]);
    label = class_list[class_ids[idx]] + ":" + label;
    // Draw class labels.
    draw_label(input_image, label, left, top);

    // Robot Direction Logic
    if (left > (INPUT_WIDTH-left-width))
      robot_direction = 1;
    else
      robot_direction = -1;
    // std::cout<<"Robot Direction: "<<robot_direction<<"\n";
  }
  return input_image;
}

void ObjectAvoidance::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

  std::vector<cv::Mat> detections;     // Process the image.
  detections = pre_process(frame);
  // cv::Mat img = post_process(frame.clone(), detections, class_list);
  cv::Mat img = post_process(frame, detections);
  // Put efficiency information.
  // The function getPerfProfile returns the overall time for
  // inference(t) and the timings for each of the layers(in layersTimes).
  std::vector<double> layersTimes;
  double freq = cv::getTickFrequency() / 1000;
  double t = net.getPerfProfile(layersTimes) / freq;
  std::string label = cv::format("Inference time : %.2f ms", t);
  putText(img, label, cv::Point(20, 40), FONT_FACE, FONT_SCALE, RED);

  update_direction();
  cv::imshow("Output", img);
  cv::waitKey(1);
}

void ObjectAvoidance::update_direction() {
  geometry_msgs::Twist msg;
  msg.linear.x = forward_speed;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = turn_speed * robot_direction;
  std::cout << "X: " << forward_speed << " | Z: " << turn_speed * robot_direction << "\n";
  teleop_pub.publish(msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "object_avoidance");
  ObjectAvoidance object_avoidance;
  std::string file_source = ros::package::getPath("object_avoidance");

  std::string class_source = file_source + "/src/files/coco.names";
  std::string model_source = file_source + "/src/files/yolov5s.onnx";

  // Load class list.
  object_avoidance.addClassList(class_source);

  // Load image.
  // cv::Mat frame;
  // frame = cv::imread("/home/tanjim/traffic.jpg");
  // if(frame.empty()){
  //    std::cout<<"No Img found!";
  //    return 0;
  // }
  // Load model.
  object_avoidance.loadModel(model_source);

  ros::spin();

  return 0;
}
