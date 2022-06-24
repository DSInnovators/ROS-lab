#ifndef OA_OBJECT_DETECTION_H_
#define OA_OBJECT_DETECTION_H_

#include <opencv2/opencv.hpp>

// Constants.
//float INPUT_WIDTH = 640.0;
//float INPUT_HEIGHT = 640.0;
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
cv::Scalar BLACK = cv::Scalar(0,0,0);
cv::Scalar BLUE = cv::Scalar(255, 178, 50);
cv::Scalar YELLOW = cv::Scalar(0, 255, 255);
cv::Scalar RED = cv::Scalar(0,0,255);


class ObjectDetection{
  private:
  std::vector<std::string> class_list;
  
  public:
  cv::dnn::Net net; // Model

  void addClassList(std::string filePath);
  void loadModel(std::string filePath);
  void draw_label(cv::Mat& input_image, std::string label, int left, int top);
  std::vector<cv::Mat> pre_process(cv::Mat &input_image);
  cv::Mat post_process(cv::Mat &input_image, std::vector<cv::Mat> &outputs);
};

#endif