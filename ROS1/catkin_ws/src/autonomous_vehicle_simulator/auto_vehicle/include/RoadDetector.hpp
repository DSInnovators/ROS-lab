#ifndef ROAD_DETECTOR_HPP_
#define ROAD_DETECTOR_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

const std::string MOVE_LEFT  = "move_left";
const std::string MOVE_RIGHT = "move_right";
const std::string MOVE_FORWARD = "move_forward";
const std::string MOVE_STOP = "stop";
const std::string TURN_LEFT = "turn_left";
const std::string TURN_RIGHT = "turn_right";

/**
 *@brief Definition of the RoadDetector class. It contains all the functions and variables depicted in the
    *@brief Activity diagram and UML Class diagram.
    *@brief It detects the lanes in an image if a highway and outputs the
    *@brief same image with the plotted lane.
    */
class RoadDetector
{
private:
    double img_size;
    double img_center;
    bool left_flag = false;  // Tells us if there's left boundary of lane detected
    bool right_flag = false; // Tells us if there's right boundary of lane detected
    cv::Point right_b;       // Members of both line equations of the lane boundaries:
    double right_m;          // y = m*x + b
    cv::Point left_b;        //
    double left_m;           //

public:
    cv::Mat deNoise(const cv::Mat& inputImage) const;                                                                   // Apply Gaussian blurring to the input Image
    cv::Mat edgeDetector(const cv::Mat& img_noise) const;                                                                     // Filter the image to obtain only edges
    cv::Mat mask(const cv::Mat& img_edges) const;                                                                             // Mask the edges image to only care about ROI
    const std::vector<cv::Vec4i> houghLines(const cv::Mat& img_mask);                                                  // Detect Hough lines in masked edges image
    const std::vector<std::vector<cv::Vec4i>> lineSeparation(const std::vector<cv::Vec4i>& lines, const cv::Mat& img_edges);         // Sprt detected lines by their slope into right and left lines    
    const std::string& getDirectionFromLines(const std::vector<std::vector<cv::Vec4i>>& left_right_lines, const cv::Mat& inputImage);    
};

#endif //ROAD_DETECTOR_HPP_
