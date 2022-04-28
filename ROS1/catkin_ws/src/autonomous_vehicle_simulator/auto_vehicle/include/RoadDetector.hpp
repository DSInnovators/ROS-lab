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
    cv::Mat deNoise(cv::Mat inputImage);                                                                         // Apply Gaussian blurring to the input Image
    cv::Mat edgeDetector(cv::Mat img_noise);                                                                     // Filter the image to obtain only edges
    cv::Mat mask(cv::Mat img_edges);                                                                             // Mask the edges image to only care about ROI
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);                                                         // Detect Hough lines in masked edges image
    std::vector<std::vector<cv::Vec4i>> lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);         // Sprt detected lines by their slope into right and left lines
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i>> left_right_lines, cv::Mat inputImage); // Get only one line for each side of the lane
    std::string getDirectionFromLines(std::vector<std::vector<cv::Vec4i>> left_right_lines, cv::Mat inputImage);
    std::string predictTurn();                                                                               // Determine if the lane is turning or not by calculating the position of the vanishing point
    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn);                             // Plot the resultant lane and turn prediction in the frame.
};

#endif //ROAD_DETECTOR_HPP_