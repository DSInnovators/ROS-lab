#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <auto_vehicle_msgs/Status.h>

#include "../include/RoadDetector.h"
#include "../include/vehicle_control.h"

const std::string AUTOMATE_VEHICLE = "automate_vehicle";
const std::string IMAGE_TOPIC = "/vehicle_camera/image_raw";

ros::Publisher pub;
ros::Publisher statusPub;


std::shared_ptr<CommandPublisher> pcmdPublisher;




void moveVehicle(const std::string direction,
                 std::shared_ptr<CommandPublisher> pcmdPublisher,std::vector<cv::Vec4i> lines)
{
    std::cout<<"Car moving\n";
    pcmdPublisher->setSpeed(0.25);
    pcmdPublisher->setTurn(0.25);
    char key_code = 'i';
    if (direction == MOVE_LEFT)
        key_code = 'u';
    if (direction == MOVE_RIGHT)
        key_code = 'o';
    if (direction == TURN_LEFT)
        key_code = 'j';
    if (direction == TURN_RIGHT)
        key_code = 'l';
    if (direction == MOVE_FORWARD)
    {
        pcmdPublisher->setSpeed(0.5);
        pcmdPublisher->setTurn(0.0);
        key_code = 'i';
    }
    if (direction == MOVE_STOP)
        key_code = 't';

    pcmdPublisher->publishCommand(key_code, pub, statusPub, lines);

}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat imgFrame = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat imgDeNoise, imgEdges, imgMask, imgLines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i>> left_right_lines;
    // std::vector<cv::Point> lane;
    std::string direction;
    // int flag_plot = -1;
    // int i = 0;

    RoadDetector roadDetector;
    try
    {
        
        // output_file2.write("{:.2f},{:.2f},{:.2f}\n".format(slope_abs, speed * 10, turn))
        // Denoise the image using a Gaussian filter
        imgDeNoise = roadDetector.deNoise(imgFrame);

        // Detect edges in the image
        imgEdges = roadDetector.edgeDetector(imgDeNoise);

        // Mask the image so that we only get the ROI
        imgMask = roadDetector.mask(imgEdges);

        // Obtain Hough lines in the cropped image
        lines = roadDetector.houghLines(imgMask);
        std::cout<<"lines one "<<lines[1] <<std::endl;
        std::cout<<"lines dissect "<<lines[1][0] <<std::endl;
        // outfile << lines[1][0] << std::endl;
        // std::cout<<"one data write"<<std::endl;


       


        if (false == lines.empty())
        {
            // Separate lines into left and right lines
            left_right_lines = roadDetector.lineSeparation(lines, imgEdges);

            // Predict the turn by determining the vanishing point of the the lines
            direction = roadDetector.getDirectionFromLines(left_right_lines, imgFrame);

            // move the vehicle
            moveVehicle(direction, pcmdPublisher, lines);

            // i += 1;
            // // cv::waitKey(25);
        }
        // else
        // {
        //     flag_plot = -1;
        // }

    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Built FROM C++ [%s]\n", AUTOMATE_VEHICLE.c_str());
    ros::init(argc, argv, AUTOMATE_VEHICLE);

    ros::NodeHandle nodeHandle;
    

    pcmdPublisher = std::shared_ptr<CommandPublisher>(new CommandPublisher());

    pub = nodeHandle.advertise<geometry_msgs::Twist>(COMMAND, 10);
    statusPub = nodeHandle.advertise<auto_vehicle_msgs::Status>("vehicle_data", 1000);
    

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 100, imageCallback);

    ros::spin();

    return 0;
}
