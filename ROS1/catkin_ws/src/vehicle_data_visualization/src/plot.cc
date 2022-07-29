
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <auto_vehicle_msgs/Status.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

const std::string vehicle_data_viz = "vehicle_data_visualization";
const std::string command_topic = "/vehicle_data";

std::vector<float> slope;
std::vector<float> turn;
std::vector<float> speed;
std::vector<int> curtime;

std::vector<float> slope2;
std::vector<float> turn2;
std::vector<float> speed2;
std::vector<int> curtime2;

int pos = 0;

void plotDataCallback(const auto_vehicle_msgs::Status::ConstPtr &msg)
{
    slope.push_back(msg->slope);
    turn.push_back(msg->turn);
    speed.push_back(msg->speed);
    curtime.push_back(msg->curtime);
}

void draw_graph(int size)
{
    for (; pos < size; pos++)
    {
        slope2.push_back(slope[pos]);
        std::cout << "thread function Executing " << pos << std::endl;
        turn2.push_back(turn[pos]);
        speed2.push_back(speed[pos]);
        curtime2.push_back(curtime[pos]);

        plt::plot(curtime2, speed2, "r");
        plt::plot(curtime2, slope2, "b");
        plt::plot(curtime2, turn2, "g");

        if (pos == 1)
        {
            plt::named_plot("speed", curtime2, speed2);
            plt::named_plot("slope", curtime2, slope2);
            plt::named_plot("turn", curtime2, turn2);
            plt::xlabel("x axis");
            plt::ylabel("y axis");

            // Add graph title
            plt::title("Vehicle Data Visualization");
            // Enable legend.
            plt::legend();
        }
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Built FROM C++ ROS LAB [%s]\n", vehicle_data_viz.c_str());
    ros::init(argc, argv, vehicle_data_viz);

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(command_topic, 1000, plotDataCallback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        int x = slope.size();

        if (x > 0)
        {
            draw_graph(x);
        }

        ros::spinOnce();
        loop_rate.sleep();

        // Display plot continuously
        plt::pause(0.01);
    }

    plt::close();

    return 0;
}
