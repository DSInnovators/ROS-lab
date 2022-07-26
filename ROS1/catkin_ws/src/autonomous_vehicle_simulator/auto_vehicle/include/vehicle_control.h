#ifndef AV_VEHICLE_CONTROL_H_
#define AV_VEHICLE_CONTROL_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <map>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

const std::string COMMAND = "vehicle/cmd_vel";

// Map for movement keys
// {key, {x, y, z, th}}
const std::map<char, std::array<int, 4>> moveBindings{
    {'i', {1, 0, 0, 0}}, {'o', {1, 0, 0, -1}}, {'j', {0, 0, 0, 1}}, {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}}, {',', {-1, 0, 0, 0}}, {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}, {'O', {1, -1, 0, 0}}, {'I', {1, 0, 0, 0}}, {'J', {0, 1, 0, 0}}, {'L', {0, -1, 0, 0}}, {'U', {1, 1, 0, 0}}, {'<', {-1, 0, 0, 0}}, {'>', {-1, -1, 0, 0}}, {'M', {-1, 1, 0, 0}}, {'t', {0, 0, 1, 0}}, {'b', {0, 0, -1, 0}}, {'k', {0, 0, 0, 0}}, {'K', {0, 0, 0, 0}}};

// Map for speed keys
// {key, {speed, turn}}
const std::map<char, std::array<float, 2>> speedBindings{
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}}, {'e', {1, 1.1}}, {'c', {1, 0.9}}};

// Reminder message
const char *msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";

class CommandPublisher
{
private:
    float speed;       // Linear velocity (m/s)
    float turn;        // Angular velocity (rad/s)
    float x, y, z, th; // Forward/backward/neutral direction vars
    char key;
    std::string direction;

public:
    CommandPublisher()
    {
        // Init variables
        speed = 0.15; // Linear velocity (m/s)
        turn = 0.15;  // Angular velocity (rad/s)
        x = 0;
        y = 0;
        z = 0;
        th = 0; // Forward/backward/neutral direction vars
        key = ' ';
    }

    void setSpeed(const float speed) { this->speed = speed; }
    void setTurn(const float turn) { this->turn = turn; }

    const std::string &getDirection() const { return direction; }

    void publishCommand(const char key_code, const ros::Publisher &pub, std::vector<cv::Vec4i> lines)
    {
        // Create Twist message
        geometry_msgs::Twist twist;

        double slope;

        cv::Point ini;
        cv::Point fini;
        double slope_thresh = 1;

        for (auto i : lines)
        {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            // Basic algebra: slope = (y1 - y0)/(x1 - x0)
            slope =
                (static_cast<double>(fini.y) - static_cast<double>(ini.y)) /
                (static_cast<double>(fini.x) - static_cast<double>(ini.x));

            
        }

        // Get the key
        key = key_code;
        std::cout << "Slope ...." << slope << std::endl;

        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = (moveBindings.at(key)).at(0);
            y = (moveBindings.at(key)).at(1);
            z = (moveBindings.at(key)).at(2);
            th = (moveBindings.at(key)).at(3);
        }

        // Otherwise if it corresponds to a key in speedBindings
        else if (speedBindings.count(key) == 1)
        {
            // Grab the speed data
            speed *= (speedBindings.at(key)).at(0);
            turn *= (speedBindings.at(key)).at(1);
        }

        // Otherwise, set the robot to stop
        else
        {
            x = 0;
            y = 0;
            z = 0;
            th = 0;

            // If ctrl-C (^C) was pressed, terminate the program
            if (key == '\x03')
            {
                printf(
                    "\n\n                 .     .\n              .  |\\-^-/|  .    \n  "
                    "           /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
                return;
            }
        }

        if(std::abs(slope) > 20) {
            turn = turn + 0.1;
        } 
        // if (std::abs(slope) > slope_thresh && std::abs(slope) < 30)
        // {
        //    turn = -0.4;
        // } 

        // Update the Twist message
        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = z * speed;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks
        ROS_INFO(
            "Publishing command:  Current speed %.2f | turn %.2f | Command: %c\n",
            speed, turn, key);
        pub.publish(twist);
    }
};

#endif // AV_VEHICLE_CONTROL_H_
