#ifndef COMMAND_PUBLISHER_HPP_
#define COMMAND_PUBLISHER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>



const std::string COMMAND = "vehicle/cmd_vel";

// Map for movement keys
std::map<char, std::vector<float>>  moveBindings{
        {'i', {1, 0, 0, 0}},
        {'o', {1, 0, 0, -1}},
        {'j', {0, 0, 0, 1}},
        {'l', {0, 0, 0, -1}},
        {'u', {1, 0, 0, 1}},
        {',', {-1, 0, 0, 0}},
        {'.', {-1, 0, 0, 1}},
        {'m', {-1, 0, 0, -1}},
        {'O', {1, -1, 0, 0}},
        {'I', {1, 0, 0, 0}},
        {'J', {0, 1, 0, 0}},
        {'L', {0, -1, 0, 0}},
        {'U', {1, 1, 0, 0}},
        {'<', {-1, 0, 0, 0}},
        {'>', {-1, -1, 0, 0}},
        {'M', {-1, 1, 0, 0}},
        {'t', {0, 0, 1, 0}},
        {'b', {0, 0, -1, 0}},
        {'k', {0, 0, 0, 0}},
        {'K', {0, 0, 0, 0}}};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings{
    {'q', {1.1, 1.1}},
    {'z', {0.9, 0.9}},
    {'w', {1.1, 1}},
    {'x', {0.9, 1}},
    {'e', {1, 1.1}},
    {'c', {1, 0.9}}};

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
    float speed;            // Linear velocity (m/s)
    float turn;             // Angular velocity (rad/s)
    float x, y, z, th; // Forward/backward/neutral direction vars
    char key;
    std::string direction;
    // static ros::Publisher pub;    
   

public:
    CommandPublisher()
    {
        // Init variables
        speed = 0.15;            // Linear velocity (m/s)
        turn = 0.15;             // Angular velocity (rad/s)
        x = 0;
        y = 0; 
        z = 0;
        th = 0; // Forward/backward/neutral direction vars
        key = ' ';        
    }

    // static void setPublisher(ros::Publisher publisher);
    // static ros::Publisher getPublisher();

    void setSpeed(float speed)
    {
        this->speed = speed;
    }

    std::string getDirection()
    {
        return direction;
    }

    void publishCommand(char key_code, ros::Publisher pub) 
    {        
        // Create Twist message
        geometry_msgs::Twist twist;

        // printf("%s", msg);
        printf("\nCurrent: speed %f\tturn %f | Awaiting command...", speed, turn);
        printf("\nCurrent: key %c \n", key_code);
    
        // Get the pressed key
        key = key_code; // getch();

        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];

            //printf("\nmovebindings Current: speed %f\tturn %f | Last command: %c\n", speed, turn, key);
        }

        // Otherwise if it corresponds to a key in speedBindings
        else if (speedBindings.count(key) == 1)
        {
            // Grab the speed data
            speed = speed * speedBindings[key][0];
            turn = turn * speedBindings[key][1];

            //printf("\nspeed bindings Current: speed %f\tturn %f | Last command: %c\n", speed, turn, key);
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
                printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
                return;
            }

            //printf("\nCurrent: speed %f\tturn %f | Invalid command! %c\n", speed, turn, key);
        }

        // Update the Twist message
        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = z * speed;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks        
        printf("\nPublishing Current: speed %f\tturn %f | Last command: %c\n", speed, turn, key);
        pub.publish(twist);
        ros::spinOnce();
        
        //return;
    }    
};


#endif // COMMAND_PUBLISHER_HPP