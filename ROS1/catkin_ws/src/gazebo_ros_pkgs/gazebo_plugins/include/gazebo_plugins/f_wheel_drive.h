#ifndef F_WHEEL_DRIVE_H_
#define F_WHEEL_DRIVE_H_
#include <geometry_msgs/Twist.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>

#include <ros/ros.h>

namespace gazebo {

enum {
  LEFT,
  RIGHT,
};

enum {
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACKWARD_LEFT,
  BACKWARD_RIGHT,
};

  class F_Wheel_Drive : public ModelPlugin
  {
   private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    //boost::shared_ptr<ros::NodeHandle> rosnode_;

    //ros::NodeHandle node_handler;

    std::shared_ptr<ros::NodeHandle> rosnode_;
    ros::Subscriber node_sub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;

    float x_,rot_;
    double wheel_speed_[2];
    std::vector<physics::JointPtr> joints_;

    void updateVelocity();

    void QueueThread() {     
      while (this->rosnode_->ok()) {
        this->rosQueue.callAvailable();
        //printf("QueueThread!\n");
      }
    }

   public: 
    F_Wheel_Drive();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    // Called by the world update start event
    void OnUpdate();

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  };
}

#endif
