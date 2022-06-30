#include <geometry_msgs/Twist.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "gazebo_plugins/f_wheel_drive.h"

namespace gazebo
{
  F_Wheel_Drive::F_Wheel_Drive():ModelPlugin() {

  }

  void F_Wheel_Drive::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    printf("Loading Model!\n");
    // Store the pointer to the model
    this->model = _parent;

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
    }

    this->rosnode_.reset(new ros::NodeHandle());
    this->rosnode_->setCallbackQueue(&(this->rosQueue));
    this->node_sub = this->rosnode_->subscribe("cmd_vel", 10, &F_Wheel_Drive::cmdVelCallback,this);
    this->rosQueueThread = std::thread(std::bind(&F_Wheel_Drive::QueueThread, this));
    physics::BasePtr bp = this->model->GetChild("car_wheel_1");
    std::cout<<bp->GetName()<<"\n";
    //physics::Model p = dynamic_cast<physics::Model>(bp);

    // joints_.push_back(this->model->GetJoint("base_link_JOINT_0"));
    // joints_.push_back(this->model->GetJoint("base_link_JOINT_1"));
    // joints_.push_back(this->model->GetJoint("base_link_JOINT_2"));
    // joints_.push_back(this->model->GetJoint("base_link_JOINT_3"));S

    // joints_[FORWARD_LEFT]->SetParam("fmax", 0, 100.0);
    // joints_[FORWARD_RIGHT]->SetParam("fmax", 0, 100.0);
    // joints_[BACKWARD_LEFT]->SetParam("fmax", 0, 100.0);
    // joints_[BACKWARD_RIGHT]->SetParam("fmax", 0, 100.0);

    std::cout<<BACKWARD_LEFT<<"\n";
    
    // joints_[0]->SetParam("vel", 0, 10.0);
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&F_Wheel_Drive::OnUpdate, this));
  }

  // Called by the world update start event
  void F_Wheel_Drive::OnUpdate()
  {
    //printf("Updating!\n");
    // Apply a small linear velocity to the model.
    F_Wheel_Drive::updateVelocity();
  }

  void F_Wheel_Drive::updateVelocity(){
    if(x_ > 0) {
      joints_[FORWARD_LEFT]->SetParam("vel", 0, 10.0);
      joints_[FORWARD_RIGHT]->SetParam("vel", 0, 10.0);
      joints_[BACKWARD_LEFT]->SetParam("vel", 0, 10.0);
      joints_[BACKWARD_RIGHT]->SetParam("vel", 0, 10.0);
      //this->model->SetLinearVel(ignition::math::Vector3d(0, 0.8, 0));
    }
    else if(x_ < 0) {
      joints_[FORWARD_LEFT]->SetParam("vel", 0, -10.0);
      joints_[FORWARD_RIGHT]->SetParam("vel", 0, -10.0);
      joints_[BACKWARD_LEFT]->SetParam("vel", 0, -10.0);
      joints_[BACKWARD_RIGHT]->SetParam("vel", 0, -10.0);
      //this->model->SetLinearVel(ignition::math::Vector3d(0, -0.8, 0));
    }
    else if(rot_ > 0) {
      joints_[FORWARD_LEFT]->SetParam("vel", 0, -10.0);
      joints_[FORWARD_RIGHT]->SetParam("vel", 0, 10.0);
      joints_[BACKWARD_LEFT]->SetParam("vel", 0, -10.0);
      joints_[BACKWARD_RIGHT]->SetParam("vel", 0, 10.0);
      //this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 8));
      //this->model->GetJoint("link_0_JOINT_1")->SetVelocity(0, 1.0);
      //this->model->GetJoint("link_0_JOINT_3")->SetVelocity(0, 1.0);
      //this->model->SetLinearVel(ignition::math::Vector3d(0.8, 0, 0));
      //if(p)
        //p->GetJoint("car_wheel_4_joint")->SetParam("vel", 0, 0.8);
      //this->model->GetJoint("F_Wheel_Drive::car_wheel_1::car_wheel_4_joint")->SetParam("vel", 0, 0.8);
    } else if(rot_ <0) {
      joints_[FORWARD_LEFT]->SetParam("vel", 0, 10.0);
      joints_[FORWARD_RIGHT]->SetParam("vel", 0, -10.0);
      joints_[BACKWARD_LEFT]->SetParam("vel", 0, 10.0);
      joints_[BACKWARD_RIGHT]->SetParam("vel", 0, -10.0);
      //this->model->SetAngularVel(ignition::math::Vector3d(0, 0, -8));
      //this->model->SetLinearVel(ignition::math::Vector3d(-0.8, 0, 0));
    }
    else {
      joints_[FORWARD_LEFT]->SetParam("vel", 0, 0.0);
      joints_[FORWARD_RIGHT]->SetParam("vel", 0, 0.0);
      joints_[BACKWARD_LEFT]->SetParam("vel", 0, 0.0);
      joints_[BACKWARD_RIGHT]->SetParam("vel", 0, 0.0);
    }
    this->model->Update();
  }

  void F_Wheel_Drive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(F_Wheel_Drive);
}
