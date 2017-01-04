// Written By : Yossi Cohen
#define MY_GAZEBO_VER 5
// If the plugin is not defined then define it
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <random>
// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/gazebo_config.h>
// ROS Communication
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
// Boost Thread Mutex
#include <boost/thread/mutex.hpp>
// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <hmmwv/hmmwvConfig.h>
#include <boost/bind.hpp> // Boost Bind

// Maximum time delay before a "no command" behaviour is initiated.
#define command_MAX_DELAY 0.3

#define PI 3.14159265359

//#define MY_GAZEBO_VER 5

namespace gazebo
{

class hmmwvDrivingPlugin : public ModelPlugin
{
  double deltaSimTime = 0.001;
  ///  Constructor
public:
  hmmwvDrivingPlugin() {}

  /// The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
  {
    std::cout << "My major GAZEBO VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;
    this->model = _model;
    // Store the pointers to the joints
    this->left_wheel_1 = this->model->GetJoint("left_wheel_1");
    this->left_wheel_2 = this->model->GetJoint("left_wheel_2");
    this->right_wheel_1 = this->model->GetJoint("right_wheel_1");
    this->right_wheel_2 = this->model->GetJoint("right_wheel_2");
    this->streer_joint_left_1 = this->model->GetJoint("steering_joint_left_1");
    this->streer_joint_right_1 = this->model->GetJoint("steering_joint_right_1");
    this->spring_left_1 = this->model->GetJoint("spring_left_1");
    this->spring_right_1 = this->model->GetJoint("spring_right_1");
    this->spring_left_2 = this->model->GetJoint("spring_left_2");
    this->spring_right_2 = this->model->GetJoint("spring_right_2");

    // Starting Timers
    Throttle_command_timer.Start();
    Angular_command_timer.Start();
    Breaking_command_timer.Start();
    this->Ros_nh = new ros::NodeHandle("hmmwvDrivingPlugin_node");

    // Subscribe to the topic, and register a callback
    Steering_rate_sub = this->Ros_nh->subscribe("/hmmwv/Driving/Steering", 100, &hmmwvDrivingPlugin::On_Steering_command, this);
    Velocity_rate_sub = this->Ros_nh->subscribe("/hmmwv/Driving/Tourque", 100, &hmmwvDrivingPlugin::On_Throttle_command, this);
    Breaking_sub = this->Ros_nh->subscribe("/hmmwv/Driving/Break", 100, &hmmwvDrivingPlugin::On_Break_command, this);

    platform_hb_pub_ = this->Ros_nh->advertise<std_msgs::Bool>("/Sahar/link_with_platform", 100);

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&hmmwvDrivingPlugin::OnUpdate, this, _1));
    std::cout << "Setting up dynamic config" << std::endl;

    this->model_reconfiguration_server = new dynamic_reconfigure::Server<hmmwv::hmmwvConfig>(*(this->Ros_nh));
    this->model_reconfiguration_server->setCallback(boost::bind(&hmmwvDrivingPlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    std::cout << "dynamic configuration is set up" << std::endl;
    Steering_Request = 0;
    Throttle_command = 0;
    std::cout << "Driving Plugin Loaded" << std::endl;
  }

public:
  void dynamic_Reconfiguration_callback(hmmwv::hmmwvConfig &config, uint32_t level)
  {
    control_P = config.Steer_control_P;
    control_I = config.Steer_control_I;
    control_D = config.Steer_control_D;
    Steering_Speed = config.Steering;
    damping = config.Damping;
    power = config.Power;
    suspenSpring = config.Spring;
    SuspenDamp = config.Damper;
    SuspenTarget = config.Target;
  }
  float tempTime = 0;
  // Called by the world update start event, This function is the event that will be called every update
public:
  void OnUpdate(const common::UpdateInfo &_info) // we are not using the pointer to the info so its commanted as an option
  {

    deltaSimTime = _info.simTime.Double() - sim_Time.Double();
    sim_Time = _info.simTime;
    // std::cout << "update function started"<<std::endl;
    // std::cout << "command_timer = " << command_timer.GetElapsed().Float() << std::endl;
    // Applying effort to the wheels , brakes if no message income
    if (Throttle_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Applies 0 throtle 
      Throttle_command = 0;
    }
    if (Breaking_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Brakes
      Break = 1;
    }
    if (Angular_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Restores straight direction.
      Steering_Request = 0;
    }
    // std::cout << "Applying efforts"<<std::endl;
    apply_efforts();
    ApplySuspension();
    breaker(Break);

    std::cout << this->spring_right_1->GetAngle(0).Radian() << std::endl;
    std_msgs::Bool connection;
    connection.data = true;
    platform_hb_pub_.publish(connection);
  }
  double WheelPower = 0;
  double DesiredAngle = 0;
  void wheel_controller(physics::JointPtr wheel_joint, double Tourque)
  {
    WheelPower = WheelPower + 0.001 * (Tourque - WheelPower);

    double wheel_omega = wheel_joint->GetVelocity(0);
    double effort_command = WheelPower - damping * wheel_omega;

    wheel_joint->SetForce(0, effort_command);
  }
  void steer_controller(physics::JointPtr steer_joint, double Angle)
  {
    // std::cout << " getting angle"<< std::endl;
    if (steer_joint)
    {
      double RefreshRate = 1000;
      DesiredAngle = DesiredAngle + Steering_Speed * deltaSimTime * (Angle - DesiredAngle);
      double wheel_angle = steer_joint->GetAngle(0).Radian();
      double steer_omega = steer_joint->GetVelocity(0);
      double effort_command = control_P * (0.6 * DesiredAngle - wheel_angle) - control_D * (steer_omega);
      steer_joint->SetForce(0, effort_command);
      //  std::cout << wheel_angle<< std::endl;
    }
    else
      std::cout << "Null Exception! \n";
    // std::cout << "efforting"<< std::endl;
    // this->jointController->SetJointPosition(steer_joint, Angle*0.61);
  }

  void apply_efforts()
  {
    float WheelTorque = Throttle_command * power;
    // std::cout << " Controlling wheels"<< std::endl;
    wheel_controller(this->left_wheel_1, WheelTorque);
    wheel_controller(this->left_wheel_2, WheelTorque);
    wheel_controller(this->right_wheel_1, WheelTorque);
    wheel_controller(this->right_wheel_2, WheelTorque);
    // std::cout << " Controlling Steering"<< std::endl;
    steer_controller(this->streer_joint_left_1, Steering_Request);
    steer_controller(this->streer_joint_right_1, Steering_Request);
    // std::cout << " Finished applying efforts"<< std::endl;
  }
  void breaker(int breaking)
  {

    // std::cout << " getting angle"<< std::endl;
    if (breaking >= 0.09 && !Breaks)
    {
      TempDamping = damping;
      damping = 10000 * Break * Break;
      Breaks = true;
      // std::cout << "Break on "<<damping<<std::endl;
    }
    else if (breaking == 0 && Breaks)
    {
      damping = TempDamping;
      Breaks = false;
      // std::cout << "Breaks off "<<damping<<std::endl;
    }
    if (breaking >= 0.09 && Breaks)
      damping = 10000 * Break * Break;

    // std::cout << "efforting"<< std::endl;
    // this->jointController->SetJointPosition(steer_joint, Angle*0.61);
  }

  void On_Break_command(const std_msgs::Float64ConstPtr &msg)
  {
    Breaking_command_mutex.lock();
    // Recieving referance velocity
    if (msg->data >= 1)
      Break = 1;
    else if (msg->data >= 0.09)
      Break = msg->data;
    else
      Break = 0;

// Reseting timer every message
#if GAZEBO_MAJOR_VERSION >= 5
    Breaking_command_timer.Reset();
#endif
    Breaking_command_timer.Start();
    Breaking_command_mutex.unlock();
  }
  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Steering_command(const std_msgs::Float64ConstPtr &msg)
  {
    Angular_command_mutex.lock();
    // Recieving referance steering angle
    if (msg->data > 1)
    {
      Steering_Request = 1;
    }
    else if (msg->data < -1)
    {
      Steering_Request = -1;
    }
    else
    {
      Steering_Request = msg->data;
    }

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
    Angular_command_timer.Reset();
#endif
    Angular_command_timer.Start();
    Angular_command_mutex.unlock();
  }

  void ApplySuspension()
  {
    Suspension(spring_left_1);
    Suspension(spring_left_2);
    Suspension(spring_right_1);
    Suspension(spring_right_2);
  }
  void Suspension(physics::JointPtr Suspension)
  {//The function to control the suspension of the Vehicle
    double Force = -(Suspension->GetAngle(0).Radian() + SuspenTarget) * suspenSpring - Suspension->GetVelocity(0) * SuspenDamp;
    Suspension->SetForce(0, Force);
  }

  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Throttle_command(const std_msgs::Float64ConstPtr &msg)
  {

    Throttle_command_mutex.lock();
    // Recieving referance velocity
    if (msg->data > 1)
    {
      Throttle_command = 1;
    }
    else if (msg->data < -1)
    {
      Throttle_command = -1;
    }
    else
    {
      Throttle_command = msg->data;
    }

// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
    Throttle_command_timer.Reset();
#endif
    Throttle_command_timer.Start();

    Throttle_command_mutex.unlock();
  }

  float gazebo_ver;

  // Defining private Pointer to model
  physics::ModelPtr model;

  // Defining private Pointer to joints

  physics::JointPtr right_wheel_1;
  physics::JointPtr right_wheel_2;
  physics::JointPtr left_wheel_1;
  physics::JointPtr left_wheel_2;
  physics::JointPtr streer_joint_left_1;
  physics::JointPtr streer_joint_right_1;
  physics::JointPtr spring_left_1;
  physics::JointPtr spring_right_1;
  physics::JointPtr spring_left_2;
  physics::JointPtr spring_right_2;

  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;

  // Defining private Ros Subscribers
  ros::Subscriber Steering_rate_sub;
  ros::Subscriber Velocity_rate_sub;
  ros::Subscriber Breaking_sub;
  // Defining private Ros Publishers
  ros::Publisher platform_hb_pub_;

  // Defining private Timers
  common::Timer Throttle_command_timer;
  common::Timer Angular_command_timer;
  common::Timer Breaking_command_timer;
  common::Time sim_Time;

  // Defining private Mutex
  boost::mutex Angular_command_mutex;
  boost::mutex Throttle_command_mutex;
  boost::mutex Breaking_command_mutex;
  float Throttle_command;
  float Steering_Request;
  int Break = 1;
  double Linear_ref_vel;
  double Angular_ref_vel;

  bool Breaks = false;
//Dynamic Configuration Definitions
  dynamic_reconfigure::Server<hmmwv::hmmwvConfig> *model_reconfiguration_server;
  double control_P, control_I, control_D, Steering_Speed,
  damping, power, TempDamping, suspenSpring, SuspenDamp, SuspenTarget;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(hmmwvDrivingPlugin)
}
