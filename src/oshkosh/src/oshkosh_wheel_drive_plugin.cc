// Written By : Daniel
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// ROS Communication
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
// Boost Thread Mutex
#include <boost/thread/mutex.hpp>
// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <oshkosh/oshkosh_wheels_driveConfig.h>
// Boost Bind
#include <boost/bind.hpp> 

// Maximum time delay before a "no command" behaviour is initiated.
#define command_MAX_DELAY 0.3


namespace gazebo
{

class oshkoshWheelDrivePlugin : public ModelPlugin
{
  ///  Constructor
public:
  oshkoshWheelDrivePlugin() {}

  /// The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
  {
    std::cout << "My major GAZEBO VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;
    this->model = _model;
    this->vehicle_name = model->GetName(); 

    
    this->shaft_name = "";       
    if (_sdf->HasElement("shaft_name")) 
       this->shaft_name = _sdf->Get<std::string>("shaft_name");   
    else
       ROS_WARN("canot fined shaft_name");


    std::string pref = "";
    // Test for whether the model is nested and a prefix is needed to find the joints
    if( this->model->GetLink("body") )
          pref = this->vehicle_name + "::" +this->shaft_name;
    else
          pref = this->vehicle_name + "::oshkosh::" + this->shaft_name; 


    std::cout << " Loading " << pref  << " Wheel Drive Plugin " << std::endl;
      

    if( ! this->model->GetJoint(pref+"::left_wheel_joint") )
      std::cout << " canot fined " << pref+"::left_wheel_joint" << std::endl;

    if( ! this->model->GetJoint(pref+"::right_wheel_joint") )
      std::cout << " canot fined " << pref+"::right_wheel_joint" << std::endl;


    // Store the pointers to the joints
    this->left_wheel_joint = this->model->GetJoint(pref+"::left_wheel_joint");
    this->right_wheel_joint = this->model->GetJoint(pref+"::right_wheel_joint");

    // Starting Timers
    Throttle_command_timer.Start();

    this->Ros_nh = new ros::NodeHandle(this->vehicle_name + "_" + this->shaft_name + "WheelDrivePlugin_PluginNode");

    // Subscribe to the topic, and register a callback
    Throttle_sub = this->Ros_nh->subscribe("/"+vehicle_name+"/Driving/Throttle", 60, &oshkoshWheelDrivePlugin::On_Throttle_command, this);
    BreakPedal_sub = this->Ros_nh->subscribe("/"+vehicle_name+"/Driving/Break", 60, &oshkoshWheelDrivePlugin::On_Break_command, this);

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&oshkoshWheelDrivePlugin::OnUpdate, this, _1));
    
    //std::cout << "Setting up dynamic config" << std::endl;
    this->model_reconfiguration_server = new dynamic_reconfigure::Server<oshkosh::oshkosh_wheels_driveConfig>(*(this->Ros_nh));
    this->model_reconfiguration_server->setCallback(boost::bind(&oshkoshWheelDrivePlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    //std::cout << "dynamic configuration is set up" << std::endl;

    Throttle_command = 0;
    std::cout << " Loaded " << pref  << " Wheel Drive Plugin " << std::endl;
  }

public:
  void dynamic_Reconfiguration_callback(oshkosh::oshkosh_wheels_driveConfig &config, uint32_t level)
  {
    power = config.WheelDrivePower;
    damping = config.WheelDriveDamping;
  }

  // Called by the world update start event, This function is the event that will be called every update
public:
  void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
  {
    // Applying effort to the wheels , brakes if no message income
    if (Throttle_command_timer.GetElapsed().Float() > command_MAX_DELAY)
      Throttle_command = 0;// Brakes
    
    if (Breaking_command_timer.GetElapsed().Float() > command_MAX_DELAY)// Brakes
      BreakPedal = 1; // Brakes


    // std::cout << "Applying efforts"<<std::endl;
    apply_efforts();
    breaker();
  }

 
  void apply_efforts()
  {
    float WheelTorque = power * Throttle_command;
    // std::cout << " Controlling wheels"<< std::endl;

    wheel_controller(this->left_wheel_joint , WheelTorque);
    wheel_controller(this->right_wheel_joint , WheelTorque);

    // std::cout << " Finished applying efforts"<< std::endl;
  }

  void wheel_controller(physics::JointPtr wheel_joint, double Tourque)
  {
    WheelPower = WheelPower + 0.002 * (Tourque - WheelPower);

    double wheel_omega = wheel_joint->GetVelocity(0);
    double Joint_Force = WheelPower - damping * wheel_omega;

    wheel_joint->SetForce(0, Joint_Force);
  }
  

  void breaker()
  {
    // std::cout << " getting angle"<< std::endl;
    if (BreakPedal >= 0.09 && !Breaks)
    {
      TempDamping = damping;
      damping = 10000 * BreakPedal * BreakPedal;
      Breaks = true;
      std::cout << "Break on " << damping << std::endl;
    }
    else if (BreakPedal == 0 && Breaks)
    {
      damping = TempDamping;
      Breaks = false;
      std::cout << "Breaks off " << damping << std::endl;
    }
    if (BreakPedal >= 0.09 && Breaks)
      damping = 10000 * BreakPedal * BreakPedal;

    // std::cout << "efforting"<< std::endl;
    // this->jointController->SetJointPosition(steer_joint, Angle*0.61);
  }
  

  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Throttle_command(const std_msgs::Float64ConstPtr &msg)
  {
    Throttle_command_mutex.lock();
    // Recieving referance velocity
    if (msg->data > 1)
      Throttle_command = 1;
    else if (msg->data < -1)
      Throttle_command = -1;
    else
      Throttle_command = msg->data;
    

// Reseting timer every time LLC publishes message
    Throttle_command_timer.Reset();
    Throttle_command_timer.Start();

    Throttle_command_mutex.unlock();
  }
  

  void On_Break_command(const std_msgs::Float64ConstPtr &msg)
  {
    Breaking_command_mutex.lock();
    // Recieving referance velocity
    if (msg->data >= 1)
      BreakPedal = 1;
    else if (msg->data >= 0.09)
      BreakPedal = msg->data;
    else
      BreakPedal = 0;

    Breaking_command_timer.Reset();
    Breaking_command_timer.Start();
    Breaking_command_mutex.unlock();
  }



  // Defining private Pointer to model
  physics::ModelPtr model;

  std::string vehicle_name;
  std::string shaft_name;

  // Defining private Pointer to joints
  physics::JointPtr left_wheel_joint;
  physics::JointPtr right_wheel_joint;

  
  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;

  // Defining private Ros Subscribers
  ros::Subscriber Throttle_sub;
  ros::Subscriber BreakPedal_sub;


  // Defining private Timers
  common::Timer Throttle_command_timer;
  common::Timer Breaking_command_timer;
  common::Time sim_Time;

  // Defining private Mutex
  boost::mutex Throttle_command_mutex;
  boost::mutex Breaking_command_mutex;
  //helper vars
  float Throttle_command;
  float BreakPedal = 1;
  double WheelPower = 0;
  bool Breaks = false;

  //Dynamic Configuration Definitions
  dynamic_reconfigure::Server<oshkosh::oshkosh_wheels_driveConfig> *model_reconfiguration_server;
    double damping = 100, power = 2200, TempDamping;



};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(oshkoshWheelDrivePlugin)
}
