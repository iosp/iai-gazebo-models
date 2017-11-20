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
#include <oshkosh_model/oshkosh_wheels_steerConfig.h>
// Boost Bind
#include <boost/bind.hpp> 


// Maximum time delay before a "no command" behaviour is initiated.
#define command_MAX_DELAY 0.3

#define PI 3.14159265359


namespace gazebo
{

class oshkoshSteerPlugin : public ModelPlugin
{
  double deltaSimTime = 0.001;
  ///  Constructor
public:
  oshkoshSteerPlugin() {}

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

    this->VehicleLength = 1;       
    if (_sdf->HasElement("VehicleLength")) 
       this->VehicleLength = _sdf->Get<double>("VehicleLength");   
    else
       ROS_WARN("canot fined VehicleLength");

    this->VehicleWidth = 1;       
    if (_sdf->HasElement("shaft_name")) 
       this->VehicleWidth = _sdf->Get<double>("VehicleWidth");   
    else
       ROS_WARN("canot fined VehicleWidth"); 

      
    std::string pref = "";
    // Test for whether the model is nested and a prefix is needed to find the joints
    if( this->model->GetLink("body") )
          pref = this->vehicle_name + "::" +this->shaft_name;
    else
          pref = this->vehicle_name + "::oshkosh::" + this->shaft_name; 

    std::cout << " Loading " << pref  << " Wheel Steer Plugin " << std::endl;
      

    if( ! this->model->GetJoint(pref+"::steering_left_wheel_joint") )
      std::cout << " canot fined " << pref+"::steering_left_wheel_joint" << std::endl;

    if( ! this->model->GetJoint(pref+"::steering_right_wheel_joint") )
      std::cout << " canot fined " << pref+"::steering_right_wheel_joint" << std::endl;


    // Store the pointers to the joints
    this->left_steer_joint = this->model->GetJoint(pref+"::steering_left_wheel_joint");
    this->right_steer_joint = this->model->GetJoint(pref+"::steering_right_wheel_joint");

    // Starting Timers
    Angular_command_timer.Start();

    this->Ros_nh = new ros::NodeHandle(this->vehicle_name + "_" + this->shaft_name + "WheelSteerPlugin_PluginNode");

    // Subscribe to the topic, and register a callback
    Steering_sub = this->Ros_nh->subscribe("/"+vehicle_name+"/Driving/Steering", 60, &oshkoshSteerPlugin::On_Angular_command, this);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&oshkoshSteerPlugin::OnUpdate, this, _1));

    //std::cout << "Setting up dynamic config" << std::endl;
    this->model_reconfiguration_server = new dynamic_reconfigure::Server<oshkosh_model::oshkosh_wheels_steerConfig>(*(this->Ros_nh));
    this->model_reconfiguration_server->setCallback(boost::bind(&oshkoshSteerPlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    //std::cout << "dynamic configuration is set up" << std::endl;

    Steering_Request = 0;

    std::cout << " Loaded " << pref  << " Wheel Steer Plugin " << std::endl;
  }

public:
  void dynamic_Reconfiguration_callback(oshkosh_model::oshkosh_wheels_steerConfig &config, uint32_t level)
  {
    SteerPower = config.WheelSteerPower;
    SteerDamping = config.WheelSteerDamping;
  }

  // Called by the world update start event, This function is the event that will be called every update
public:
  void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
  {
    deltaSimTime = simInfo.simTime.Double() - sim_Time.Double();
    sim_Time = simInfo.simTime;
    // std::cout << "update function started"<<std::endl;
    // std::cout << "command_timer = " << command_timer.GetElapsed().Float() << std::endl;

    // Applying effort to the wheels , brakes if no message income

    if (Angular_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Brakes
      Steering_Request = 0;
    }

    apply_steering();
  }

    void apply_steering()
  {
    double Len = this->VehicleLength;
    double Wid = this->VehicleWidth;

    double left_steer = 0;
    double right_steer = 0;

    if (Steering_Request != 0)
    {
      if ( Steering_Request > 0 )  // turning left - left wheel is the iner one
        {
         double R = Len/atan(Steering_Request);
         left_steer =  atan(Len/(R-Wid/2)); 
         right_steer = atan(Len/(R+Wid/2)); 
        }
      else // turning right - right wheel is the iner one
        {
         double R = Len/atan(-Steering_Request);
         left_steer =  -atan(Len/(R+Wid/2)); 
         right_steer = -atan(Len/(R-Wid/2)); 
        }
    }

    steer_controller(this->left_steer_joint,  left_steer);
    steer_controller(this->right_steer_joint, right_steer);
  
  }
  
  
  void steer_controller(physics::JointPtr steer_joint, double target_steer)
  {
      ignition::math::Angle AnglePose =  steer_joint->Position(0);  
      double curent_steer = AnglePose.Radian();
      double steer_omega = steer_joint->GetVelocity(0);

      double effort_command = SteerPower * (target_steer - curent_steer) - SteerDamping * steer_omega ;

      steer_joint->SetForce(0, effort_command);
  }

  
  
  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Angular_command(const std_msgs::Float64ConstPtr &msg)
  {
    Angular_command_mutex.lock();
    // Recieving referance steering angle
    if (msg->data > PI/4)
      Steering_Request = PI/4;
    else if (msg->data < -PI/4)
      Steering_Request = -PI/4;
    else
      Steering_Request = msg->data;

    // Reseting timer every time LLC publishes message
    Angular_command_timer.Reset();
    Angular_command_timer.Start();

    Angular_command_mutex.unlock();
  }


  // Defining private Pointer to model
  physics::ModelPtr model;

  std::string vehicle_name;
  std::string shaft_name;
  double VehicleLength;
  double VehicleWidth;

  // Defining private Pointer to joints
  physics::JointPtr left_steer_joint;
  physics::JointPtr right_steer_joint;


  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;

  // Defining private Ros Subscribers
  ros::Subscriber Steering_sub;


  // Defining private Timers
  common::Timer Angular_command_timer;
  common::Time sim_Time;

  // Defining private Mutex
  boost::mutex Angular_command_mutex;
  //helper vars
  float Steering_Request;
  double Angular_ref_vel;
  double DesiredAngle = 0;
  double DesiredAngleR = 0;

  //Dynamic Configuration Definitions
  dynamic_reconfigure::Server<oshkosh_model::oshkosh_wheels_steerConfig> *model_reconfiguration_server;
    double SteerPower, SteerDamping;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(oshkoshSteerPlugin)
}
