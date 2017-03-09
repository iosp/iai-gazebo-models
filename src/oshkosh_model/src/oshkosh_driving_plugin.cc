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
#include <oshkosh_model/oshkosh_modelConfig.h>
#include <boost/bind.hpp> // Boost Bind

#include <tf/transform_broadcaster.h>

// Maximum time delay before a "no command" behaviour is initiated.
#define command_MAX_DELAY 0.3
#define PI 3.14159265359
#define VehicleLength 6
#define VehicleWidth 2
#define WheelRadius 0.71
//#define MY_GAZEBO_VER 5

namespace gazebo
{

class oshkoshDrivingPlugin : public ModelPlugin
{
  double deltaSimTime = 0.001;
  ///  Constructor
public:
  oshkoshDrivingPlugin() {}

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
    this->left_wheel_3 = this->model->GetJoint("left_wheel_3");
    this->left_wheel_4 = this->model->GetJoint("left_wheel_4");
    this->right_wheel_1 = this->model->GetJoint("right_wheel_1");
    this->right_wheel_2 = this->model->GetJoint("right_wheel_2");
    this->right_wheel_3 = this->model->GetJoint("right_wheel_3");
    this->right_wheel_4 = this->model->GetJoint("right_wheel_4");
    this->streer_joint_left_1 = this->model->GetJoint("steering_joint_left_1");
    this->streer_joint_left_2 = this->model->GetJoint("steering_joint_left_2");
    this->streer_joint_right_1 = this->model->GetJoint("steering_joint_right_1");
    this->streer_joint_right_2 = this->model->GetJoint("steering_joint_right_2");
    this->spring_left_1 = this->model->GetJoint("spring_left_1");
    this->spring_right_1 = this->model->GetJoint("spring_right_1");
    this->spring_left_2 = this->model->GetJoint("spring_left_2");
    this->spring_right_2 = this->model->GetJoint("spring_right_2");
    this->spring_left_3 = this->model->GetJoint("spring_left_3");
    this->spring_right_3 = this->model->GetJoint("spring_right_3");
    this->spring_left_4 = this->model->GetJoint("spring_left_4");
    this->spring_right_4 = this->model->GetJoint("spring_right_4");
    // Starting Timers
    Throttle_command_timer.Start();
    Angular_command_timer.Start();
    Breaking_command_timer.Start();
    this->Ros_nh = new ros::NodeHandle("oshkoshDrivingPlugin_node");

    // Subscribe to the topic, and register a callback
    Steering_rate_sub = this->Ros_nh->subscribe("/Oshkosh/Driving/Steering", 60, &oshkoshDrivingPlugin::On_Angular_command, this);
    Velocity_rate_sub = this->Ros_nh->subscribe("/Oshkosh/Driving/Throttle", 60, &oshkoshDrivingPlugin::On_Throttle_command, this);
    BreakPedal_sub = this->Ros_nh->subscribe("/Oshkosh/Driving/Break", 60, &oshkoshDrivingPlugin::On_Break_command, this);

    platform_hb_pub_ = this->Ros_nh->advertise<std_msgs::Bool>("/Oshkosh/link_with_platform", 60);
    platform_Speedometer_pub = this->Ros_nh->advertise<std_msgs::Float64>("/Oshkosh/Speedometer", 60);
    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&oshkoshDrivingPlugin::OnUpdate, this, _1));
    std::cout << "Setting up dynamic config" << std::endl;

    this->model_reconfiguration_server = new dynamic_reconfigure::Server<oshkosh_model::oshkosh_modelConfig>(*(this->Ros_nh));
    this->model_reconfiguration_server->setCallback(boost::bind(&oshkoshDrivingPlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    std::cout << "dynamic configuration is set up" << std::endl;

    Steering_Request = 0;
    Throttle_command = 0;
    std::cout << "Driving Plugin Loaded" << std::endl;
  }

public:
  void dynamic_Reconfiguration_callback(oshkosh_model::oshkosh_modelConfig &config, uint32_t level)
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

  // Called by the world update start event, This function is the event that will be called every update
public:
  void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
  {
    deltaSimTime = simInfo.simTime.Double() - sim_Time.Double();
    sim_Time = simInfo.simTime;
    // std::cout << "update function started"<<std::endl;
    // std::cout << "command_timer = " << command_timer.GetElapsed().Float() << std::endl;

    // Applying effort to the wheels , brakes if no message income
    if (Throttle_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Brakes
      Throttle_command = 0;
    }
    if (Breaking_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Brakes
      BreakPedal = 1;
    }
    if (Angular_command_timer.GetElapsed().Float() > command_MAX_DELAY)
    {
      // Brakes
      Steering_Request = 0;
    }
    // std::cout << "Applying efforts"<<std::endl;

    apply_efforts();
    apply_steering();
    ApplySuspension();
    breaker();

    // std::cout << "Applied"<<std::endl;
    SpeedMsg.data = Speed;
    platform_Speedometer_pub.publish(SpeedMsg);
    std_msgs::Bool connection;
    connection.data = true;
    platform_hb_pub_.publish(connection);
    tf::Transform transform;
	  transform.setOrigin( tf::Vector3(model->GetWorldPose().pos.x, model->GetWorldPose().pos.y, model->GetWorldPose().pos.z) );
	  transform.setRotation(tf::Quaternion(model->GetWorldPose().rot.x,model->GetWorldPose().rot.y,model->GetWorldPose().rot.z,model->GetWorldPose().rot.w));

	  TF_Broadcast(transform, "WORLD", model->GetName(), simInfo.simTime);

  }
  	void TF_Broadcast(tf::Transform transform, std::string frame_id, std::string child_frame_id, common::Time time)
	{
		 static tf::TransformBroadcaster br;
		 tf::StampedTransform st(transform, ros::Time::now()/*(time.sec, time.nsec)*/, frame_id, child_frame_id);
		 br.sendTransform(st);
	}
  void apply_efforts()
  {
    float WheelTorque = power * Throttle_command;
    // std::cout << " Controlling wheels"<< std::endl;
    wheel_controller(this->left_wheel_1 , WheelTorque);
    wheel_controller(this->left_wheel_2 , WheelTorque);
    wheel_controller(this->left_wheel_3, WheelTorque);
    wheel_controller(this->left_wheel_4, WheelTorque);
    wheel_controller(this->right_wheel_1, WheelTorque);
    wheel_controller(this->right_wheel_2, WheelTorque);
    wheel_controller(this->right_wheel_3, WheelTorque);
    wheel_controller(this->right_wheel_4, WheelTorque);
    // std::cout << " Finished applying efforts"<< std::endl;
  }
    void apply_steering()
  {
    double ThetaAckerman = 0;
    double ThetaOuter = 0;
    if (Steering_Request > 0)
    {

      ThetaAckerman = atan(1 / ((1 / (tan(Steering_Request)) + (VehicleWidth / VehicleLength))));
      steer_controller(this->streer_joint_left_1, Steering_Request);
      steer_controller(this->streer_joint_right_1, ThetaAckerman);
      steer_controller(this->streer_joint_left_2, Steering_Request);
      steer_controller(this->streer_joint_right_2, ThetaAckerman);
    }
    else if (Steering_Request < 0)
    {
      ThetaAckerman = atan(1 / ((1 / (tan(-Steering_Request)) + (VehicleWidth / VehicleLength))));
      steer_controller(this->streer_joint_left_1, -ThetaAckerman);
      steer_controller(this->streer_joint_right_1, Steering_Request);
      steer_controller(this->streer_joint_left_2, -ThetaAckerman);
      steer_controller(this->streer_joint_right_2, Steering_Request);
    }
    else
    {
      steer_controller(this->streer_joint_left_1, 0);
      steer_controller(this->streer_joint_right_1, 0);
      steer_controller(this->streer_joint_left_2, 0);
      steer_controller(this->streer_joint_right_2, 0);
    }

    // std::cout << ThetaAckerman << std::endl;
  }
  void wheel_controller(physics::JointPtr wheel_joint, double Tourque)
  {
    WheelPower = WheelPower + 0.002 * (Tourque - WheelPower);

    double wheel_omega = wheel_joint->GetVelocity(0);
    double Joint_Force = WheelPower - damping * wheel_omega;

    wheel_joint->SetForce(0, Joint_Force);
    if (wheel_joint == right_wheel_4)
    {
      wheelsSpeedSum = wheelsSpeedSum + wheel_omega;
      Speed = wheelsSpeedSum * WheelRadius / 8;
      wheelsSpeedSum = 0;
    }
    else
      wheelsSpeedSum = wheelsSpeedSum + wheel_omega;
  }
  void steer_controller(physics::JointPtr steer_joint, double Angle)
  {
    // std::cout << " getting angle"<< std::endl;
    if (steer_joint)
    {
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
  void ApplySuspension()
  {
    Suspension(spring_left_1);
    Suspension(spring_left_2);
    Suspension(spring_right_1);
    Suspension(spring_right_2);
    Suspension(spring_left_3);
    Suspension(spring_left_4);
    Suspension(spring_right_3);
    Suspension(spring_right_4);
  }
  void Suspension(physics::JointPtr Suspension)
  { //The function to control the suspension of the Vehicle
    double SpringForce = -(Suspension->GetAngle(0).Radian() + SuspenTarget) * suspenSpring - Suspension->GetVelocity(0) * SuspenDamp;
    Suspension->SetForce(0, SpringForce);
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
  // The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
  void On_Angular_command(const std_msgs::Float64ConstPtr &msg)
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
    Steering_Request = -Steering_Request;
// Reseting timer every time LLC publishes message
#if GAZEBO_MAJOR_VERSION >= 5
    Angular_command_timer.Reset();
#endif
    Angular_command_timer.Start();

    Angular_command_mutex.unlock();
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

// Reseting timer every message
#if GAZEBO_MAJOR_VERSION >= 5
    Breaking_command_timer.Reset();
#endif
    Breaking_command_timer.Start();
    Breaking_command_mutex.unlock();
  }
  float gazebo_ver;

  // Defining private Pointer to model
  physics::ModelPtr model;

  // Defining private Pointer to joints

  physics::JointPtr right_wheel_1;
  physics::JointPtr right_wheel_2;
  physics::JointPtr right_wheel_3;
  physics::JointPtr right_wheel_4;
  physics::JointPtr left_wheel_1;
  physics::JointPtr left_wheel_2;
  physics::JointPtr left_wheel_3;
  physics::JointPtr left_wheel_4;
  physics::JointPtr streer_joint_left_1;
  physics::JointPtr streer_joint_left_2;
  physics::JointPtr streer_joint_right_1;
  physics::JointPtr streer_joint_right_2;
  physics::JointPtr spring_left_1;
  physics::JointPtr spring_right_1;
  physics::JointPtr spring_left_2;
  physics::JointPtr spring_right_2;
  physics::JointPtr spring_left_3;
  physics::JointPtr spring_right_3;
  physics::JointPtr spring_left_4;
  physics::JointPtr spring_right_4;
  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;

  // Defining private Ros Subscribers
  ros::Subscriber Steering_rate_sub;
  ros::Subscriber Velocity_rate_sub;
  ros::Subscriber BreakPedal_sub;
  // Defining private Ros Publishers
  ros::Publisher platform_hb_pub_;
  ros::Publisher platform_Speedometer_pub;
  std_msgs::Bool connection;
  std_msgs::Float64 SpeedMsg;

  // Defining private Timers
  common::Timer Throttle_command_timer;
  common::Timer Angular_command_timer;
  common::Timer Breaking_command_timer;
  common::Time sim_Time;

  // Defining private Mutex
  boost::mutex Angular_command_mutex;
  boost::mutex Throttle_command_mutex;
  boost::mutex Breaking_command_mutex;
  //helper vars
  float Throttle_command;
  float Steering_Request;
  float BreakPedal = 1;
  double Linear_ref_vel;
  double Angular_ref_vel;
  double WheelPower = 0;
  double DesiredAngle = 0;
  double DesiredAngleR = 0;
  double wheelsSpeedSum = 0;
  float tempTime = 0;
  float Speed = 0;
  bool Breaks = false;
  //Dynamic Configuration Definitions
  dynamic_reconfigure::Server<oshkosh_model::oshkosh_modelConfig> *model_reconfiguration_server;
    double control_P, control_I, control_D, Steering_Speed,
      damping, power, TempDamping, suspenSpring, SuspenDamp, SuspenTarget;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(oshkoshDrivingPlugin)
}
