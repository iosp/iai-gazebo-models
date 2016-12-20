// Written By : Sagi Vald

// If the plugin is not defined then define it
#ifndef _HAMMVEE_DRIVE_PLUGIN_HH_
#define _HAMMVEE_DRIVE_PLUGIN_HH_

// Including Used Libraries

// Boost Bind
#include <boost/bind.hpp>

// Boost Thread Mutex
#include <boost/thread/mutex.hpp>

// Standard Messages - Float Type
#include "std_msgs/Float64.h"

// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <tf/transform_broadcaster.h>

#include <stdio.h>

// ROS Communication
#include "ros/ros.h"

// Maximum time delays
#define velocity_message_max_time_delay 0.03
#define steering_message_max_time_delay 0.03

// PID - Gain Values
#define Kp 100
#define Kd 20

namespace gazebo
{
  ///  A plugin to control the Hammvee driving.
  class HammveeDrivePlugin : public ModelPlugin
  {
    ///  Constructor
    public: HammveeDrivePlugin() {}

    /// The load function is called by Gazebo when the plugin is inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
    {
      // Store the pointer to the model
      this->model = _model;
	
      // Store the pointers to the joints
      this->steering_joint = this->model->GetJoint("ackerman_backL_link_joint");
      this->back_left_wheel_joint = this->model->GetJoint("back_left_wheel_joint");
      this->back_right_wheel_joint = this->model->GetJoint("back_right_wheel_joint");
      this->front_left_wheel_joint = this->model->GetJoint("front_left_wheel_joint");
      this->front_right_wheel_joint = this->model->GetJoint("front_right_wheel_joint");

      // Starting Timers
      steering_timer.Start();
      velocity_timer.Start();

      // Subscribe to the topic, and register a callback
      Steering_rate_sub = n.subscribe("/HAMMVEE_LLC/Steering_Angle" , 1000, &HammveeDrivePlugin::On_Steering_Angle_Msg, this);
      Velocity_rate_sub = n.subscribe("/HAMMVEE_LLC/Velocity" , 1000, &HammveeDrivePlugin::On_Velocity_Msg, this);
      // Listen to the update event. This event is broadcast every simulation iteration. 
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&HammveeDrivePlugin::OnUpdate, this, _1));

    }

    // Called by the world update start event, This function is the event that will be called every update
  public: void OnUpdate(const common::UpdateInfo & _info)  // we are not using the pointer to the info so its commanted as an option
    {
	// Applying effort to the wheels , brakes if no message income
	if (velocity_timer.GetElapsed().Float()>velocity_message_max_time_delay)
	{
		// Brakes
	  this->back_left_wheel_joint->SetForce(0,Set_Velocity_Back_Left_Wheel_Effort(1));
	  this->back_right_wheel_joint->SetForce(0,Set_Velocity_Back_Right_Wheel_Effort(1));
	  this->front_left_wheel_joint->SetForce(0,Set_Velocity_Front_Left_Wheel_Effort(1));
	  this->front_right_wheel_joint->SetForce(0,Set_Velocity_Front_Right_Wheel_Effort(1));
	}
	else
	{
	       // Accelerates
	  this->back_left_wheel_joint->SetForce(0,Set_Velocity_Back_Left_Wheel_Effort(0));
	  this->back_right_wheel_joint->SetForce(0,Set_Velocity_Back_Right_Wheel_Effort(0));
	  this->front_left_wheel_joint->SetForce(0,Set_Velocity_Front_Left_Wheel_Effort(0));
	  this->front_right_wheel_joint->SetForce(0,Set_Velocity_Front_Right_Wheel_Effort(0));
	}

	// Applying effort to the ackerman mechanisem , brakes if no message income
	if (steering_timer.GetElapsed().Float()>steering_message_max_time_delay)
	{
		// Stabilize
          this->steering_joint->SetForce(0,Set_Steering_Angle_Effort(1));
	}
	else
	{
	       // Turn
          this->steering_joint->SetForce(0,Set_Steering_Angle_Effort(0));
	}

	tf::Transform transform;
	transform.setOrigin( tf::Vector3(model->GetWorldPose().pos.x, model->GetWorldPose().pos.y, model->GetWorldPose().pos.z) );
	transform.setRotation(tf::Quaternion(model->GetWorldPose().rot.x,model->GetWorldPose().rot.y,model->GetWorldPose().rot.z,model->GetWorldPose().rot.w));

	TF_Broadcast(transform, "WORLD", model->GetName(), _info.simTime);

    }

	void TF_Broadcast(tf::Transform transform, std::string frame_id, std::string child_frame_id, common::Time time)
	{
		 static tf::TransformBroadcaster br;
		 tf::StampedTransform st(transform, ros::Time::now()/*(time.sec, time.nsec)*/, frame_id, child_frame_id);
		 br.sendTransform(st);

	}

     // Defining private Pointer to model
    private: physics::ModelPtr model;

     // Defining private Pointer to joints
    private: physics::JointPtr steering_joint;
    private: physics::JointPtr back_left_wheel_joint;
    private: physics::JointPtr back_right_wheel_joint;
    private: physics::JointPtr front_left_wheel_joint;
    private: physics::JointPtr front_right_wheel_joint;
     // Defining private Pointer to link
    private: physics::LinkPtr link;

     // Defining private Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Defining private Ros Node Handle
    private: ros::NodeHandle n;
    
    // Defining private Ros Subscribers
    private: ros::Subscriber Steering_rate_sub;
    private: ros::Subscriber Velocity_rate_sub;
    
    // Defining private Timers
    private: common::Timer steering_timer;
    private: common::Timer velocity_timer;

    // Defining private Reference Holders
    private: float steering_angle_ref;
    private: float velocity_ref;
   
    // Defining private Mutex
    private: boost::mutex steering_angle_ref_mutex;
    private: boost::mutex velocity_ref_mutex;


	// The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
	private: void On_Steering_Angle_Msg(const std_msgs::Float64ConstPtr &msg)
	{
	  steering_angle_ref_mutex.lock();
		  // Recieving referance steering angle
		  steering_angle_ref=msg->data;
		  // Reseting timer every time LLC publishes message
		  steering_timer.Reset();
	  steering_angle_ref_mutex.unlock();
	}

	// The subscriber callback , each time data is published to the subscriber this function is being called and recieves the data in pointer msg
	private: void On_Velocity_Msg(const std_msgs::Float64ConstPtr &msg)
	{
	  velocity_ref_mutex.lock();
		  // Recieving referance hammer velocity
		  velocity_ref=msg->data;
		  // Reseting timer every time LLC publishes message
		  velocity_timer.Reset();
	  velocity_ref_mutex.unlock();

	}

	// this function sets the efforts given to the ackerman mechanisem according to error getting to refarance position, effort inserted via steering_joint	
	// if brake command is recieved refarance change to 0
	private: float Set_Steering_Angle_Effort(int brake)
	{
		steering_angle_ref_mutex.lock();
			float error_dot,error,effort=0;
			if (brake)
				steering_angle_ref=0;
			error = (steering_angle_ref-(this->steering_joint->GetAngle(0).Radian()*180/3.14));
			error_dot = (0-(this->steering_joint->GetVelocity(0)*180/3.14)) ;
			effort = Kp*error+Kd*error_dot;
		steering_angle_ref_mutex.unlock();
		return effort;

	}

	// this function sets the efforts given to the hammer wheels according to error getting to refarance velocity, effort inserted via wheel_joints	
	// if brake command is recieved refarance change to 0
	private: float Set_Velocity_Back_Left_Wheel_Effort(int brake)
	{
		velocity_ref_mutex.lock();
			float error,effort=0;
			if (brake)
				velocity_ref=0;
			error = (velocity_ref-(this->back_left_wheel_joint->GetVelocity(0)));
			effort = Kp*error;
		velocity_ref_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Back_Right_Wheel_Effort(int brake)
	{
		velocity_ref_mutex.lock();
			float error,effort=0;
			if (brake)
				velocity_ref=0;
			error = (velocity_ref-(this->back_right_wheel_joint->GetVelocity(0)));
			effort = Kp*error;
		velocity_ref_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Front_Left_Wheel_Effort(int brake)
	{
		velocity_ref_mutex.lock();
			float error,effort=0;
			if (brake)
				velocity_ref=0;
			error = (velocity_ref-(this->front_left_wheel_joint->GetVelocity(0)));
			effort = Kp*error;
		velocity_ref_mutex.unlock();
		return effort;
	}

	private: float Set_Velocity_Front_Right_Wheel_Effort(int brake)
	{
		velocity_ref_mutex.lock();
			float error,effort=0;
			if (brake)
				velocity_ref=0;
			error = (velocity_ref-(this->front_right_wheel_joint->GetVelocity(0)));
			effort = Kp*error;
		velocity_ref_mutex.unlock();
		return effort;
	}

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(HammveeDrivePlugin)
}
#endif
