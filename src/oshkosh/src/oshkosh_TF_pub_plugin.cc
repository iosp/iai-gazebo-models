// Written By : Daniel
#include <stdlib.h>
#include <stdio.h>
// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS Communication
#include "ros/ros.h"

#include <tf/transform_broadcaster.h>

// Boost Bind
#include <boost/bind.hpp> 
#include <tf/transform_broadcaster.h>


namespace gazebo
{

class oshkoshTFPublisherPlugin : public ModelPlugin
{
  ///  Constructor
public:
  oshkoshTFPublisherPlugin() {}

  /// The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // we are not using the pointer to the sdf file so its commanted as an option
  {
    std::cout << "My major GAZEBO VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;

    this->model = _model;
    vehicle_name = model->GetName(); 
    std::cout << " Loading " << vehicle_name  << " TF publisher Plugin " << std::endl;


    // Starting Timers
    this->Ros_nh = new ros::NodeHandle(vehicle_name+"_oshkoshTFPublisherPlugin_node");
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&oshkoshTFPublisherPlugin::OnUpdate, this, _1));

    std::cout << " Loaded " << vehicle_name  << " TF publisher Plugin " << std::endl;
  }


  // Called by the world update start event, This function is the event that will be called every update
public:
  void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
  {
    tf::Transform transform;
    ignition::math::Pose3d pose = model->WorldPose(); 
	  transform.setOrigin( tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()) );
	  transform.setRotation(tf::Quaternion(pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z(),pose.Rot().W()));

	  TF_Broadcast(transform, "WORLD", model->GetName(), simInfo.simTime);
  }


  void TF_Broadcast(tf::Transform transform, std::string frame_id, std::string child_frame_id, common::Time time)
	{
		 static tf::TransformBroadcaster br;
		 tf::StampedTransform st(transform, ros::Time::now()/*(time.sec, time.nsec)*/, frame_id, child_frame_id);
		 br.sendTransform(st);
	}
 
   
  // Defining private Pointer to model
  physics::ModelPtr model;
  std::string vehicle_name;

  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(oshkoshTFPublisherPlugin)
}
