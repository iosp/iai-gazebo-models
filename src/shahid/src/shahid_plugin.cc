#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>

#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// ROS Communication
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#define WALKING_ANIMATION "walking"

namespace gazebo
{
  class shahidPlugin : public ModelPlugin
  {

/////////////////////////////////////////////////
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->sdf = _sdf;
      this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
      this->world = this->actor->GetWorld();
    
      this->Reset();
      
      //Read in the animation factor (applied in the OnUpdate function).
      if (_sdf->HasElement("shahid_name"))
        this->actor_name = _sdf->Get<std::string>("shahid_name");
      else
        this->actor_name = "jon";
      

      this->Ros_nh = new ros::NodeHandle("shahid_actor_node");
      
      // Subscribe to the topic, and register a callback
      target_sub = this->Ros_nh->subscribe("/"+actor_name+"/target", 60, &shahidPlugin::NewTarget, this);
         

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&shahidPlugin::OnUpdate, this, _1));
    }

/////////////////////////////////////////////////
    void Reset()
    {
      this->velocity = 1.0;
      this->lastUpdate = 0;

      this->target = ignition::math::Vector3d(10, 10, 1.0);

      auto skelAnims = this->actor->SkeletonAnimations();
      if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
      {
        gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
      }
      else
      {
        // Create custom trajectory
        this->trajectoryInfo.reset(new physics::TrajectoryInfo());
        this->trajectoryInfo->type = WALKING_ANIMATION;
        this->trajectoryInfo->duration = 1.0;

        this->actor->SetCustomTrajectory(this->trajectoryInfo);
      }
    }

/////////////////////////////////////////////////
    void NewTarget(const geometry_msgs::PointPtr &msg)
    {
      this->target.X(msg->x);
      this->target.Y(msg->y);
      this->velocity = msg->z;
    }


/////////////////////////////////////////////////
    public: void OnUpdate(const common::UpdateInfo & _info)
    {

      // Time delta
      double dt = (_info.simTime - this->lastUpdate).Double();
  
      ignition::math::Pose3d pose = this->actor->WorldPose();
      ignition::math::Vector3d pos = this->target - pose.Pos();
      ignition::math::Vector3d rpy = pose.Rot().Euler();
      
      double distance = pos.Length();
      
      pos = pos.Normalize(); 

      ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
      yaw.Normalize();

      // Rotate in place, instead of jumping.
      if (std::abs(yaw.Radian()) > IGN_DTOR(10))
      {
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
            yaw.Radian()*0.001);
      }
      else
      {
        pose.Pos() += pos * this->velocity * dt;
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
      }


      // Distance traveled is used to coordinate motion with the walking animation
      double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

      this->actor->SetWorldPose(pose, false, false);
      
      this->actor->SetScriptTime(this->actor->ScriptTime() +
                     (distanceTraveled * this->animationFactor));
      this->lastUpdate = _info.simTime;

    }


      
    private: ros::NodeHandle *Ros_nh;  // Defining private Ros Node Handle
    private: ros::Subscriber target_sub; // Defining private Ros Subscribers
    
    private: std::string actor_name;   


    private: event::ConnectionPtr updateConnection;    // Pointer to the update event connection

    private: physics::ActorPtr actor;
    private: physics::WorldPtr world;
    private: sdf::ElementPtr sdf;
    private: ignition::math::Vector3d target;

    private: common::Time lastUpdate;
    private: ignition::math::Vector3d velocity; // Velocity of the actor

    private: physics::TrajectoryInfoPtr trajectoryInfo; //Custom trajectory info.
    
    private: double animationFactor = 4.5; // Time scaling factor. Used to coordinate translational motion with the actor's walking animation.
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(shahidPlugin)
}