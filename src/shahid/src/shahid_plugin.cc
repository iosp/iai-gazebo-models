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

      this->actor_sylinder = this->world->GetModel(this->actor_name + "_cylinder_model");
        
 

      this->Ros_nh = new ros::NodeHandle("shahid_actor_node");
      
      // Subscribe to the topic, and register a callback
      target_pose_sub = this->Ros_nh->subscribe("/"+actor_name+"/target_pose", 60, &shahidPlugin::Newtarget_pose, this);
         

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&shahidPlugin::OnUpdate, this, _1));

    }

/////////////////////////////////////////////////
    void Reset()
    {
      this->velocity = 1.0;
      this->lastUpdate = 0;
     
      this->target_pose = ignition::math::Vector3d(this->actor->WorldPose().Pos());
      

      ignition::math::Pose3d actorPose = this->actor->WorldPose();
      ignition::math::Vector3d actorRPY = actorPose.Rot().Euler();
      actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, actorRPY.Z());
      this->actor->SetWorldPose(actorPose, false, false);
      

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
    void Newtarget_pose(const geometry_msgs::PointPtr &msg)
    {
      this->target_pose.X(msg->x);
      this->target_pose.Y(msg->y);
      this->velocity = msg->z;
    }


/////////////////////////////////////////////////
    public: void OnUpdate(const common::UpdateInfo & _info)
    {
      // Time delta
      double dt = (_info.simTime - this->lastUpdate).Double();
      this->lastUpdate = _info.simTime;
      
      ignition::math::Pose3d actorPose = this->actor->WorldPose();
      //std::cout << "actor  pose :   x = " << actorPose.Pos().X() << " y = " << actorPose.Pos().Y() << std::endl;
      //std::cout << "target pose :   x = " << this->target_pose.X() << " y = " << this->target_pose.Y() << std::endl;
      

   


      ignition::math::Vector3d actorRPY = actorPose.Rot().Euler();
      ignition::math::Vector3d targetDiss = this->target_pose - actorPose.Pos();

      if (targetDiss.Length() < 0.5)
      {   
        return;
      }

      ignition::math::Vector3d targetDirection = targetDiss.Normalize(); 
      ignition::math::Angle yaw = atan2(targetDirection.Y(), targetDirection.X()) + 1.5707 - actorRPY.Z();
      yaw.Normalize();

      ignition::math::Pose3d actorNextPose = actorPose;
      actorNextPose.Rot() = ignition::math::Quaterniond(1.5707, 0, actorRPY.Z() + yaw.Radian());
      double disTraveled = 0;


      // Rotate in place, instead of jumping.
      if (std::abs(yaw.Radian()) > IGN_DTOR(10))
          {
            actorNextPose.Rot() = ignition::math::Quaterniond(1.5707, 0, actorRPY.Z() + yaw.Radian()*0.01);
          }
      else
          {
            actorNextPose.Pos() = actorPose.Pos() + targetDirection * this->velocity * dt;
            disTraveled = (this->velocity * dt).Length();
          }

      //std::cout << "next pose :   x = " << actorNextPose.Pos().X() << " y = " << actorNextPose.Pos().Y() << std::endl;
          

      // Distance traveled is used to coordinate motion with the walking animation
      this->actor->SetWorldPose(actorNextPose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() + (disTraveled * this->animationFactor));
      
      
     ignition::math::Pose3d cylinder_NextPose = actorNextPose;
     cylinder_NextPose.Rot() = ignition::math::Quaterniond( 0 , 0, 0);
     this->actor_sylinder->SetLinkWorldPose(cylinder_NextPose, this->actor_name+"_cylinder_link");
    }


      
    private: ros::NodeHandle *Ros_nh;  // Defining private Ros Node Handle
    private: ros::Subscriber target_pose_sub; // Defining private Ros Subscribers
    
    private: std::string actor_name;   


    private: physics::ModelPtr actor_sylinder;


    private: event::ConnectionPtr updateConnection;    // Pointer to the update event connection

    private: physics::ActorPtr actor;
    private: physics::WorldPtr world;
    private: sdf::ElementPtr sdf;
    private: ignition::math::Vector3d target_pose;

    private: common::Time lastUpdate;
    private: ignition::math::Vector3d velocity; // Velocity of the actor

    private: physics::TrajectoryInfoPtr trajectoryInfo; //Custom trajectory info.
    
    private: double animationFactor = 4.5; // Time scaling factor. Used to coordinate translational motion with the actor's walking animation.
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(shahidPlugin)
}