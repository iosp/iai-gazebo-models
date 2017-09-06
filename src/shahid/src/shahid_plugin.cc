#include <stdio.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>

#include <boost/bind.hpp>

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



      physics::ModelPtr model = this->world->GetModel("worldHightmap");
      physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
      this->heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());


      this->Reset();
      
      //Read in the shahid_name from the sdf file
      if (_sdf->HasElement("shahid_name"))
        this->actor_Name = _sdf->Get<std::string>("shahid_name");
      else
        this->actor_Name = "musa";

      //Read in the animationFrameRate from the sdf file
      if (_sdf->HasElement("animationFrameRate"))
        this->animationFrameRate = _sdf->Get<double>("animationFrameRate");
      else
        this->animationFrameRate = 30.0;

      //Read in the carectorHight from the sdf file
      if (_sdf->HasElement("carectorHight"))
        this->carectorHight = _sdf->Get<double>("carectorHight");
      else
        this->carectorHight = 2.0;


      // conecting to the BoundingCylinder   
      this->actor_BoundingCylinder_ModelName = this->actor_Name + "_boundingCylinder_model";
      this->actor_BoundingCylinder_LinkName = this->actor_Name+"_boundingCylinder_link";
      this->actor_BoundingCylinder = this->world->GetModel(this->actor_BoundingCylinder_ModelName);

 

      this->Ros_nh = new ros::NodeHandle("shahid_actor_node");
      
      // Subscribe to the topic, and register a callback
      actorTargetPose_sub = this->Ros_nh->subscribe("/"+actor_Name+"/target_pose", 60, &shahidPlugin::newActorTarget, this);
      

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&shahidPlugin::OnUpdate, this, _1));

    }

/////////////////////////////////////////////////
    double HightMapZ(double x, double y)
    {
    // The HeightmapShape does not work with the same coordinate system, so get some data:
    math::Vector3 size = this->heightmap->GetSize(); 
    math::Vector2i vc = this->heightmap->GetVertexCount();


    // And finally get your z from your x and y:
    int index_x = (  ( (x + size.x/2)/size.x ) * vc.x - 1 ) ;
    int index_y = (  ( (-y + size.y/2)/size.y ) * vc.y - 1 ) ;
    double z =  this->heightmap->GetHeight( index_x , index_y ) ; 

    return z;
    }

/////////////////////////////////////////////////
    void Reset()
    {
      this->actor_TargetVel = 1.0;
      this->lastUpdateAnimation = 0;
     
      this->actor_TargetPose = ignition::math::Vector3d(this->actor->WorldPose().Pos());
      

      ignition::math::Pose3d actorPose = this->actor->WorldPose();
      ignition::math::Vector3d actorRPY = actorPose.Rot().Euler();
      actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, actorRPY.Z());
      actorPose.Pos().Z() = this->HightMapZ( actorPose.Pos().X(), actorPose.Pos().Y()) + carectorHight/2 ; 
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
    void newActorTarget(const geometry_msgs::PointPtr &msg)
    {
      this->actor_TargetPose.X(msg->x);
      this->actor_TargetPose.Y(msg->y);
      this->actor_TargetPose.Z(this->HightMapZ( msg->x, msg->y) + carectorHight/2);
      this->actor_TargetVel = msg->z;     
    }


/////////////////////////////////////////////////
    public: void OnUpdate(const common::UpdateInfo & _info)
    {

      // Time delta
      double dtAnimation = (_info.simTime - this->lastUpdateAnimation).Double();
      if (dtAnimation >= 1/animationFrameRate )
       {
        
          this->lastUpdateAnimation = _info.simTime;

          ignition::math::Pose3d actorPose = this->actor->WorldPose();
          
          ignition::math::Vector3d actorRPY = actorPose.Rot().Euler();
          ignition::math::Vector3d targetDiss = this->actor_TargetPose - actorPose.Pos();

          
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
                actorNextPose.Rot() = ignition::math::Quaterniond(1.5707, 0, actorRPY.Z() + yaw.Radian()*dtAnimation*this->actor_TargetVel.Length());
              }
          else
              {
                actorNextPose.Pos() = actorPose.Pos() + targetDirection * this->actor_TargetVel * dtAnimation;
                disTraveled = (this->actor_TargetVel * dtAnimation).Length();
              }
              
              actorNextPose.Pos().Z() = this->HightMapZ( actorNextPose.Pos().X(), actorNextPose.Pos().Y()) + carectorHight/2;    
              
          // Distance traveled is used to coordinate motion with the walking animation
          this->actor->SetWorldPose(actorNextPose, false, false);
          this->actor->SetScriptTime(this->actor->ScriptTime() + (disTraveled * this->animationFactor));
                

          // moving the bounding Cylinder 
          ignition::math::Pose3d boundingCylinder_NextPose = actorNextPose;
          boundingCylinder_NextPose.Rot() = ignition::math::Quaterniond( 0 , 0, 0);
          this->actor_BoundingCylinder->SetLinkWorldPose(boundingCylinder_NextPose, this->actor_BoundingCylinder_LinkName);
                    
        }
    }


      
    private: ros::NodeHandle *Ros_nh;  // Defining private Ros Node Handle
    private: ros::Subscriber actorTargetPose_sub; // Defining private Ros Subscribers
    
    private: std::string actor_Name; 
    private: double carectorHight;  

    private: physics::ModelPtr actor_BoundingCylinder;
    private: std::string actor_BoundingCylinder_ModelName;
    private: std::string actor_BoundingCylinder_LinkName;

    private: event::ConnectionPtr updateConnection;    // Pointer to the update event connection

    private: physics::ActorPtr actor;
    private: physics::WorldPtr world;
    private: physics::HeightmapShapePtr heightmap;
    private: sdf::ElementPtr sdf;

    private: ignition::math::Vector3d actor_TargetPose;
    private: ignition::math::Vector3d actor_TargetVel; // actor_TargetVel of the actor

    private: common::Time lastUpdateAnimation;
    private: double animationFrameRate;
  
    private: physics::TrajectoryInfoPtr trajectoryInfo; //Custom trajectory info.
    private: double animationFactor = 4.5; // Time scaling factor. Used to coordinate translational motion with the actor's walking animation.
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(shahidPlugin)
}