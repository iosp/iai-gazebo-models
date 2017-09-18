#ifndef _velodyne16_PLUGIN_HH_
#define _velodyne16_PLUGIN_HH_

#include <stdlib.h>
#include <stdio.h>

#include <gazebo/gazebo.hh>
#include "gazebo/sensors/sensors.hh"

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"

#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/GpuRaySensor.hh"

#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Joint.hh"
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <boost/thread.hpp>
#include <string>
#include <tf/transform_broadcaster.h>

#include <math.h>

#define MAX_NUM_OF_RAY_SENSORS 50 // 32 - CPU, 16 - GPU 
#define CPUvsGPU 2 // CPU = 1 , GPU = 2

#define NUM_OF_PLANES 16
 


using namespace std;

namespace gazebo
{
/// \brief A plugin to control a velodyne16 sensor.
class velodyne16 : public ModelPlugin
{
  /// \brief Constructor
public:
  velodyne16() {}

  tf::Transform transformBuilder(float x, float y, float z, float Roll, float Pitch, float Yaw)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));

    tf::Quaternion q;
    q.setRPY(Roll, Pitch, Yaw);
    transform.setRotation(q);
    return (transform);
  }

  void TF_Broadcast(double x, double y, double z, double Roll, double Pitch, double Yaw, std::string frame_id, std::string child_frame_id, ros::Time t)
  {
    static tf::TransformBroadcaster br;
    tf::StampedTransform st(transformBuilder(x, y, z, Roll, Pitch, Yaw), t, frame_id, child_frame_id);
    br.sendTransform(st);
  }

  void thread_RVIZ(double rangesArray[][NUM_OF_PLANES], math::Angle sensorAngle ,common::Time time)
  {
    //		ros::Time RVIZlastUpdateTime = ros::Time::now();
    //		while(true)
    //		{
    //			ros::Time newRosTime = ros::Time::now();
    //			double diff = (newRosTime.toSec() - RVIZlastUpdateTime.toSec()) - (1/RVIZPublishRate);
    //			if(diff > 0.0001)
    //			{
    ros::Time t(time.sec, time.nsec);
    RVIZ_Publisher(rangesArray, t ,sensorAngle);
    //				RVIZlastUpdateTime = newRosTime;
    // TF publish
    TF_Broadcast(0.0, 0.0, 0.0, 0.0, 0.0, sensorAngle.Radian(), model_name, model_name+"_velodyne16", t);
    //			}
    //		}
  }

  void OnUpdate(const common::UpdateInfo &_info)
  {
    double dTime = (_info.simTime - this->lastUpdateTime).Double();
    
    math::Angle dAngle =  dTime * 2*M_PI*this->rotRate;

    //std::cout << " dTime = " << dTime <<  " dAngle = " << dAngle.Degree() << " Step = "  << this->angleRes * numOfRaySensors << std::endl;

    math::Angle AnglStep =  M_PI/180*(this->angleRes*this->numOfRaySensors);

    if (dAngle.Degree() < AnglStep.Degree() )
      return;

    if (dAngle.Degree() > 2*AnglStep.Degree() ) {
      gzerr << " Step resolution of " << this->angleRes << " cannot be reached !! \n"; 
      return; 
          }

    this->lastUpdateTime = _info.simTime;


    getRanges();

    

    math::Angle jointAngle =  this->joint->Position(0);
    
    math::Angle jointNextAngle = jointAngle +  dAngle; // AnglStep; //
    
    this->joint->SetPosition(0,jointNextAngle.Radian()); 

   // std::cout << " dTime = " << dTime <<  " dAngle = " << dAngle.Degree() <<  "  jointNextAngle.Degree() = " << jointNextAngle.Degree() << std::endl;
    

    boost::thread(&velodyne16::thread_RVIZ, this,  this->rangesArray, jointAngle ,_info.simTime);

  }

  void initSensors()
  {
    sensors::Sensor_V v_sensor = gazebo::sensors::SensorManager::Instance()->GetSensors();

    int senCount = 0;
    for (auto sensor_it = v_sensor.begin(); sensor_it != v_sensor.end(); ++sensor_it) 
        {
          std::string p_name = (*sensor_it)->ParentName();
          if (p_name.compare(0, parent_name.length(), parent_name) == 0 )
            {
              #if CPUvsGPU == 1
                std::cout << " casting in to CPU ray sensor " << senCount << std::endl; 
                raySensors.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(*sensor_it));
              #elif CPUvsGPU == 2
                std::cout << " casting in to GPU ray sensor " << senCount << std::endl;               
                raySensors.push_back(std::dynamic_pointer_cast<sensors::GpuRaySensor>(*sensor_it));
              #endif
              senCount++;
            }
        }
    
    this->numOfRaySensors = senCount;    
  }

  //getRanges
  void getRanges()
  {
    if(this->raySensors.size() != this->numOfRaySensors) {
      gzerr << " this->raySensors.size() != this->numOfRaySensors  \n"; 
      return; 
         }

    for (int sensor_i = 0; sensor_i < this->numOfRaySensors; sensor_i++)
      {
        std::vector<double> rayMeasuresVec;

        this->raySensors[sensor_i]->Ranges(rayMeasuresVec);

      if (rayMeasuresVec.size() != NUM_OF_PLANES) {
        gzerr << " rayMeasuresVec.size() != NUM_OF_PLANES  \n"; 
        return;
        }

      for (int plain_i = 0; plain_i < NUM_OF_PLANES; plain_i++)
         {
           this->rangesArray[sensor_i][plain_i] = rayMeasuresVec[plain_i];
         }
      }
  }



  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, velodyne16 plugin not loaded\n";
      return;
    }

    // Store the model pointer for convenience.
    this->model = _model;

    this->model_name = model->GetName(); 
    std::cout << "Loading " << model_name << " Plugin" << std::endl;
    this->parent_name = model->GetParentModel()->GetName();
    std::cout << "parent_name = " << parent_name << std::endl;
    


    // set the angle resolution, default is 0.2 deg
    this->angleRes = 0.2;  // Default is 0.2deg
    if (_sdf->HasElement("angleRes"))
    this->angleRes = _sdf->Get<double>("angleRes");
    else
      ROS_WARN("velodyne16: there is no 'angleRes' parameter in the SDF seting to defult of 0.2deg");
   

    // geting the verticalAngleResolutionFromSDF from sdf
    // using for validate the user, compare this veriable and real vertical angle (VerticalAngelResolutionReal[])
    verticalAngleResolutionFromSDF = 0;
    if (_sdf->HasElement("verticalAngleResolution"))
      verticalAngleResolutionFromSDF = _sdf->Get<double>("verticalAngleResolution");
    else
      ROS_WARN("velodyne16: there is no 'verticalAngleResolution' parameter in the SDF (validation parameter)");

    
    // geting the verticalAngelMin from sdf
    verticalAngelMin = 0; // Default is 0 deg
    if (_sdf->HasElement("verticalAngelMin"))
      verticalAngelMin = _sdf->Get<double>("verticalAngelMin");
    else
      ROS_WARN("velodyne16: there is no 'verticalAngelMin' parameter in the SDF seting to defult of 0");


    
    // geting the rotRate from sdf
    this->rotRate = 1.0; // Default is 1HZ velocity
    if (_sdf->HasElement("rotRate"))
      this->rotRate = _sdf->Get<double>("rotRate");
    else 
      ROS_WARN("velodyne16: there is no 'rotRate' parameter in the SDF seting to defult of 1Hz");
    
      
    //RVIZ Publish Rate, default is 1HZ
    //RVIZPublishRate = 1.0;   // Default is 0Hz
    //if (_sdf->HasElement("RVIZPublishRate"))
    //   RVIZPublishRate = _sdf->Get<double>("RVIZPublishRate");
    //else
    //   ROS_WARN("velodyne16: there is no 'RVIZPublishRate' parameter in the SDF seting to defult of 1Hz");
         


    this->TopLink = _model->GetLink("velodyne16::top");

    this->joint = _model->GetJoint("velodyne16::velodyne16_joint");    


    //set the topic
    std::string topic_name = model_name+"/velodyne16";
    _pointCloud_pub = _nodeHandle.advertise<sensor_msgs::PointCloud>(topic_name, 10);

    //init the ray sensors
    initSensors();

    lastDegree = -1;
    this->sensorDegree = 0;
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&velodyne16::OnUpdate, this, _1));
 
  }

  void RVIZ_Publisher(double rangesArray[][NUM_OF_PLANES], ros::Time time, math::Angle sensorAngle)
  {
      sensor_msgs::PointCloud points;
      for (int sensor_i = 0; sensor_i < this->numOfRaySensors; sensor_i++)
      {
        for (int plan_i = 0; plan_i < NUM_OF_PLANES; plan_i++)
        {
          geometry_msgs::Point32 point;
          double yaw_ang = sensor_i * this->angleRes * (M_PI/180); 
          double pitch_ang = (this->verticalAngelMin + plan_i * verticalAngleResolutionFromSDF) * (M_PI/180);
          point.x = rangesArray[sensor_i][plan_i]  * cos(pitch_ang) * cos(yaw_ang);
          point.y = rangesArray[sensor_i][plan_i]  * cos(pitch_ang) * sin(yaw_ang);
          point.z = rangesArray[sensor_i][plan_i]  * sin(pitch_ang);
          points.points.push_back(point);
        }
      }
      points.header.stamp = time; //ros::Time();
      points.header.frame_id = parent_name+"_velodyne16"; 

      _pointCloud_pub.publish(points);

     // if(sensorAngle - this->test_prev_sensorAngle != M_PI/180*(this->angleRes*this->numOfRaySensors) )
     //       std::cout << " Time = " << time << " diff =  " << sensorAngle.Degree() - this->test_prev_sensorAngle.Degree() << std::endl;            
     // test_prev_sensorAngle =  sensorAngle;   
  }

 // math::Angle test_prev_sensorAngle;
  

  physics::ModelPtr model;

  physics::JointPtr joint;
  physics::LinkPtr TopLink;
  physics::LinkPtr TopLinkAcncore;

  std::string model_name;
  std::string parent_name;
  common::PID pid;



  #if CPUvsGPU == 1
    vector<gazebo::sensors::RaySensorPtr> raySensors;  
  #elif CPUvsGPU == 2
    vector<gazebo::sensors::GpuRaySensorPtr> raySensors;
  #endif


  event::ConnectionPtr _updateConnection; // Pointer to the update event connection
  
  private: common::Time lastUpdateTime;
  

  double lastDegree;
  double angleRes;
  double sensorDegree;
  double rotRate;


  int numOfRaySensors;

  double rangesArray[MAX_NUM_OF_RAY_SENSORS][NUM_OF_PLANES];
  //   double RVIZPublishRate;
  double verticalAngleResolutionFromSDF;
  double VerticalAngelResolutionReal[MAX_NUM_OF_RAY_SENSORS];
  double verticalAngelMin;
  ros::Publisher _pointCloud_pub;
  ros::NodeHandle _nodeHandle;

  boost::thread _threadRVIZ;

  ros::NodeHandle n;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(velodyne16)
}
#endif
