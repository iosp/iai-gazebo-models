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


#include <ignition/math/Angle.hh>

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


  /// \brief The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr <<this->model_name << ": Invalid joint count, velodyne16 plugin not loaded\n";
      return;
    }

    // Store the model pointer for convenience.
    this->model = _model;

    this->model_name = _model->GetName(); 
    std::cout << "Loading " << model_name << " Plugin" << std::endl;
    this->parent_name = _model->GetParentModel()->GetName();
    std::cout << "parent_name = " << parent_name << std::endl;
    
    
    //  std::cout << "_model->GetName() = " <<  _model->GetName()  << std::endl;  
    //  std::cout << "_model->GetParentModel()->GetName() = " <<  _model->GetParentModel()->GetName() << std::endl;        
    //  std::cout << "_model->GetParentModel()->GetParentModel()->GetName() = " <<  _model->GetParentModel()->GetParentModel()->GetName() << std::endl;    
    //  std::cout << "_model->GetPluginCount() = " << _model->GetPluginCount() << std::endl;
    //  std::cout << "_model->GetScopedName() = " <<   _model->GetScopedName() << std::endl;
    

    // set the angle resolution, default is 0.2 deg
    this->angleRes = 0.2;  // Default is 0.2deg
    if (_sdf->HasElement("angleRes"))
    this->angleRes = _sdf->Get<double>("angleRes");
    else
      ROS_WARN("velodyne16: there is no 'angleRes' parameter in the SDF seting to defult of 0.2deg");
   

    // geting the verticalAngleResolution from sdf
    verticalAngleResolution = 0;
    if (_sdf->HasElement("verticalAngleResolution"))
      verticalAngleResolution = _sdf->Get<double>("verticalAngleResolution");
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
         


    // Test for whether the model is nested and a prefix is needed to find the joint
    std::string pref = _model->GetName();
    if( ! this->model->GetJoint(pref+"::velodyne16_joint") )
        pref = "velodyne16";

    this->joint = this->model->GetJoint(pref+"::velodyne16_joint");    


    //set the topic
    std::string topic_name = model_name+"/velodyne16";
    _pointCloud_pub = _nodeHandle.advertise<sensor_msgs::PointCloud>(topic_name, 10);

    //init the ray sensors
    initSensors();

    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&velodyne16::OnUpdate, this, _1));
  }


  void initSensors()
  {
    sensors::Sensor_V v_sensor = gazebo::sensors::SensorManager::Instance()->GetSensors();

    int senCount = 0;
    for (auto sensor_it = v_sensor.begin(); sensor_it != v_sensor.end(); ++sensor_it) 
        {
          std::string p_name = (*sensor_it)->ParentName();
          std::string s_name = this->parent_name + "::velodyne16";
          if  (p_name.compare(0, s_name.length(), s_name) == 0 )
            {
              #if CPUvsGPU == 1
                std::cout << " casting in to CPU ray sensor "  << senCount << " parent_name = " << parent_name <<  "   p_name = " << p_name << std::endl; 
                raySensors.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(*sensor_it));
              #elif CPUvsGPU == 2
                std::cout << " casting in to GPU ray sensor " << senCount << " parent_name = " << parent_name <<  "   p_name = " << p_name << std::endl; 
                raySensors.push_back(std::dynamic_pointer_cast<sensors::GpuRaySensor>(*sensor_it));
              #endif
              senCount++;
            }
        }
    
    this->numOfRaySensors = senCount;    
  }


  void OnUpdate(const common::UpdateInfo &_info)
  {
    common::Time currentUpdateTime = _info.simTime;  

    double dTime = (currentUpdateTime - this->lastUpdateTime).Double();
    
    ignition::math::Angle dAngle =  dTime * 2*M_PI*this->rotRate;
    ignition::math::Angle AnglStep =  M_PI/180*(this->angleRes*this->numOfRaySensors);

    if (dAngle.Degree() < AnglStep.Degree() )
      return;

    if (dAngle.Degree() > 2*AnglStep.Degree() ) {
      gzerr << this->model_name << ":  Step resolution of " << this->angleRes << " cannot be reached !! \n"; 
      return; 
          }

    common::Time sensorsUpdateTime = getRanges();
        
    ignition::math::Angle currentJointAngle =  this->joint->Position(0);
    double sensorDelay = currentUpdateTime.Double() - sensorsUpdateTime.Double();
    if (  ( sensorDelay >= 0 )  && (sensorDelay <= 0.007  ) )
    {
     ignition::math::Angle sensorDelayAngle =  sensorDelay * 2*M_PI*this->rotRate;

     ignition::math::Angle AngleCorectionToSensorDelay = 0;
     if( CPUvsGPU == 1) // CPU = 1 , GPU = 2
         AngleCorectionToSensorDelay =  ( std::floor( sensorDelayAngle.Radian() / AnglStep.Radian() ) ) * AnglStep.Radian(); 
     else if (CPUvsGPU == 2 )  
         AngleCorectionToSensorDelay =  ( std::floor( (sensorDelayAngle.Radian() - 0.5*AnglStep.Radian() )/ AnglStep.Radian() ) ) * AnglStep.Radian(); 
     
     
    float test = 0; // sensorDelay * 1000;


     boost::thread(&velodyne16::thread_RVIZ, this,  this->rangesArray, currentJointAngle - AngleCorectionToSensorDelay  ,this->lastUpdateTime , test );
     //std::cout << " sensorDelay = " << sensorDelay <<  "     AngleCorectionToSensorDelay = "  << AngleCorectionToSensorDelay.Degree()  << std::endl;
    }


    ignition::math::Angle jointNextAngle = currentJointAngle +  dAngle; 
    this->joint->SetPosition(0,jointNextAngle.Radian()); 

    this->lastUpdateTime = currentUpdateTime;

  }


  common::Time getRanges()
  {
    common::Time sensorDataTime = this->raySensors[0]->LastUpdateTime(); 
    double minTime = sensorDataTime.Double();
    double maxTime = sensorDataTime.Double();       
    
    
    int unSyncRaysConter = 0;
    for (int sensor_i = 0; sensor_i < this->numOfRaySensors; sensor_i++)
    {
      std::vector<double> rayMeasuresVec;
      this->raySensors[sensor_i]->Ranges(rayMeasuresVec);
      
      
      if (rayMeasuresVec.size() != NUM_OF_PLANES) {
        if ( rayMeasuresVec.size() >= 1)  // avoiding error prints before the sensors loaded 
        gzerr <<this->model_name << ": rayMeasuresVec.size() != NUM_OF_PLANES  \n"; 
        return(sensorDataTime);  
      }
      
      common::Time rayDataTime = this->raySensors[sensor_i]->LastUpdateTime();
      
      if ( rayDataTime.Double() - sensorDataTime.Double() > 0.000  ) { 
        std::fill(rayMeasuresVec.begin(), rayMeasuresVec.end(), 0);   // getting rid of the scans that are not in sync with sensor_i = 0 ,  more relevant for the GPU
        unSyncRaysConter++;
        gzerr << this->model_name << " : velodyne rayDataTime unsync - throwing mesurment    ( sensor_i = " << sensor_i <<  " rayDataTime = " <<  rayDataTime.Double()  << "   sensorDataTime = "  << sensorDataTime.Double() << " ) \n"; 
      }
      
      for (int plain_i = 0; plain_i < NUM_OF_PLANES; plain_i++)         
      this->rangesArray[sensor_i][plain_i] = rayMeasuresVec[plain_i];
    }
    
      if (unSyncRaysConter >= 0.9*this->numOfRaySensors  ) {
        gzerr << this->model_name << ":  The number of un-sync rays (that are thrown) is : " << unSyncRaysConter << "\n" ; 
        return(sensorDataTime);  
        }
       
     return(sensorDataTime);
  }



  void thread_RVIZ(double rangesArray[][NUM_OF_PLANES], ignition::math::Angle sensorAngle ,common::Time time, float test)
  {
    //		ros::Time RVIZlastUpdateTime = ros::Time::now();
    //		while(true)
    //		{
    //			ros::Time newRosTime = ros::Time::now();
    //			double diff = (newRosTime.toSec() - RVIZlastUpdateTime.toSec()) - (1/RVIZPublishRate);
    //			if(diff > 0.0001)
    //			{
    ros::Time t(time.sec, time.nsec);
    RVIZ_Publisher(rangesArray, t ,sensorAngle, test);
    //				RVIZlastUpdateTime = newRosTime;
    // TF publish
    TF_Broadcast(0.0, 0.0, 0.0, 0.0, 0.0, sensorAngle.Radian(), model_name, model_name+"_velodyne16", t);
    //			}
    //		}
  }


  void RVIZ_Publisher(double rangesArray[][NUM_OF_PLANES], ros::Time time, ignition::math::Angle sensorAngle, float test)
  {
      sensor_msgs::PointCloud points;
      for (int sensor_i = 0; sensor_i < this->numOfRaySensors; sensor_i++)
      {
        for (int plan_i = 0; plan_i < NUM_OF_PLANES; plan_i++)
        {
          geometry_msgs::Point32 point;
          double yaw_ang = sensor_i * this->angleRes * (M_PI/180); 
          double pitch_ang = (this->verticalAngelMin + plan_i * verticalAngleResolution) * (M_PI/180);
          point.x = rangesArray[sensor_i][plan_i]  * cos(pitch_ang) * cos(yaw_ang);
          point.y = rangesArray[sensor_i][plan_i]  * cos(pitch_ang) * sin(yaw_ang);
          point.z = test + rangesArray[sensor_i][plan_i]  * sin(pitch_ang);
          points.points.push_back(point);
        }
      }
      points.header.stamp = time; //ros::Time();
      points.header.frame_id = parent_name+"_velodyne16"; 

      _pointCloud_pub.publish(points);   
  }

  
  void TF_Broadcast(double x, double y, double z, double Roll, double Pitch, double Yaw, std::string frame_id, std::string child_frame_id, ros::Time t)
  {
    static tf::TransformBroadcaster br;
    tf::StampedTransform st(transformBuilder(x, y, z, Roll, Pitch, Yaw), t, frame_id, child_frame_id);
    br.sendTransform(st);
  }

  tf::Transform transformBuilder(float x, float y, float z, float Roll, float Pitch, float Yaw)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));

    tf::Quaternion q;
    q.setRPY(Roll, Pitch, Yaw);
    transform.setRotation(q);
    return (transform);
  }

  
  
  
  
  double angleRes; 
  double rotRate;
  double verticalAngleResolution;
  double verticalAngelMin;
//double RVIZPublishRate;

  physics::ModelPtr model;
  physics::JointPtr joint;


  std::string model_name;
  std::string parent_name;

  
  #if CPUvsGPU == 1
    vector<gazebo::sensors::RaySensorPtr> raySensors;  
  #elif CPUvsGPU == 2
    vector<gazebo::sensors::GpuRaySensorPtr> raySensors;
  #endif
  
  double rangesArray[MAX_NUM_OF_RAY_SENSORS][NUM_OF_PLANES];
  int numOfRaySensors;

  common::Time lastUpdateTime;

  event::ConnectionPtr _updateConnection; // Pointer to the update event connection


  ros::NodeHandle _nodeHandle;
  ros::Publisher _pointCloud_pub;
  //boost::thread _threadRVIZ;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(velodyne16)
}
#endif
