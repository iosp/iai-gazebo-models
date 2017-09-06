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

#define NUM_OF_RAY_SENSORS 16 // 36 - CPU, 16 - GPU 
#define CPUvsGPU 2 // CPU = 1 , GPU = 2

#define NUM_OF_PLANES 16
#define ANGULAR_STEPS 1800 // 360 * 5 (resulution of 0.2deg)
 


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

  void thread_RVIZ(double rangesArray[][NUM_OF_PLANES], common::Time time)
  {
    //		ros::Time lastUpdateTime = ros::Time::now();
    //		while(true)
    //		{
    //			ros::Time newRosTime = ros::Time::now();
    //			double diff = (newRosTime.toSec() - lastUpdateTime.toSec()) - (1/RVIZPublishRate);
    //			if(diff > 0.0001)
    //			{
    ros::Time t(time.sec, time.nsec);
    RVIZ_Publisher(rangesArray, t);
    //				lastUpdateTime = newRosTime;
    // TF publish
    TF_Broadcast(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, model_name, model_name+"_velodyne16", t);
    //			}
    //		}
  }

  void OnUpdate(const common::UpdateInfo &_info)
  {

    #if GAZEBO_MAJOR_VERSION >= 8
      double dgree = fmod(joint->Position(0)*180/M_PI, 360.0);
    #elif GAZEBO_MAJOR_VERSION < 8
      double dgree = fmod(joint->GetAngle(0).Degree(), 360.0);
    #endif



    double diff = dgree - lastDegree;
    if (diff < 0)
      diff = diff + 360.0; //

    if ((diff - angleRes) > 0.000001)
    {
      //double tick = dgree;
      lastDegree = dgree;
      getRanges(dgree);

      boost::thread(&velodyne16::thread_RVIZ, this, rangesArray, _info.simTime);
    }
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
                myRays.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(*sensor_it));
              #elif CPUvsGPU == 2
                std::cout << " casting in to GPU ray sensor " << senCount << std::endl;               
                myRays.push_back(std::dynamic_pointer_cast<sensors::GpuRaySensor>(*sensor_it));
              #endif
              senCount++;
            }
        }
    
    if (senCount != NUM_OF_RAY_SENSORS )
    ROS_WARN("velodyne: NUM_OF_RAY_SENSORS do not equal the actual number of ray sesors in the sdf ");
  }

  //getRanges
  void getRanges(double dgree)
  {
    // 	std::vector<std::vector<double>> ranges;
    double tick = dgree * (ANGULAR_STEPS / 360);

    for (int i = 0; i < NUM_OF_RAY_SENSORS; i++)
      {
        std::vector<double> newVector;
        //ranges.push_back(newVector);

        myRays[i]->Ranges(newVector);


        if (newVector.size() == NUM_OF_PLANES)
        {
        //int j = fmod(tick + i*(360/NUM_OF_RAY_SENSORS),360);
          int j = fmod(tick + i * (ANGULAR_STEPS / NUM_OF_RAY_SENSORS), ANGULAR_STEPS);

          for (int k = 0; k < NUM_OF_PLANES; k++)
          {
            rangesArray[j][k] = newVector[k];
          }
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
    angleRes = 0.2;  // Default is 0.2deg
    if (_sdf->HasElement("angleRes"))
      angleRes = _sdf->Get<double>("angleRes");
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
    double rotRate = 1.0; // Default is 1HZ velocity
    if (_sdf->HasElement("rotRate"))
      rotRate = _sdf->Get<double>("rotRate");
    else 
      ROS_WARN("velodyne16: there is no 'rotRate' parameter in the SDF seting to defult of 1Hz");
    
      
    //RVIZ Publish Rate, default is 1HZ
    //RVIZPublishRate = 1.0;   // Default is 0Hz
    //if (_sdf->HasElement("RVIZPublishRate"))
    //   RVIZPublishRate = _sdf->Get<double>("RVIZPublishRate");
    //else
    //   ROS_WARN("velodyne16: there is no 'RVIZPublishRate' parameter in the SDF seting to defult of 1Hz");
         

    // sets tte rotation
    double rotVel = (rotRate / NUM_OF_RAY_SENSORS) * 2 * M_PI; 
    this->joint = _model->GetJoint("velodyne16::velodyne16_joint");    
    this->pid = common::PID(0.1, 0, 0); // Setup a P-controller, with a gain of 0.1.       
    this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid); // Apply the P-controller to the joint.
    this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), rotVel);    // Set the joint's target velocity. 

    //set the topic
    std::string topic_name = model_name+"/velodyne16";
    _pointCloud_pub = _nodeHandle.advertise<sensor_msgs::PointCloud>(topic_name, 10);

    //init the ray sensors
    initSensors();

    lastDegree = -1;
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&velodyne16::OnUpdate, this, _1));

  }

  void RVIZ_Publisher(double rangesArray[][NUM_OF_PLANES], ros::Time time)
  {
      sensor_msgs::PointCloud points;
      for (int tick = 0; tick < ANGULAR_STEPS; tick++)
      {
        for (int j = 0; j < NUM_OF_PLANES; j++)
        {
          if (rangesArray[tick][j] > 69)
            continue;
          geometry_msgs::Point32 point;
          double yaw_ang = tick * (2 * M_PI / ANGULAR_STEPS); //* (3.14159 /180);
          double pitch_ang = (verticalAngelMin + j * verticalAngleResolutionFromSDF) * (M_PI / 180);
          point.x = rangesArray[tick][j] * cos(pitch_ang) * cos(yaw_ang);
          point.y = rangesArray[tick][j] * cos(pitch_ang) * sin(yaw_ang);
          point.z = rangesArray[tick][j] * sin(pitch_ang);
          //points.points.insert(points.points.begin(), point);
          points.points.push_back(point);
        }
      }
      points.header.stamp = time; //ros::Time();
      points.header.frame_id = parent_name+"_velodyne16"; 

      _pointCloud_pub.publish(points);
  }



  physics::ModelPtr model;
  physics::JointPtr joint;
  std::string model_name;
  std::string parent_name;
  common::PID pid;



  #if CPUvsGPU == 1
    vector<gazebo::sensors::RaySensorPtr> myRays;  
  #elif CPUvsGPU == 2
    vector<gazebo::sensors::GpuRaySensorPtr> myRays;
  #endif


  event::ConnectionPtr _updateConnection; // Pointer to the update event connection
  common::Time sim_Time;

  double lastDegree;
  double angleRes;
  double rangesArray[ANGULAR_STEPS][NUM_OF_PLANES];
  //   double RVIZPublishRate;
  double verticalAngleResolutionFromSDF;
  double VerticalAngelResolutionReal[NUM_OF_RAY_SENSORS];
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
