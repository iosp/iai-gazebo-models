#ifndef _velodyne_16_PLUGIN_HH_
#define _velodyne_16_PLUGIN_HH_

#include <stdlib.h>
#include <stdio.h>

#include <gazebo/gazebo.hh>
#include "gazebo/sensors/sensors.hh"

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"

#include "gazebo/sensors/RaySensor.hh"
//#include "gazebo/sensors/GpuRaySensor.hh"

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

#define NUM_OF_PLANES 16
#define NUM_OF_RAY_SENSORS 36
#define ANGULAR_STEPS 1800 // 360 * 5
using namespace std;

namespace gazebo
{
/// \brief A plugin to control a velodyne_16 sensor.
class velodyne_16 : public ModelPlugin
{
  /// \brief Constructor
public:
  velodyne_16() {}

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
    TF_Broadcast(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, model_name, model_name+"_velodyne_16", t);
    //			}
    //		}
  }

  void OnUpdate(const common::UpdateInfo &_info)
  {
    double dgree = fmod(joint->GetAngle(0).Degree(), 360.0);

    double diff = dgree - lastDegree;
    if (diff < 0)
      diff = diff + 360.0; //

    if ((diff - angleRes) > 0.000001)
    {
      //double tick = dgree;
      lastDegree = dgree;
      getRanges(dgree);

      boost::thread(&velodyne_16::thread_RVIZ, this, rangesArray, _info.simTime);
    }
  }

  void initSensors(string modelName)
  {
    for (int i = 0; i < NUM_OF_RAY_SENSORS; i++)
    {
      string nameOfRay = "velodyne_16_ray_" + std::to_string(i);
#if GAZEBO_MAJOR_VERSION > 6
      myRays.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(sensors::SensorManager::Instance()->GetSensor(nameOfRay)));
      //myRays.push_back(std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensors::SensorManager::Instance()->GetSensor(nameOfRay)));
      
#elif GAZEBO_MAJOR_VERSION < 7
      myRays.push_back(boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::SensorManager::Instance()->GetSensor(nameOfRay)));
      //myRays.push_back(boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensors::SensorManager::Instance()->GetSensor(nameOfRay)));
#endif
      if (!myRays[i])
      {
        std::string error = "velodyne_16 Sensor Model \"" + modelName + "\" failed to locate his sub-sensor.\n(Do the names in the .sdf match the names in the .cpp?)\n";
        gzthrow(error);
        return;
      }

#if GAZEBO_MAJOR_VERSION > 6
      VerticalAngelResolutionReal[i] = (myRays[i]->AngleResolution() * 180 / M_PI);
#elif GAZEBO_MAJOR_VERSION < 7
      VerticalAngelResolutionReal[i] = (myRays[i]->GetAngleResolution() * 180 / M_PI);
#endif
      if ((int)(VerticalAngelResolutionReal[i] * 100) != (int)(verticalAngleResolutionFromSDF * 100))
	ROS_WARN("velodyne_16: the parameter 'verticalAngleResolution' is not equal to GetAngleResolution() in Ray_%d", i);
    }
  }

  //getRanges
  void getRanges(double dgree)
  {
    //    	std::vector<std::vector<double>> ranges;

    double tick = dgree * (ANGULAR_STEPS / 360);

    for (int i = 0; i < NUM_OF_RAY_SENSORS; i++)
    {
      std::vector<double> newVector;
//    		ranges.push_back(newVector);

#if GAZEBO_MAJOR_VERSION > 6
      myRays[i]->Ranges(newVector);
#elif GAZEBO_MAJOR_VERSION <= 6
      myRays[i]->GetRanges(newVector);
#endif

      if (newVector.size() == NUM_OF_PLANES)
      {
	// int j = fmod(tick + i*(360/NUM_OF_RAY_SENSORS),360);
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
      std::cerr << "Invalid joint count, Velodyne_16 plugin not loaded\n";
      return;
    }

    // Store the model pointer for convenience.
    this->model = _model;

    this->model_name = model->GetName(); 
    std::cout << "Loading " << model_name << " Plugin" << std::endl;
    this->parent_name = model->GetParentModel()->GetName();
    std::cout << "parent_name = " << parent_name << std::endl;
    

    // Get the joint.
    std::string jointName = "velodyne_16::velodyne_16_joint";
    if (_sdf->HasElement("jointName"))
    {
      _sdf->GetElement("jointName")->GetValue()->Get<std::string>(jointName);
    }
    this->joint = _model->GetJoint(jointName);

    // Setup a P-controller, with a gain of 0.1.
    this->pid = common::PID(0.1, 0, 0);

    // Apply the P-controller to the joint.
    this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), 100.0);

    // set the angle resolution, default is 0.2 deg
    angleRes = 0.2;
    if (_sdf->HasElement("angleRes"))
      angleRes = _sdf->Get<double>("angleRes");

    //RVIZ Publish Rate, default is 1HZ
    //RVIZPublishRate = 1.0;
    //if (_sdf->HasElement("RVIZPublishRate"))
    //	  RVIZPublishRate = _sdf->Get<double>("RVIZPublishRate");

    // get the 'verticalAngleResolutionFromSDF' parameter
    // using for validate the user, compare this veriable and real vertical angle (VerticalAngelResolutionReal[])
    verticalAngleResolutionFromSDF = 0;
    if (_sdf->HasElement("verticalAngleResolution"))
      verticalAngleResolutionFromSDF = _sdf->Get<double>("verticalAngleResolution");
    else
      ROS_WARN("velodyne_16: there is no 'verticalAngleResolution' parameter in the SDF (validation parameter)");

    
    if (_sdf->HasElement("verticalAngelMin"))
      verticalAngelMin = _sdf->Get<double>("verticalAngelMin");
    else
      ROS_WARN("velodyne_16: there is no 'verticalAngelMin' parameter in the SDF");


    // Default to 1HZ velocity
    rate = 1.0;
    // Check that the velocity element exists, then read the value
    if (_sdf->HasElement("rate"))
    {
      rate = _sdf->Get<double>("rate");
    }

    //set the velocity
    velocity = (rate / NUM_OF_RAY_SENSORS) * 2 * M_PI; //depend on rate

    // Set the joint's target velocity. This target velocity is just
    // for demonstration purposes.
    this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);

    //set the topic
    std::string topic_name = model_name+"/velodyne_16";
    _pointCloud_pub = _nodeHandle.advertise<sensor_msgs::PointCloud>(topic_name, 10);

    //init sensors
    initSensors(_model->GetName());

    lastDegree = -1;
    //_threadRVIZ=boost::thread(&velodyne_16::thread_RVIZ,this);
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&velodyne_16::OnUpdate, this, _1));

    if (_sdf->HasElement("tf_x") && _sdf->HasElement("tf_y") && _sdf->HasElement("tf_z") && _sdf->HasElement("tf_roll") && _sdf->HasElement("tf_pitch") && _sdf->HasElement("tf_yaw"))
    {
      tf_x = _sdf->Get<double>("tf_x");
      tf_y = _sdf->Get<double>("tf_y");
      tf_z = _sdf->Get<double>("tf_z");
      tf_roll = _sdf->Get<double>("tf_roll");
      tf_pitch = _sdf->Get<double>("tf_pitch");
      tf_yaw = _sdf->Get<double>("tf_yaw");
    }
    else
    {
      tf_x = 0;
      tf_y = 0;
      tf_z = 0;
      tf_roll = 0;
      tf_pitch = 0;
      tf_yaw = 0;

      ROS_DEBUG("velodyne: ERROR in reading tf_x, tf_y, tf_z, tf_r, tf_p, tf_y parameters");
    }
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
      points.header.frame_id = parent_name+"_velodyne_16"; 

      _pointCloud_pub.publish(points);
  }

  physics::ModelPtr model;
  physics::JointPtr joint;
  std::string model_name;
  std::string parent_name;
  common::PID pid;

  vector<gazebo::sensors::RaySensorPtr> myRays;  
  //vector<gazebo::sensors::GpuRaySensorPtr> myRays;

  event::ConnectionPtr _updateConnection; // Pointer to the update event connection
  common::Time sim_Time;

  double lastDegree;
  double angleRes;
  double velocity;
  double rate;
  double rangesArray[ANGULAR_STEPS][NUM_OF_PLANES];
  //   double RVIZPublishRate;
  double verticalAngleResolutionFromSDF;
  double VerticalAngelResolutionReal[NUM_OF_RAY_SENSORS];
  double verticalAngelMin;
  ros::Publisher _pointCloud_pub;
  ros::NodeHandle _nodeHandle;

  boost::thread _threadRVIZ;

  ros::NodeHandle n;
  tf::Transform _laser_TF_point_of_origin;
  double tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(velodyne_16)
}
#endif
