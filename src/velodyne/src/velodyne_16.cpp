#ifndef _velodyne_16_PLUGIN_HH_
#define _velodyne_16_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include "gazebo/sensors/sensors.hh"

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
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
#define NUM_OF_RAY_SENSORS 24
#define ANGULAR_STEPS 1800 // 360 * 5
using namespace std;

namespace gazebo
{
  /// \brief A plugin to control a velodyne_16 sensor.
  class velodyne_16 : public ModelPlugin
  {
    /// \brief Constructor
    public:
	velodyne_16() 
	{
		UDPSocketIsConnected = false;
	}

	tf::Transform transformBuilder(float x,float y,float z,float Roll,float Pitch,float Yaw)
	{
		 tf::Transform transform;
		 transform.setOrigin( tf::Vector3(x, y, z) );

		 tf::Quaternion q;
		 q.setRPY(Roll,Pitch,Yaw);
		 transform.setRotation(q);
		 return(transform);
	}

	void TF_Broadcast(double x, double y, double z, double Roll, double Pitch, double Yaw, std::string frame_id, std::string child_frame_id, ros::Time t)
	{
		 static tf::TransformBroadcaster br;
		 tf::StampedTransform st(transformBuilder(x,y,z,Roll,Pitch,Yaw), t, frame_id, child_frame_id);
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
				TF_Broadcast(tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw, model->GetParentModel()->GetName(), "velodyne_16", t);
//			}
//		}
	}

#include <stdio.h>
#include <stdlib.h>

#define SIZE_OF_CHANNEL_DATA 3
// 3 = 2 bytes of return distance and 1 byte of calibrated reflectivity

#define SIZE_OF_DATA_BLOCK 100
// 100 = 2 * 48 (16 channels * SIZE_OF_CHANNEL_DATA + 2 bytes for flag + 2 bytes for azimuth

#define SIZE_OF_PACKET 1246
// 1246 = 12 * SIZE_OF_DATA_BLOCK + 42 bytes of header + 4 bytes for time stamp + 2 bytes for factory

  void insertReverseShortToBuffer(double num, char* buff)
  {
    short num_sh = num * 500; // 500 = 1000 (for transfer from meters to mm) / 2 (the nearest is 2.0mm);

    memcpy(buff, (char*)&num_sh+1, 1);
    memcpy(buff+1, &num_sh, 1);
  }

  void fillDataBlock(double rangesArray[NUM_OF_PLANES], char* buff, double dgree = -1)
  {
    if(dgree != -1) //even block
    {
        //flag
        memcpy(buff, "\xff", 1);
        buff+=1;
        memcpy(buff, "\xee", 1);
        buff+=1;

        //azimuth
        //insertReverseShortToBuffer(dgree, buff);
        buff+=2;
    }
    for(int i = 0 ; i< NUM_OF_PLANES ; i++)
      insertReverseShortToBuffer(rangesArray[i], buff + (i*SIZE_OF_CHANNEL_DATA));
  }

  void thread_UDP(double rangesArray[][NUM_OF_PLANES], common::Time time, double dgree)
  {
    ros::Time t(time.sec, time.nsec);
    char buff[SIZE_OF_PACKET] = {};
    char* pToBuff = buff;
    pToBuff += 42; // size of header, arnon don't read it

    for(int i = 0 ; i < NUM_OF_RAY_SENSORS ; i++)
      {
        if(i%2 == 0) //even date block - with flag and azimuth
	{
		fillDataBlock(rangesArray[i], pToBuff, dgree);
		pToBuff+=52; // 2 bytes for flag + 2 bytes for azimuth + 48 (16 channels * SIZE_OF_CHANNEL_DATA) bytes for the data
	}
	else
	{
		fillDataBlock(rangesArray[i], pToBuff);
		pToBuff+=48; // 48 bytes for the data, without azimuth and flag
	}
      }

    // then we need to fill the time stamp and factory bytes



	int bytes_sent=0;
	while(bytes_sent<SIZE_OF_PACKET && UDPSocketIsConnected)
	{
		int n = sendto(UDPsockfd, buff+bytes_sent, SIZE_OF_PACKET-bytes_sent, 0, (sockaddr *)&UDPserv_addr, sizeof(UDPserv_addr));
		if(n>0)
		{
		   bytes_sent+=n;
		  //ROS_INFO("bytes_sent = %d", bytes_sent);
		}
	   	else if(n<0)
		{
		   if(UDPsockfd>0)
		   {
			cout << "shutdown" << endl;
			shutdown(UDPsockfd,SHUT_RDWR);
			UDPsockfd=-1;
		   }
		   UDPSocketIsConnected=false;
		   ROS_INFO("NOT SEND, errno = %s", strerror(errno));
		   break;
		}
	}


//    std::cout << "start" << std::endl;
//    for(int i = 0 ; i < 1246 ; i++)
//      std::cout << buff[i] << " ";
//    std::cout << "end" << std::endl;

//    std::cout << "start" << std::endl;

//    char buff[2];
//    insertNumToBuffer(rangesArray[0][0], buff);

//    char temp2 = buff[1]+buff[0];
//    short val = (short)temp2;
//    float res = (float)val/1000.0 * 2.0;

//    std::cout << "res " << res << ", org " << rangesArray[0][0]<< std::endl;
//    std::cout << "end" << std::endl << std::endl;

    
  }

    bool initUDPConnection()
    {
        UDPsockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(UDPsockfd<0)
        {
        	ROS_ERROR("Velodyne_16, function: initConnection(), Velodyne_16 socket not connected\n");
      	  	return false;
        }
	memset((char *) & UDPserv_addr, 0,sizeof(UDPserv_addr));
	UDPserv_addr.sin_family=AF_INET;
	UDPserv_addr.sin_port=htons(5030);
 	if(inet_aton("192.168.1.77", &UDPserv_addr.sin_addr)==0)
 	{
 		ROS_ERROR("Velodyne_16, function: initConnection(), Velodyne_16 inet_aton() failed\n");
  	        return false;
  	}
 	UDPSocketIsConnected = true;
   	ROS_INFO("Velodyne_16 initialize of UDP connection success");
 	return true;
    }


     void OnUpdate(const common::UpdateInfo & _info)
     {
    	double dgree = fmod(joint->GetAngle(0).Degree(),360.0);

		double diff = dgree - lastDegree;
		if (diff < 0)
			diff = diff+360.0; //

		if ((diff - angleRes) > 0.000001)
		{
			//double tick = dgree;
			lastDegree = dgree;
			getRanges(dgree);

			boost::thread(&velodyne_16::thread_RVIZ,this, rangesArray, _info.simTime);
		      	boost::thread(&velodyne_16::thread_UDP, this, rangesArray, _info.simTime, dgree);		}
     }

     void initSensors(string modelName)
     {
    	 for(int i = 0 ; i < NUM_OF_RAY_SENSORS ; i++)
    	 {
    		 string nameOfRay = "velodyne_16_ray_" + std::to_string(i);
#if GAZEBO_MAJOR_VERSION > 6
      myRays.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(sensors::SensorManager::Instance()->GetSensor(nameOfRay)));
#elif GAZEBO_MAJOR_VERSION < 7
      myRays.push_back(boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::SensorManager::Instance()->GetSensor(nameOfRay)));
#endif
    		 if(!myRays[i])
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
    	   	 if((int)(VerticalAngelResolutionReal[i] * 100) != (int)(verticalAngleResolutionFromSDF * 100))
    	   		ROS_WARN("velodyne_16: the parameter 'verticalAngleResolution' is not equal to GetAngleResolution() in Ray_%d", i);
    	 }
     }

    //getRanges
    void getRanges(double dgree)
    {
//    	std::vector<std::vector<double>> ranges;

    	double tick = dgree * (1/angleRes); //(ANGULAR _ STEPS/360);

    	for(int i = 0 ; i < NUM_OF_RAY_SENSORS ; i++)
    	{
    		std::vector<double> newVector;
#if GAZEBO_MAJOR_VERSION > 6
      myRays[i]->Ranges(newVector);
#elif GAZEBO_MAJOR_VERSION <= 6
      myRays[i]->GetRanges(newVector);
#endif

    		if(newVector.size() == NUM_OF_PLANES)
    		{
    			int j = fmod(tick + i/**(ANGULAR _ STEPS/NUM_OF_RAY_SENSORS)*/,ANGULAR_STEPS);


    			for(int k = 0 ; k < NUM_OF_PLANES ; k++)
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
	    std::cerr << "Invalid joint count, velodyne_16 plugin not loaded\n";
	    return;
	  }

	  // Store the model pointer for convenience.
	  this->model = _model;

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

	  // Default to 1HZ velocity
	  rate = 1.0;
	  // Check that the velocity element exists, then read the value
	  if (_sdf->HasElement("rate"))
	  {
		  rate = _sdf->Get<double>("rate");
	  }

	  //set the velocity
	  velocity =  (rate / NUM_OF_RAY_SENSORS) * 2 * M_PI; //depend on rate

	  // Set the joint's target velocity. This target velocity is just
   	  // for demonstration purposes.
   	  this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);

   	  //set the topic
	  std::string topic = "/SENSORS/velodyne_16";
	  if (_sdf->HasElement("topic"))
	  {
	  	   _sdf->GetElement("topic")->GetValue()->Get<std::string>(topic);
	  }
   	  _pointCloud_pub = _nodeHandle.advertise<sensor_msgs::PointCloud>(topic, 10);

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

		  ROS_DEBUG("velodyne_16: ERROR in reading tf_x, tf_y, tf_z, tf_r, tf_p, tf_y parameters");
	  }
	  initUDPConnection();
	}

	void RVIZ_Publisher(double rangesArray[][NUM_OF_PLANES], ros::Time time)
	{
		sensor_msgs::PointCloud points;
		for(int tick = 0 ; tick < ANGULAR_STEPS ; tick++)
		{
			for(int j = 0 ; j < NUM_OF_PLANES ; j++)
			{
				if(rangesArray[tick][j] > 69 )
					continue;

				geometry_msgs::Point32 point;
				double yaw_ang = tick * (2* M_PI/ANGULAR_STEPS); //* (3.14159 /180);
				double pitch_ang((j-NUM_OF_PLANES/2) * verticalAngleResolutionFromSDF * (M_PI /180));
				point.x = rangesArray[tick][j] * cos(pitch_ang) * cos(yaw_ang);
				point.y = rangesArray[tick][j] * cos(pitch_ang) * sin(yaw_ang);
				point.z	= rangesArray[tick][j] * sin (pitch_ang);
				//points.points.insert(points.points.begin(), point);
				points.points.push_back(point);
			}
		}
		points.header.stamp = time;//ros::Time();
		points.header.frame_id = "velodyne_16";

		_pointCloud_pub.publish(points);
	}


    physics::ModelPtr model;
    physics::JointPtr joint;
    common::PID pid;

    vector<gazebo::sensors::RaySensorPtr> myRays;

    event::ConnectionPtr 		_updateConnection; // Pointer to the update event connection
    common::Time			sim_Time;

    double lastDegree;
	double angleRes;
	double velocity;
	double rate;
    double rangesArray[ANGULAR_STEPS][NUM_OF_PLANES];
 //   double RVIZPublishRate;
    double verticalAngleResolutionFromSDF;
    double VerticalAngelResolutionReal[NUM_OF_RAY_SENSORS];
    ros::Publisher _pointCloud_pub;
    ros::NodeHandle		_nodeHandle;

    boost::thread _threadRVIZ;

    ros::NodeHandle n;
    tf::Transform _laser_TF_point_of_origin;
    double tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw;

    int UDPsockfd;
    bool UDPSocketIsConnected;
    int UDPPortno;
    struct sockaddr_in UDPserv_addr;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(velodyne_16)
}
#endif
