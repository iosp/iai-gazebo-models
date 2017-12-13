
#include "VLPCommunication.h"
#include "Logger.h"
#include "velodyne16.h"
#include "Utilities.h"

#include "gazebo/sensors/sensors.hh"
#include "sensor_msgs/PointCloud.h"

#include <boost/range/irange.hpp> // boost::irange
#include <algorithm> // std::transform

using namespace gazebo;

static const double DEG_TO_RAD = ( M_PI / 180.0 );
static const double RAD_TO_DEG = ( 1 / DEG_TO_RAD );
static const double FIRING_SEQUENCE = 55.296;
double DEF_VAL = 10;

// #include <iostream>
// void velodyne16::threadInput() {
//     // while (true) {
//     //     std::cin >> DEF_VAL;
//     //     sleep(1);
//     // }
// }

velodyne16::~velodyne16() {
     delete m_vlp;
}

void velodyne16::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    // Safety check
    if (_model->GetJointCount() == 0) {
        std::cerr <<this->m_model_name << ": Invalid joint count, velodyne16 plugin not loaded\n";
        return;
    }

    // Store the model pointer for convenience.
    this->m_model = _model;

    this->m_model_name = _model->GetName(); 
    std::cout << "Loading " << m_model_name << " Plugin" << std::endl;
    this->m_parent_name = _model->GetParentModel()->GetName();
    std::cout << "parent_name = " << m_parent_name << std::endl;

    // set the angle resolution, default is 0.2 deg
    if (!GetValueFromSdfFile(_sdf, "angleRes", this->m_angleRes, 0.2)) {
        ROS_WARN("velodyne16: there is no 'angleRes' parameter in the SDF seting to defult of 0.2deg");
    }

    // geting the verticalAngleResolution from sdf
    if (!GetValueFromSdfFile(_sdf, "verticalAngleResolution", this->m_verticalAngleResolution, 0)) {
        ROS_WARN("velodyne16: there is no 'verticalAngleResolution' parameter in the SDF (validation parameter)");
    }

    // geting the verticalAngelMin from sdf
    if (!GetValueFromSdfFile(_sdf, "verticalAngelMin", this->m_verticalAngelMin, 0)) {
        ROS_WARN("velodyne16: there is no 'verticalAngelMin' parameter in the SDF seting to defult of 0");
    }

    // geting the rotRate from sdf
    if (!GetValueFromSdfFile(_sdf, "rotRate", this->m_rotRate, 1.0)) {
        ROS_WARN("velodyne16: there is no 'rotRate' parameter in the SDF seting to defult of 1Hz");
    }

    // Test for whether the model is nested and a prefix is needed to find the joint
    std::string pref = _model->GetName();
    if( ! this->m_model->GetJoint(pref+"::velodyne16_joint") ) {
        pref = "velodyne16";
    }

    this->m_joint = this->m_model->GetJoint(pref + "::velodyne16_joint");    

    //set the topic
    std::string topic_name = m_model_name + "/velodyne16";
    m_pointCloud_pub = m_nodeHandle.advertise<sensor_msgs::PointCloud>(topic_name, 10);

    //init the ray sensors
    initSensors();

    // create instance of VLP communication
    VLPCommunication::VLPConfig conf("192.168.1.77", "2368", VLPCommunication::_RES02_, VLPCommunication::_VEL16_);
    m_vlp = new VLPCommunication(conf);
    m_vlp->Run();

    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&velodyne16::OnUpdate, this, _1));
    // boost::thread(&velodyne16::threadInput, this);
}

bool velodyne16::GetValueFromSdfFile(sdf::ElementPtr _sdf, const std::string& elemName, double& outValue, double defaultValue) const {
    if (_sdf->HasElement(elemName)) {
        outValue = _sdf->Get<double>(elemName);
        return true;
    }
    else {
        outValue = defaultValue;
        return false;
    }
}

void velodyne16::initSensors() {
    sensors::Sensor_V v_sensor = gazebo::sensors::SensorManager::Instance()->GetSensors();

    int senCount = 0;
    for (auto sensor_it = v_sensor.begin(); sensor_it != v_sensor.end(); ++sensor_it) {
        std::string p_name = (*sensor_it)->ParentName();
        std::string s_name = this->m_parent_name + "::velodyne16";
        if  (p_name.compare(0, s_name.length(), s_name) == 0 ){
        #if CPUvsGPU == 1
            std::cout << " casting in to CPU ray sensor "  << senCount << " parent_name = " << m_parent_name <<  "   p_name = " << p_name << std::endl; 
            m_raySensors.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(*sensor_it));
        #elif CPUvsGPU == 2
            std::cout << " casting in to GPU ray sensor " << senCount << " parent_name = " << m_parent_name <<  "   p_name = " << p_name << std::endl; 
            m_raySensors.push_back(std::dynamic_pointer_cast<sensors::GpuRaySensor>(*sensor_it));
        #endif
        senCount++;
        }
    }

    this->m_numOfRaySensors = senCount;    
}

void velodyne16::OnUpdate(const common::UpdateInfo &_info) {
    common::Time currentUpdateTime = _info.simTime;  

    double dTime = (currentUpdateTime - this->m_lastUpdateTime).Double();
    
    ignition::math::Angle dAngle =  dTime * 2*M_PI*this->m_rotRate;
    ignition::math::Angle AnglStep =  DEG_TO_RAD*(this->m_angleRes * this->m_numOfRaySensors);

    // if (dAngle.Degree() < AnglStep.Degree() ) {
    //     return;
    // }

    if (dAngle.Degree() > 2*AnglStep.Degree() ) {
        gzerr << this->m_model_name << ":  Step resolution of " << this->m_angleRes << " cannot be reached !! \n"; 
        return; 
    }

    common::Time sensorsUpdateTime = getRanges();
        
    ignition::math::Angle currentJointAngle =  this->m_joint->Position(0);
    ignition::math::Angle jointNextAngle = currentJointAngle +  dAngle; 
    double sensorDelay = currentUpdateTime.Double() - sensorsUpdateTime.Double();

    if ((sensorDelay < 0) || (sensorDelay > 0.007)) {
        this->m_joint->SetPosition(0,jointNextAngle.Radian()); 
        this->m_lastUpdateTime = currentUpdateTime;
        return;
    }
    ignition::math::Angle sensorDelayAngle =  sensorDelay * 2*M_PI*this->m_rotRate;

    ignition::math::Angle AngleCorrectionToSensorDelay = 0;
    if ( CPUvsGPU == 1) { // CPU = 1 , GPU = 2
        AngleCorrectionToSensorDelay =  ( std::floor( sensorDelayAngle.Radian() / AnglStep.Radian() ) ) * AnglStep.Radian(); 
    }
    else if (CPUvsGPU == 2 )  {
        AngleCorrectionToSensorDelay =  ( std::floor( (sensorDelayAngle.Radian() - 0.5*AnglStep.Radian() )/ AnglStep.Radian() ) ) * AnglStep.Radian(); 
    }
    float test = 0; // sensorDelay * 1000;

    boost::thread(&velodyne16::thread_RVIZ, this,  this->m_rangesArray, currentJointAngle - AngleCorrectionToSensorDelay  ,this->m_lastUpdateTime , test );
    //std::cout << " sensorDelay = " << sensorDelay <<  "     AngleCorrectionToSensorDelay = "  << AngleCorrectionToSensorDelay.Degree()  << std::endl;
    if (currentJointAngle - AngleCorrectionToSensorDelay >= 0) {
        SetVLPData(currentJointAngle - AngleCorrectionToSensorDelay);
    }

    this->m_joint->SetPosition(0,jointNextAngle.Radian()); 
    this->m_lastUpdateTime = currentUpdateTime;
}

void velodyne16::SetVLPData(const ignition::math::Angle& angle) {
 
    double timeOffset = 0;
    std::vector<VLPCommunication::VLPData> dataCollection;
    double degAngle = Utilities::dmod(angle.Radian() * RAD_TO_DEG, 360);

    for (auto sensor_i : boost::irange(0, this->m_numOfRaySensors)) {
        for (double offset = 0; offset < m_angleRes; offset += 0.2) {
            // create azimuth
            // double offset = 0;
            double azimuth = angle.Radian() +  (offset * DEG_TO_RAD) + sensor_i * m_angleRes * DEG_TO_RAD;
            azimuth = Utilities::dmod(azimuth * RAD_TO_DEG, 360);
            // create distance and reflectivity channels data
            std::vector<double> ranges(this->m_rangesArray[sensor_i], 
                this->m_rangesArray[sensor_i] + sizeof(this->m_rangesArray[sensor_i]) / sizeof(this->m_rangesArray[sensor_i][0]));
            // std::vector<double> ranges2(ranges.size(), DEF_VAL);
            std::vector<short> reflections(ranges.size());
            VLPCommunication::t_channel_data channelsData;
            channelsData.reserve(ranges.size());
            std::transform(ranges.begin(), ranges.end(), reflections.begin(), std::back_inserter(channelsData),
                        [](double distance, short ref) { return std::make_pair(distance, ref); });
            // create time stamp
            timeOffset +=  FIRING_SEQUENCE;
            boost::posix_time::time_duration td = boost::posix_time::microseconds((this->m_lastUpdateTime.Double() * 1000000) + timeOffset);
            VLPCommunication::VLPData data(azimuth, channelsData, td);
            dataCollection.push_back(data);
        }
    }
    m_vlp->SetData(dataCollection);
}

common::Time velodyne16::getRanges() {
    common::Time sensorDataTime = this->m_raySensors[0]->LastUpdateTime(); 
    double minTime = sensorDataTime.Double();
    double maxTime = sensorDataTime.Double();       

    int unSyncRaysConter = 0;
    for (int sensor_i = 0; sensor_i < this->m_numOfRaySensors; sensor_i++) {
        std::vector<double> rayMeasuresVec;
        this->m_raySensors[sensor_i]->Ranges(rayMeasuresVec);
        
        if (rayMeasuresVec.size() != NUM_OF_PLANES) {
            if ( rayMeasuresVec.size() >= 1)  {// avoiding error prints before the sensors loaded 
                gzerr <<this->m_model_name << ": rayMeasuresVec.size() != NUM_OF_PLANES  \n"; 
            }
            return(sensorDataTime);  
        }
        
        common::Time rayDataTime = this->m_raySensors[sensor_i]->LastUpdateTime();
        
        if ( rayDataTime.Double() - sensorDataTime.Double() > 0.000  ) { 
            std::fill(rayMeasuresVec.begin(), rayMeasuresVec.end(), 0);   // getting rid of the scans that are not in sync with sensor_i = 0 ,  more relevant for the GPU
            unSyncRaysConter++;
            gzerr << this->m_model_name << " : velodyne rayDataTime unsync - throwing mesurment    ( sensor_i = " << sensor_i <<  " rayDataTime = " <<  rayDataTime.Double()  << "   sensorDataTime = "  << sensorDataTime.Double() << " ) \n"; 
        }
        
        for (int plain_i = 0; plain_i < NUM_OF_PLANES; plain_i++) {    
            this->m_rangesArray[sensor_i][plain_i] = rayMeasuresVec[plain_i];
        }
    }

    if (unSyncRaysConter >= 0.9*this->m_numOfRaySensors  ) {
        gzerr << this->m_model_name << ":  The number of un-sync rays (that are thrown) is : " << unSyncRaysConter << "\n" ; 
        return(sensorDataTime);  
    }    
    return(sensorDataTime);
}

void velodyne16::thread_RVIZ(double rangesArray[][NUM_OF_PLANES], ignition::math::Angle sensorAngle ,common::Time time, float test) {
    ros::Time t(time.sec, time.nsec);
    RVIZ_Publisher(rangesArray, t ,sensorAngle, test);
    TF_Broadcast(0.0, 0.0, 0.0, 0.0, 0.0, sensorAngle.Radian(), m_model_name, m_model_name + "_velodyne16", t);
}

void velodyne16::RVIZ_Publisher(double rangesArray[][NUM_OF_PLANES], ros::Time time, ignition::math::Angle sensorAngle, float test) {
    sensor_msgs::PointCloud points;
    for (int sensor_i = 0; sensor_i < this->m_numOfRaySensors; sensor_i++) {
        for (int plan_i = 0; plan_i < NUM_OF_PLANES; plan_i++) {
            geometry_msgs::Point32 point;
            double yaw_ang = sensor_i * this->m_angleRes * DEG_TO_RAD; 
            double pitch_ang = (this->m_verticalAngelMin + plan_i * m_verticalAngleResolution) * DEG_TO_RAD;
            point.x = rangesArray[sensor_i][plan_i]  * cos(pitch_ang) * cos(yaw_ang);
            point.y = rangesArray[sensor_i][plan_i]  * cos(pitch_ang) * sin(yaw_ang);
            point.z = test + rangesArray[sensor_i][plan_i]  * sin(pitch_ang);
            points.points.push_back(point);
        }
    }
    points.header.stamp = time; //ros::Time();
    points.header.frame_id = m_parent_name+"_velodyne16"; 

    m_pointCloud_pub.publish(points);   
}
 
void velodyne16::TF_Broadcast(double x, double y, double z, double Roll, double Pitch, double Yaw,
     const std::string& frame_id, const std::string& child_frame_id, ros::Time t) const {
    static tf::TransformBroadcaster br;
    tf::StampedTransform st(transformBuilder(x, y, z, Roll, Pitch, Yaw), t, frame_id, child_frame_id);
    br.sendTransform(st);
}

tf::Transform velodyne16::transformBuilder(float x, float y, float z, float Roll, float Pitch, float Yaw) const {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));

    tf::Quaternion q;
    q.setRPY(Roll, Pitch, Yaw);
    transform.setRotation(q);
    return (transform);
}