#ifndef _velodyne16_PLUGIN_HH_
#define _velodyne16_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define CPUvsGPU 2 // CPU = 1 , GPU = 2

class VLPCommunication; // forward declaration

static const int MAX_NUM_OF_RAY_SENSORS = 50; // 32 - CPU, 16 - GPU 
static const int NUM_OF_PLANES = 16;

namespace gazebo
{
/// \brief A plugin to control a velodyne16 sensor.
class velodyne16 : public ModelPlugin {
public:
    velodyne16() = default;
    ~velodyne16();

    /// \brief The load function is called by Gazebo when the plugin is inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo &_info);

private:
    void initSensors();

    void SetVLPData(const ignition::math::Angle& angle);

    common::Time getRanges();

    void thread_RVIZ(double rangesArray[][NUM_OF_PLANES], ignition::math::Angle sensorAngle ,common::Time time, float test);

    void RVIZ_Publisher(double rangesArray[][NUM_OF_PLANES], ros::Time time, ignition::math::Angle sensorAngle, float test);

    void TF_Broadcast(double x, double y, double z, double Roll, double Pitch, double Yaw,
         const std::string& frame_id, const std::string& child_frame_id, ros::Time t) const;
    
    tf::Transform transformBuilder(float x, float y, float z, float Roll, float Pitch, float Yaw) const;

    bool GetValueFromSdfFile(sdf::ElementPtr _sdf, const std::string& elemName, double& outValue, double defaultValue) const;

    void threadInput();

    double m_angleRes; 
    double m_rotRate;
    double m_verticalAngleResolution;
    double m_verticalAngelMin;

    physics::ModelPtr m_model;
    physics::JointPtr m_joint;

    std::string m_model_name;
    std::string m_parent_name;

    #if CPUvsGPU == 1
        std::vector<gazebo::sensors::RaySensorPtr> m_raySensors;  
    #elif CPUvsGPU == 2
        std::vector<gazebo::sensors::GpuRaySensorPtr> m_raySensors;
    #endif

    double m_rangesArray[MAX_NUM_OF_RAY_SENSORS][NUM_OF_PLANES];

    int m_numOfRaySensors;

    VLPCommunication* m_vlp;

    common::Time m_lastUpdateTime;

    event::ConnectionPtr m_updateConnection; // Pointer to the update event connection


    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_pointCloud_pub;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(velodyne16)
}
#endif
