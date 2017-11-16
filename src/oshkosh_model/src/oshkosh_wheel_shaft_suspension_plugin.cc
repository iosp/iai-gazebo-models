// Written By : Daniel
#include <stdlib.h>
#include <stdio.h>
// Gazebo Libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Angle.hh>


// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <oshkosh_model/oshkosh_wheels_shaft_dumpingConfig.h>

#include <boost/bind.hpp> // Boost Bind


namespace gazebo
{
 
class oshkoshWheelShaftSuspensionPlugin : public ModelPlugin
{
  ///  Constructor
public:
    oshkoshWheelShaftSuspensionPlugin() {}

  /// The load function is called by Gazebo when the plugin is inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
  {
    std::cout << "My major GAZEBO VER = [" << GAZEBO_MAJOR_VERSION << "]" << std::endl;
    this->model = _model;
    this->model_name = model->GetName(); 
    

    this->shaft_name = "";       
    if (_sdf->HasElement("shaft_name")) 
       this->shaft_name = _sdf->Get<std::string>("shaft_name");   
    else
       ROS_WARN("canot fined shaft_name");


    // Test for whether the model is nested and a prefix is needed to find the joint
    std::string pref = this->model_name + "::" +this->shaft_name;
    std::cout << " Loding Plugin for Dumping in " << pref << std::endl;
    

    if( ! this->model->GetJoint(pref+"::torsion_spring") )
        std::cout << " canot fined " << pref+"::torsion_spring" << std::endl;

    if( ! this->model->GetJoint(pref+"::linear_spring") )
        std::cout << " canot fined " << pref+"::linear_spring" << std::endl;

         
    // Store the pointers to the joints
    this->torsion_spring_joint = this->model->GetJoint(pref+"::torsion_spring");
    this->linear_spring_joint = this->model->GetJoint(pref+"::linear_spring");
        
    this->Ros_nh = new ros::NodeHandle(this->model_name + "_" + this->shaft_name + "_PluginNode");

    
    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&oshkoshWheelShaftSuspensionPlugin::OnUpdate, this, _1));
    

    std::cout << "Setting up dynamic config" << std::endl;
    this->model_reconfiguration_server = new dynamic_reconfigure::Server<oshkosh_model::oshkosh_wheels_shaft_dumpingConfig>(*(this->Ros_nh));
    this->model_reconfiguration_server->setCallback(boost::bind(&oshkoshWheelShaftSuspensionPlugin::dynamic_Reconfiguration_callback, this, _1, _2));
    std::cout << "dynamic configuration is set up" << std::endl;


    std::cout << "Oshkosh Wheel Shaft Suspension Plugin Loaded" << std::endl;
  }

public:
   void dynamic_Reconfiguration_callback(oshkosh_model::oshkosh_wheels_shaft_dumpingConfig &config, uint32_t level)
  {
    this->linear_spring = config.Spring_back;
    this->linear_damping = config.Damper_back;
    this->linear_target = config.Target_back;

    this->torsion_spring = config.Spring_torsion_back;
    this->torsion_damping = config.Damper_torsion_back;
    this->torsion_target = config.Target_torsion_back;
  }


public:
  // Called by the world update start event, This function is the event that will be called every update  
  void OnUpdate(const common::UpdateInfo &simInfo) // we are not using the pointer to the info so its commanted as an option
  {
    Suspension(torsion_spring_joint,torsion_spring,torsion_damping,torsion_target);
    Suspension(linear_spring_joint,linear_spring,linear_damping,linear_target);
  }


  //The function to control the Suspension of the Vehicle
  void Suspension(physics::JointPtr Suspension_joint,double spring,double dunmping, double target)
  { 
    ignition::math::Angle AnglePose =  Suspension_joint->Position(0);  
    double Pose = AnglePose.Radian();
    double Vel = Suspension_joint->GetVelocity(0);

    double SpringForce = (target - Pose) * spring  -  Vel * dunmping;

    Suspension_joint->SetForce(0, SpringForce);
  }

  
  // Defining private Pointer to model
  physics::ModelPtr model;

  std::string model_name;
  std::string shaft_name;

  // Defining private Pointer to joints
  physics::JointPtr torsion_spring_joint;
  physics::JointPtr linear_spring_joint;

  
  // Defining private Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Defining private Ros Node Handle
  ros::NodeHandle *Ros_nh;


  //Dynamic Configuration Definitions
dynamic_reconfigure::Server<oshkosh_model::oshkosh_wheels_shaft_dumpingConfig> *model_reconfiguration_server;
double linear_spring, linear_damping, linear_target ;   
double torsion_spring, torsion_damping, torsion_target;

};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(oshkoshWheelShaftSuspensionPlugin)
}
