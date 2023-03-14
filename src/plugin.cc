#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
 
namespace gazebo                          
{
class PIDJoints : public ModelPlugin             
  {
    double dt;
    physics::LinkPtr body;                                   
    physics::JointPtr right_wheel_joint;                     
    physics::JointPtr left_wheel_joint;                     
    physics::ModelPtr model;
    
    common::Time last_update_time;
    event::ConnectionPtr update_connection_;
    
    
   public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
       {
      this->model = _model;
    /*this->body = this->model->GetLink("body");
      this->right_wheel_joint = this->model->GetJoint(“R_wheel_joint");
      this->left_wheel_joint = this->model->GetJoint(“L_wheel_joint");*/
      this->body = this->model->GetLink("KUDOS_body");
       //this->right_wheel_joint = this->model->GetJoint("KUDOS_wheel_right_shaft");
      // this->left_wheel_joint = this->model->GetJoint("KUDOS_wheel_left_shaft");
       this->right_wheel_joint = this->model->GetJoint("right_wheel_joint");
       this->left_wheel_joint = this->model->GetJoint("left_wheel_joint");
    /*this->last_update_time = this->model->GetWorld()->GetSimTime();*/
      this->last_update_time = this->model->GetWorld()->SimTime();
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PIDJoints::UpdatePID, this));
      
    }
 
 
    void UpdatePID()
    {
     /*common::Time current_time = this->model->GetWorld()->GetSimTime();*/
      common::Time current_time = this->model->GetWorld()->SimTime();
      dt = current_time.Double() - this->last_update_time.Double();
      
      this->right_wheel_joint->SetForce(1, 1);   //setForce(axis,Force value)
      this->left_wheel_joint->SetForce(1, 1);
    
      this->last_update_time = current_time;
      
    }
  };
    GZ_REGISTER_MODEL_PLUGIN(PIDJoints);
}

