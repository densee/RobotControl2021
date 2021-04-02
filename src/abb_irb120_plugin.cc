/*
 * ABB code
 * master : BKCho
 * First develop : HSKim
 * Second develop : Yunho Han
 * Update date : 2020.04.09
 */

// Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

// Header file for Gazebo and Ros
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

// Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.


#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace gazebo
{

    class ABB_IRB120 : public ModelPlugin
    {
        //Link variable for Gazebo
        physics::LinkPtr BASE;
        physics::LinkPtr LINK1;
        physics::LinkPtr LINK2;
        physics::LinkPtr LINK3;
        physics::LinkPtr LINK4;
        physics::LinkPtr LINK5;
        physics::LinkPtr LINK6;

        //Joint variable for Gazebo
        physics::JointPtr LINK1_JOINT;
        physics::JointPtr LINK2_JOINT;
        physics::JointPtr LINK3_JOINT;
        physics::JointPtr LINK4_JOINT;
        physics::JointPtr LINK5_JOINT;
        physics::JointPtr LINK6_JOINT;
        physics::ModelPtr model;

        //PID gain variable for joint
        common::PID pid1;
        common::PID pid2;
        common::PID pid3;
        common::PID pid4;
        common::PID pid5;
        common::PID pid6;

        // Target angle of joint
        VectorXd tar_deg = VectorXd::Zero(7);
        VectorXd m_deg = VectorXd::Zero(6);
        // Angle error of joint
        VectorXd angle_err = VectorXd::Zero(7);

        //setting for getting <dt>(=derivative time) 
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //For model load
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();


    };
    GZ_REGISTER_MODEL_PLUGIN(ABB_IRB120);
}

void gazebo::ABB_IRB120::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{

    // model = link + joint +sensor
    this->model = _model;


    //Load Link Part from model.sdf 
    this->BASE = this->model->GetLink("BASE");
    this->LINK1 = this->model->GetLink("LINK1");
    this->LINK2 = this->model->GetLink("LINK2");
    this->LINK3 = this->model->GetLink("LINK3");
    this->LINK4 = this->model->GetLink("LINK4");
    this->LINK5 = this->model->GetLink("LINK5");
    this->LINK6 = this->model->GetLink("LINK6");

    //Load Joint Part from model.sdf 
    this->LINK1_JOINT = this->model->GetJoint("LINK1_JOINT");
    this->LINK2_JOINT = this->model->GetJoint("LINK2_JOINT");
    this->LINK3_JOINT = this->model->GetJoint("LINK3_JOINT");
    this->LINK4_JOINT = this->model->GetJoint("LINK4_JOINT");
    this->LINK5_JOINT = this->model->GetJoint("LINK5_JOINT");
    this->LINK6_JOINT = this->model->GetJoint("LINK6_JOINT");

    //JOINT PID GAIN
    this->pid1.Init(90, 0, 55, 200, -200, 1000, -1000);
    this->pid2.Init(3000, 0, 60, 200, -200, 1000, -1000);
    this->pid3.Init(3000, 0, 50, 200, -200, 1000, -1000);
    this->pid4.Init(100, 0, 50, 200, -200, 1000, -1000);
    this->pid5.Init(1000, 0, 40, 200, -200, 1000, -1000);
    this->pid6.Init(100, 0.1, 40, 200, -200, 1000, -1000);

    //Time initialize
    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ABB_IRB120::UpdateAlgorithm, this)); // Execute Gazebo Algorithm

}

void gazebo::ABB_IRB120::UpdateAlgorithm()
{ //* Writing realtime code here!!

    //* Calculate time  
    common::Time current_time = this->model->GetWorld()->GetSimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;

    //* Angle error
    tar_deg(1) = 10;
    tar_deg(2) = 20;
    tar_deg(3) = 30;
    tar_deg(4) = 40;
    tar_deg(5) = 50;
    tar_deg(6) = 60;
    
    m_deg(0) = this->LINK1_JOINT->GetAngle(2).Radian();
    m_deg(1) = this->LINK2_JOINT->GetAngle(1).Radian();
    m_deg(2) = this->LINK3_JOINT->GetAngle(1).Radian();
    m_deg(3) = this->LINK4_JOINT->GetAngle(0).Radian();        
    m_deg(4) = this->LINK5_JOINT->GetAngle(1).Radian();
    m_deg(5) = this->LINK6_JOINT->GetAngle(0).Radian();
            

    angle_err[1] = m_deg(0) - (tar_deg(1) * D2R);
    angle_err[2] = m_deg(1) - (tar_deg(2) * D2R);
    angle_err[3] = m_deg(2) - (tar_deg(3) * D2R);
    angle_err[4] = m_deg(3) - (tar_deg(4) * D2R);
    angle_err[5] = m_deg(4) - (tar_deg(5) * D2R);
    angle_err[6] = m_deg(5) - (tar_deg(6) * D2R);

    //* Control Law
    this->pid1.Update(angle_err[1], dt);
    this->pid2.Update(angle_err[2], dt);
    this->pid3.Update(angle_err[3], dt);
    this->pid4.Update(angle_err[4], dt);
    this->pid5.Update(angle_err[5], dt);
    this->pid6.Update(angle_err[6], dt);


    //* Apply torque to joint
    this->LINK1_JOINT->SetForce(2, this->pid1.GetCmd());
    this->LINK2_JOINT->SetForce(1, this->pid2.GetCmd());
    this->LINK3_JOINT->SetForce(1, this->pid3.GetCmd());
    this->LINK4_JOINT->SetForce(0, this->pid4.GetCmd());
    this->LINK5_JOINT->SetForce(1, this->pid5.GetCmd());
    this->LINK6_JOINT->SetForce(0, this->pid6.GetCmd());

    //*setting for getting dt
    this->last_update_time = current_time;

}
