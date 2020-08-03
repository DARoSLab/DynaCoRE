#include "TELLO_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>

#include "common/utils.h"

#include <DynaController/TELLO_Controller/TELLO_interface.hpp>
#include <DynaController/TELLO_Controller/TELLO_DynaCtrl_Definition.h>
#include <RobotSystems/TELLO/TELLO_Definition.h>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>

TELLO_Dyn_environment::TELLO_Dyn_environment():
    ang_vel_(3)
{
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());
    /********** Robot Set  **********/
    robot_ = new TELLO();
    robot_->BuildRobot(Vec3 (0., 0., 0.), 
            srSystem::FIXED, srJoint::TORQUE, ModelPath"TELLO/TELLO_humanoid.urdf");
            //srSystem::FIXED, srJoint::TORQUE, ModelPath"TELLO/TELLO_URDF.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    interface_ = new TELLO_interface();
    data_ = new TELLO_SensorData();
    cmd_ = new TELLO_Command();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(0.001);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(50);

    std::cout<<robot_->link_[robot_->link_idx_map_.find("R_FOOT")->second]
        ->GetPosition()<<std::endl;;
    std::cout<<robot_->link_[robot_->link_idx_map_.find("L_FOOT")->second]
        ->GetPosition()<<std::endl;;
    printf("[TELLO Dynamic Environment] Build Dynamic Environment\n");
}

void TELLO_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;
    TELLO_Dyn_environment* pDyn_env = (TELLO_Dyn_environment*)_data;
    TELLO* robot = (TELLO*)(pDyn_env->robot_);
    TELLO_SensorData* p_data = pDyn_env->data_;
    
    std::vector<double> torque_command(robot->num_act_joint_);

    for(int i(0); i<robot->num_act_joint_; ++i){
        p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        p_data->jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    }
    pDyn_env->_CheckFootContact(
            p_data->rfoot_contact, p_data->lfoot_contact);
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    pDyn_env->getIMU_Data(imu_acc, imu_ang_vel);
    for (int i(0); i<3; ++i){
        p_data->imu_ang_vel[i] = imu_ang_vel[i];
        p_data->imu_acc[i] = -imu_acc[i];
    }
    pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    for(int i(0); i<3; ++i){
        robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }

    if( count < 100 ){
    //if(true){
        robot->vp_joint_[0]->m_State.m_rCommand = 
            -1000. * robot->vp_joint_[0]->m_State.m_rValue[0]
            - 10. * robot->vp_joint_[0]->m_State.m_rValue[1];
        robot->vp_joint_[1]->m_State.m_rCommand = 
            -1000. * robot->vp_joint_[1]->m_State.m_rValue[0]
            - 10. * robot->vp_joint_[1]->m_State.m_rValue[1];
    }

    double Kp(100.);
    double Kd(5.0);
    double ramp(1.);
    if( count < 10 ){
        ramp = ((double)count)/10.;
    }
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = 
          pDyn_env->cmd_->jtorque_cmd[i] +
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
    }

    /*
    Kp = 1.0;
    Kd = 0.1;
    // Right Ankle
    int r_ankle_idx = robot->r_joint_idx_map_.find("r_ankle")->second;
    //printf("r_ankle idx: %d\n", r_ankle_idx);
    robot->r_joint_[r_ankle_idx]->m_State.m_rCommand = 
      pDyn_env->cmd_->jtorque_cmd[r_ankle_idx] +
            Kp * (pDyn_env->cmd_->jpos_cmd[r_ankle_idx] - p_data->jpos[r_ankle_idx]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[r_ankle_idx] - p_data->jvel[r_ankle_idx]);

    int l_ankle_idx = robot->r_joint_idx_map_.find("l_ankle")->second;
    //printf("l_ankle idx: %d\n", l_ankle_idx);
    robot->r_joint_[l_ankle_idx]->m_State.m_rCommand = 
      pDyn_env->cmd_->jtorque_cmd[l_ankle_idx] +
            Kp * (pDyn_env->cmd_->jpos_cmd[l_ankle_idx] - p_data->jpos[l_ankle_idx]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[l_ankle_idx] - p_data->jvel[l_ankle_idx]);
*/
     // TEST
    //for(int i(0); i<3; ++i){
        //robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        //robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    //}
    //for(int i(0); i<robot->num_act_joint_; ++i){
        //robot->r_joint_[i]->m_State.m_rCommand = 0.;
    //}
}


  void TELLO_Dyn_environment::Rendering_Fnc(){
  }
    void TELLO_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
        SO3 so3_body =  robot_->
            link_[robot_->link_idx_map_.find("BODY")->second]->GetOrientation();

        Eigen::Matrix3d ori_mtx;
        for (int i(0); i<3; ++i){
            ori_mtx(i, 0) = so3_body[0+i];
            ori_mtx(i, 1) = so3_body[3+i];
            ori_mtx(i, 2) = so3_body[6+i];
        }
        dynacore::Quaternion ori_quat(ori_mtx);
        rot = ori_quat;
    }
    TELLO_Dyn_environment::~TELLO_Dyn_environment()
    {
        SR_SAFE_DELETE(interface_);
        SR_SAFE_DELETE(robot_);
        SR_SAFE_DELETE(m_Space);
        SR_SAFE_DELETE(m_ground);
    }

void TELLO_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("L_FOOT")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("R_FOOT")->second]->GetPosition();

    //std::cout<<"right: "<<rfoot_pos<<std::endl;
    //std::cout<<"left: "<<lfoot_pos<<std::endl;

    // TELLO contact check
    //if(  fabs(lfoot_pos[2]) < 0.016){
        //l_contact = true;
    //}else { l_contact = false; }

    //if (fabs(rfoot_pos[2])<0.016  ){
        //r_contact = true;
    //} else { r_contact = false; }

    // Humanoid contact check
    if(  fabs(lfoot_pos[2]) < 0.03){
        l_contact = true;
    }else { l_contact = false; }
    if (fabs(rfoot_pos[2])<0.03  ){
        r_contact = true;
    } else { r_contact = false; }

    //printf("\n");
}
void TELLO_Dyn_environment::getIMU_Data(
    std::vector<double> & imu_acc,
    std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = 
      robot_->link_[robot_->link_idx_map_.find("BODY")->second]->GetVel();
  se3 imu_se3_acc = 
      robot_->link_[robot_->link_idx_map_.find("BODY")->second]->GetAcc();
  SE3 imu_frame = 
      robot_->link_[robot_->link_idx_map_.find("BODY")->second]->GetFrame();
  SO3 imu_ori = 
      robot_->link_[robot_->link_idx_map_.find("BODY")->second]->GetOrientation();

  Eigen::Matrix3d Rot;
  Rot<<
    imu_frame(0,0), imu_frame(0,1), imu_frame(0,2),
    imu_frame(1,0), imu_frame(1,1), imu_frame(1,2),
    imu_frame(2,0), imu_frame(2,1), imu_frame(2,2);

  dynacore::Vect3 grav; grav.setZero();
  grav[2] = 9.81;
  dynacore::Vect3 local_grav = Rot.transpose() * grav;

  for(int i(0); i<3; ++i){
    imu_ang_vel[i] = imu_se3_vel[i];
    imu_acc[i] = local_grav[i];
  }
}
