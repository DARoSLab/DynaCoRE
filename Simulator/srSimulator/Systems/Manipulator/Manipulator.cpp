#include "Manipulator.h"

Manipulator::Manipulator():SystemGenerator()
{
  printf("[Manipulator] ASSEMBLED\n");
}

Manipulator::~Manipulator(){
}

void Manipulator::_SetJointLimit(){
}

void Manipulator::_SetCollision(){
}

void Manipulator::_SetInitialConf(){
  //case 0 : just stand
  //case 1 : left leg up
  //case 2 : lower stand
  //case 3: walking ready
  //case 4 : ICRA 2018

  int pose(2);

  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  vp_joint_[2]->m_State.m_rValue[0] = 0.5;
  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  for(int i(0); i< num_act_joint_; ++i) r_joint_[i]->m_State.m_rValue[0] = 0.;

  r_joint_[r_joint_idx_map_.find("thigh_fr_to_knee_fr_j")->second]->
      m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("thigh_fl_to_knee_fl_j")->second]->
      m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("thigh_hr_to_knee_hr_j")->second]->
      m_State.m_rValue[0] = -0.4;
  r_joint_[r_joint_idx_map_.find("thigh_hl_to_knee_hl_j")->second]->
      m_State.m_rValue[0] = -0.4;

  switch(pose){
  case 0:

    break;

  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 1.131;
    break;

  case 2:
    vp_joint_[2]->m_State.m_rValue[0] =  0.4;
    break;

  case 3:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.032720;
    vp_joint_[2]->m_State.m_rValue[0] =  1.050418;
    break;

  case 4:
    vp_joint_[0]->m_State.m_rValue[0] =  -0.0183;
    vp_joint_[2]->m_State.m_rValue[0] =  1.079;

    break;
  }
  KIN_UpdateFrame_All_The_Entity();
}
