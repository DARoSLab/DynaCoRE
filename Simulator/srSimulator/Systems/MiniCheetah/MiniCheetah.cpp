#include "MiniCheetah.h"

MiniCheetah::MiniCheetah():SystemGenerator()
{
  printf("[MiniCheetah] ASSEMBLED\n");
}

MiniCheetah::~MiniCheetah(){
}

void MiniCheetah::_SetJointLimit(){
}

void MiniCheetah::_SetCollision(){
  //collision_.resize(2);
  //for (int i = 0; i < collision_.size(); ++i) {
    //collision_[i] = new srCollision();
  //}

  //collision_[0]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  //collision_[0]->GetGeomInfo().SetDimension(0.23, 0.16, 0.03);
  //collision_[1]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
  //collision_[1]->GetGeomInfo().SetDimension(0.23, 0.16, 0.03);

  //link_[link_idx_map_.find("r_foot")->second]->AddCollision(collision_[0]);
  //link_[link_idx_map_.find("l_foot")->second]->AddCollision(collision_[1]);

  //double fric(0.8);
  //link_[link_idx_map_.find("r_foot")->second]->SetFriction(fric);
  //link_[link_idx_map_.find("l_foot")->second]->SetFriction(fric);

  //double damp(0.01);
  //link_[link_idx_map_.find("r_foot")->second]->SetDamping(damp);
  //link_[link_idx_map_.find("l_foot")->second]->SetDamping(damp);

  //double restit(0.0);
  //link_[link_idx_map_.find("r_foot")->second]->SetRestitution(restit);
  //link_[link_idx_map_.find("l_foot")->second]->SetRestitution(restit);
}

void MiniCheetah::_SetInitialConf(){
  //case 0 : just stand
  //case 1 : left leg up
  //case 2 : lower stand
  //case 3: walking ready
  //case 4 : ICRA 2018

  int pose(1);

  vp_joint_[0]->m_State.m_rValue[0] = 0.0;
  vp_joint_[1]->m_State.m_rValue[0] = 0.0;
  vp_joint_[2]->m_State.m_rValue[0] = 1.135;// + 0.3;

  vr_joint_[0]->m_State.m_rValue[0] = 0.0;
  vr_joint_[1]->m_State.m_rValue[0] = 0.0;
  vr_joint_[2]->m_State.m_rValue[0] = 0.0;

  switch(pose){
  case 0:

    break;

  case 1:
    vp_joint_[2]->m_State.m_rValue[0] = 1.131;

    r_joint_[0]->m_State.m_rValue[0] = 0.0;
    r_joint_[1]->m_State.m_rValue[0] = -0.7;
    r_joint_[2]->m_State.m_rValue[0] = 1.4;

    r_joint_[3]->m_State.m_rValue[0] = 0.0;
    r_joint_[4]->m_State.m_rValue[0] = -0.7;
    r_joint_[5]->m_State.m_rValue[0] = 1.4;

    r_joint_[6]->m_State.m_rValue[0] = 0.0;
    r_joint_[7]->m_State.m_rValue[0] = -0.7;
    r_joint_[8]->m_State.m_rValue[0] = 1.4;

    r_joint_[9]->m_State.m_rValue[0] = 0.0;
    r_joint_[10]->m_State.m_rValue[0] = -0.7;
    r_joint_[11]->m_State.m_rValue[0] = 1.4;
      break;

  case 2:
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
