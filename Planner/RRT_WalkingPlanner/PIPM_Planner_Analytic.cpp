#include "PIPM_Planner_Analytic.h"
#include "EnvironmentSetup/Terrain.h"
#include "EnvironmentSetup/CoMSurface.h"
#include <Utils/utilities.h>
#include <iostream>
#include <stdio.h>

Planner_Analytic::Planner_Analytic():Planner(){
}
Planner_Analytic::~Planner_Analytic(){
}


void Planner_Analytic::_SetAB(int idx, double * A, double * B){
  double xp( foot_list_[idx][0] );
  double yp( foot_list_[idx][1] );

  double x0( initial_state_list_[idx][0] );
  double y0( initial_state_list_[idx][1] );

  double x0dot( initial_state_list_[idx][3] );
  double y0dot( initial_state_list_[idx][4] );

  A[0] = 0.5 * ( (x0 - xp) + 1./omega * x0dot);
  B[0] = 0.5 * ( (x0 - xp) - 1./omega * x0dot);

  A[1] = 0.5 * ( (y0 - yp) + 1./omega * y0dot);
  B[1] = 0.5 * ( (y0 - yp) - 1./omega * y0dot);
}


bool Planner_Analytic::GetCoMState(int step_idx, double time, sejong::Vector & com_full_state) {
  // (x, y, z, xdot, ydot, zdot, xddot, yddot, zddot)
  com_full_state = sejong::Vector::Zero(9);
  double foot_pos[2];

  foot_pos[0] = foot_list_[step_idx][0];
  foot_pos[1] = foot_list_[step_idx][1];

  double A[2];
  double B[2];
  _SetAB(step_idx, A, B);

  for (int i(0); i < 2; ++i){
    com_full_state[i] = A[i] * exp(omega*time) + B[i] * exp(-omega*time) + foot_pos[i];
    com_full_state[3 + i] = omega * (A[i] * exp(omega*time) - B[i] * exp(-omega*time));
    com_full_state[6 + i] = omega * omega * (com_full_state[i] - foot_pos[i]);
  }
  com_full_state[2] = com_surf_->getZ(com_full_state[0],
                                      com_full_state[1]);

  com_full_state[5] = com_surf_->getZdot(com_full_state[0],
                                         com_full_state[1],
                                         com_full_state[3],
                                         com_full_state[4]);

  com_full_state[8] = com_surf_->getZddot(com_full_state[0],
                                          com_full_state[1],
                                          com_full_state[3],
                                          com_full_state[4]);

  return true;
}

double Planner_Analytic::GetFinTime(int step_idx) {
  return switching_time_list_[step_idx];
}

bool Planner_Analytic::GetFootPlacement(int sequence_idx,
                      sejong::Vector & foot_placement){
  if(sequence_idx > num_sequence_){ return false; }
  else {
    foot_placement = foot_list_[sequence_idx];
  }
  return true;
}

bool Planner_Analytic::GetOrientation(int sequence_idx, sejong::Quaternion & ori){
  if(sequence_idx > num_sequence_){ return false; }
  else {
    ori = ori_list_[sequence_idx];
    return true;
  }
}
sejong::Quaternion Planner_Analytic::_Quaternion_RotZ(double theta){
  sejong::Quaternion ret_quat;
  ret_quat.w() = cos(theta/2.);
  ret_quat.x() = 0.;
  ret_quat.y() = 0.;
  ret_quat.z() = sin(theta/2.);

  return ret_quat;
}
bool Planner_Analytic::_SolvePlanning(PlanningParam* param){
  AnalyticPL_Param* _param = static_cast<AnalyticPL_Param*>(param);
  omega = sqrt(9.81/_param->curr_com_state[2]);
  num_sequence_ = (_param->xp_list).size();

  // Initial
  int start_idx(0);
  sejong::Vector ini_state(6);
  if(_param->is_initial){
    foot_list_.push_back(_param->fixed_pivot);
    initial_state_list_.push_back(_param->curr_com_state);
    ori_list_.push_back(_Quaternion_RotZ(0.));
    _find_initial(_param->curr_com_state, _param->fixed_pivot,
                  _param->xp_list[0], _param->apex_vel_list[0],
                  ini_state); // Add 1st foot too
    ori_list_.push_back(_Quaternion_RotZ(0.));
    initial_state_list_.push_back(ini_state);
    start_idx = 1;
  }else {
    ini_state = _param->curr_com_state;
    initial_state_list_.push_back(ini_state);
    foot_list_.push_back(_param->fixed_pivot);
  }

  sejong::Vect3 nx_foot, local_nx_foot;

  sejong::Vect3 curr_foot(foot_list_[start_idx]);
  sejong::Vect3 vect3_zero;
  vect3_zero.setZero();
  sejong::Vector curr_state(ini_state);
  sejong::Vector local_state;

  double switching_time;
  double xp;
  double v_apex, ydot_des;
  sejong::Vector switching_state(6);
  sejong::Vector local_switching_state(6);

  double theta;

  for (int i = start_idx; i< num_sequence_; ++i){
    theta = _param->theta_list[i];
    // Project to Local Frame
    xp = _param->xp_list[i];
    v_apex = _param->apex_vel_list[i];
    printf("xp, v apex: %f, %f \n", xp, v_apex);
    _ChangeFrame(curr_state, curr_foot,  theta, local_state);
    _find_switching(xp, v_apex, 0.,
                    local_state, vect3_zero,
                    local_switching_state, local_nx_foot, switching_time);

    // Project Back to Global Frame
    _ChangeFrameBack(local_switching_state, curr_foot,  theta, switching_state);
    _ChangeFrameBack(local_nx_foot, curr_foot,  theta, nx_foot);

    sejong::pretty_print(switching_state, std::cout, "switching state");
    sejong::pretty_print((sejong::Vector)nx_foot, std::cout, "nx foot");
    printf ("switching time: %f \n \n", switching_time);

    foot_list_.push_back(nx_foot);
    ori_list_.push_back(_Quaternion_RotZ(theta));
    initial_state_list_.push_back(switching_state);
    switching_time_list_.push_back(switching_time);

    curr_foot = nx_foot;
    curr_state = switching_state;
  }

  // Final: TODO

  return true;
}
void Planner_Analytic::_ChangeFrameBack(const sejong::Vect3 & pos,
                                        const sejong::Vect3 & offset,
                                        double theta,
                                        sejong::Vect3 & changed_pos){
  changed_pos = pos;
  // Position
  changed_pos.head(2) = _2D_Rotate(pos.head(2), theta);
  changed_pos.head(2) += offset.head(2);
}

void Planner_Analytic::_ChangeFrameBack(const sejong::Vector & state,
                                        const sejong::Vect3 & offset,
                                        double theta,
                                        sejong::Vector & changed_state){
  changed_state = state;
  // Position
  sejong::Vector original_pos = state.head(2);
  original_pos = _2D_Rotate(original_pos, theta);
  original_pos += offset.head(2);
  changed_state.head(2) = original_pos;
  // Velocity
  changed_state.segment(3,2) = _2D_Rotate(changed_state.segment(3,2), theta);

}

void Planner_Analytic::_ChangeFrame(const sejong::Vector & state,
                                    const sejong::Vect3 & offset,
                                    double theta,
                                    sejong::Vector & changed_state){
  changed_state = state;
  //Position
  sejong::Vector local_pos = state.head(3) - offset;
  local_pos.head(2) = _2D_Rotate(local_pos.head(2), -theta);
  changed_state.head(2) = local_pos.head(2);
  // Velocity
  changed_state.segment(3,2) = _2D_Rotate(changed_state.segment(3,2), -theta);
}

sejong::Vector Planner_Analytic::_2D_Rotate(const sejong::Vector & vec2,
                                            double theta){
  sejong::Vector vec2_tran(2);
  vec2_tran[0] = vec2[0] * cos(theta) - vec2[1] * sin(theta);
  vec2_tran[1] = vec2[0] * sin(theta) + vec2[1] * cos(theta);

  return vec2_tran;
}

bool Planner_Analytic::_find_initial(const sejong::Vector & curr_com,
                                     const sejong::Vect3 & curr_foot,
                                     double nx_xp, double v_apex,
                                     sejong::Vector & ini_state){
  sejong::pretty_print(curr_com, std::cout, "ini com");
  sejong::pretty_print((sejong::Vector)curr_foot, std::cout, "curr foot");

  // x, xdot, z, zdot are same
  double yini(-0.0);
  ini_state[1] = yini;

  double yp (curr_foot[1]);
  double y0 (curr_com[1]);
  double y0dot (curr_com[4]);

  // First swing time
  double A( 0.5 * ((y0 - yp)  + 1./omega * y0dot) );
  double B( 0.5 * ((y0 - yp)  - 1./omega * y0dot) );

  double exp_omega_t = (-(yp-yini) + sqrt(pow(yp-yini,2) - 4 * A * B))/(2 * A);
  if(exp_omega_t <0 ){
    exp_omega_t = (-(yp-yini) - sqrt(pow(yp-yini,2) - 4 * A * B))/(2 * A);
  }
  printf("exp omega t: %f \n", exp_omega_t);

  double t_s = 1./omega * log (exp_omega_t);
  switching_time_list_.push_back(t_s);

  ini_state[4] = omega * (A * exp(omega * t_s) - B * exp(-omega * t_s));
  printf("ts: %f \n", t_s);

  double xp(curr_foot[0]);
  double x0(curr_com[0]);
  double x0dot(curr_com[3]);

  double A_x(0.5 * ((x0 - xp) + 1./omega * x0dot ) );
  double B_x(0.5 * ((x0 - xp) - 1./omega * x0dot ) );

  ini_state[0] = A_x * exp(omega * t_s) + B_x * exp(-omega  * t_s) + xp;
  ini_state[3] = omega * (A_x * exp(omega * t_s) - B_x * exp(-omega  * t_s) );

  ini_state[2] = com_surf_->getZ(ini_state[0],
                                 ini_state[1]);
  ini_state[5] = com_surf_->getZdot(ini_state[0],
                                    ini_state[1],
                                    ini_state[3],
                                    ini_state[4]);
  sejong::pretty_print(ini_state, std::cout, "ini state");
  double x1(ini_state[0]);
  double x1dot(ini_state[3]);

  double A_nx(0.5 * ((x1 - nx_xp) + 1./omega * x1dot ) );
  double B_nx(0.5 * ((x1 - nx_xp) - 1./omega * x1dot ) );

  double t_apex = 1./(2 * omega) * log  ( - B_nx/ A_nx);
  printf("t apex: %f\n", t_apex);
  sejong::Vect3 nx_foot;
  nx_foot[0] = nx_xp;
  nx_foot[1] = _find_yp(ini_state[1], ini_state[4], t_apex);
  nx_foot[2] = terrain_->getZp(nx_foot[0], nx_foot[1]);
  sejong::pretty_print((sejong::Vector)nx_foot, std::cout, "nx foot");

  foot_list_.push_back(nx_foot);
  return true;
}

bool Planner_Analytic::_find_switching(double xp,
                                       double v_apex, double ydot_des,
                                       const sejong::Vector & curr_state,
                                       const sejong::Vect3 & curr_foot,
                                       sejong::Vector & switching_state,
                                       sejong::Vect3 & nx_foot,
                                       double & switching_time){
  nx_foot[0] = xp;
  sejong::pretty_print((sejong::Vector)curr_foot, std::cout, "curr foot");
  sejong::pretty_print(curr_state, std::cout, "curr state");

  switching_state[0] = _switching_state_x(curr_foot[0], xp,
                                          curr_state[0], curr_state[3],
                                          v_apex);

  switching_state[3] = _find_velocity(switching_state[0],
                                      curr_foot[0],
                                      curr_state[0], curr_state[3]);


  switching_time = _find_time(switching_state[0], switching_state[3],
                              curr_state[0], curr_state[3],
                              curr_foot[0]);

  double t_apex = _find_time(xp, v_apex,
                             switching_state[0], switching_state[3],
                             xp);
  printf("apex time : %f \n", t_apex);

  _find_y_switching_state(curr_state[1], curr_state[4],
                          switching_time, curr_foot[1],
                          switching_state[1], switching_state[4]);

  nx_foot[1] = _find_yp(switching_state[1], switching_state[4],
                        t_apex, ydot_des);

  nx_foot[2] = terrain_->getZp(nx_foot[0], nx_foot[1]);

  switching_state[2] = com_surf_->getZ(switching_state[0],
                                       switching_state[1]);
  switching_state[5] = com_surf_->getZdot(switching_state[0],
                                          switching_state[1],
                                          switching_state[3],
                                          switching_state[4]);

  return true;
}
double Planner_Analytic::_find_yp( double y0, double y0dot, double t_apex){
  return _find_yp(y0, y0dot, t_apex, 0.);
}
double Planner_Analytic::_find_yp( double y0, double y0dot, double t_apex,double ydot_des){
  double yp ( y0 - (1 + exp( 2* omega * t_apex))/ (1 - exp( 2* omega * t_apex)) * (y0dot/ omega)  + 2 * ydot_des/(( exp(-omega * t_apex) - exp(omega*t_apex)  ) * omega) );
  return yp;
}



void Planner_Analytic::_find_y_switching_state(double y0, double y0dot,
                                               double t_switch,
                                               double yp,
                                               double & sw_pos,
                                               double & sw_vel){
  double A (0.5 * ( (y0 - yp) + 1./omega * y0dot));
  double B (0.5 * ( (y0 - yp) - 1./omega * y0dot));

  sw_pos = A * exp(omega * t_switch) + B * exp(-omega * t_switch) + yp;
  sw_vel = omega * ( A * exp(omega * t_switch) - B * exp (-omega * t_switch));
  // printf("y0 pos, vel, yp: (%f, %f, %f)\n", y0, y0dot, yp);

  // printf("y sw pos, vel: (%f, %f)\n", sw_pos, sw_vel);
}

double Planner_Analytic::_find_time(double x, double xdot,
                                    double x0, double x0dot,
                                    double xp){
  double A ( 0.5 * ((x0-xp) + 1/omega * x0dot) );
  double time = 1/omega * log ( (x + 1/omega * xdot - xp)/(2*A) );
  return time;
}

double Planner_Analytic::_find_velocity(double x, double xp, double x0, double x0dot){
  double sign(1.0);
  if(x0dot < 0) sign = -1.0;

  double xdot = sign * sqrt( pow(omega,2) * ( pow(x - xp, 2) - pow(x0 - xp, 2) ) + pow(x0dot, 2) );

  return xdot;
}
double Planner_Analytic::_switching_state_x(double xp1, double xp2,
                                            double x0, double x0dot,
                                            double v_apex){
  double C ( pow(x0 - xp1, 2.) + (pow(v_apex, 2.) - pow(x0dot, 2.))/pow(omega, 2.) );

  double x_switch( 0.5 * (C / (xp2 - xp1) + (xp1 + xp2) ) );

  return x_switch;
}

void Planner_Analytic::_PreProcessing(Terrain* terrain,
                                      CoMSurface* com_surf){
  terrain_ = terrain;
  com_surf_ = com_surf;
  foot_list_.clear();
  initial_state_list_.clear();
  switching_time_list_.clear();
}
