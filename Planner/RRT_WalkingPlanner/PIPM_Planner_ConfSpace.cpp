#include "PIPM_Planner_ConfSpace.h"
#include "EnvironmentSetup/Terrain.h"
#include "EnvironmentSetup/CoMSurface.h"
#include "Configuration.h"
#include <Utils/utilities.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>

RRT_Analytic_Solver::RRT_Analytic_Solver():Planner(){
}
RRT_Analytic_Solver::~RRT_Analytic_Solver(){
}

void RRT_Analytic_Solver::_SetAB(int idx, double * A, double * B){
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

void RRT_Analytic_Solver::_Set_local_AB_to_apex(int idx, double * A, double * B){
  double xp( (next_pivot_list_[idx - 2])[0] );
  double yp( (next_pivot_list_[idx - 2])[1] );

  double x0( (initial_state_list_[idx])[0] );
  double y0( (initial_state_list_[idx])[1] );
  double x0dot( (initial_state_list_[idx])[3] );
  double y0dot( (initial_state_list_[idx])[4] );

  A[0] = 0.5 * ( (x0 - xp) + 1./omega * x0dot);
  B[0] = 0.5 * ( (x0 - xp) - 1./omega * x0dot);

  A[1] = 0.5 * ( (y0 - yp) + 1./omega * y0dot);
  B[1] = 0.5 * ( (y0 - yp) - 1./omega * y0dot);
}

void RRT_Analytic_Solver::_Set_local_AB_to_peak(int idx, double * A, double * B){

  double xp;
  double yp;
  double x0dot;
  if(idx == 1){
    xp = 0.;
    yp = foot_list_[1][1];
    x0dot = apex_state_list_[idx - 1][2];
  } else{
    xp = (curr_pivot_list_[idx - 2])[0];
    yp = (curr_pivot_list_[idx - 2])[1];
    x0dot = (apex_vel_decomp_[idx - 2])[0];
  }

  double x0( (apex_state_list_[idx - 1])[0] );
  double y0( (apex_state_list_[idx - 1])[1] );
  double y0dot(0);

  A[0] = 0.5 * ( (x0 - xp) + 1./omega * x0dot);
  B[0] = 0.5 * ( (x0 - xp) - 1./omega * x0dot);

  A[1] = 0.5 * ( (y0 - yp) + 1./omega * y0dot);
  B[1] = 0.5 * ( (y0 - yp) - 1./omega * y0dot);
}

bool RRT_Analytic_Solver::GetCoMState(int step_idx, double time, sejong::Vector & com_full_state) {
  if((step_idx == 0) || (step_idx == 1 && time <= apex_time_list_[step_idx - 1])){
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
    // com_full_state[2] = com_surf_->getZ(com_full_state[0],
    //                                     com_full_state[1]);

    // com_full_state[5] = com_surf_->getZdot(com_full_state[0],
    //                                        com_full_state[1],
    //                                        com_full_state[3],
    //                                        com_full_state[4]);

    // com_full_state[8] = com_surf_->getZddot(com_full_state[0],
    //                                         com_full_state[1],
    //                                         com_full_state[3],
    //                                         com_full_state[4]);
    com_full_state[2] = 1.026702;
    com_full_state[5] = 0.;
    com_full_state[8] = 0.;

    return true;
  } else{
    if(time <= apex_time_list_[step_idx - 1]){
      //peak -> apex
      com_full_state = sejong::Vector::Zero(9);

      double foot_pos[2];
      foot_pos[0] = (next_pivot_list_[step_idx - 2])[0];
      foot_pos[1] = (next_pivot_list_[step_idx - 2])[1];

      double A[2];
      double B[2];
      _Set_local_AB_to_apex(step_idx, A, B);

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

      //local state -> global state
      _TF_to_global(step_idx - 2, com_full_state);

      return true;
    } else{
      //apex -> peak
      time -= apex_time_list_[step_idx - 1];
      com_full_state = sejong::Vector::Zero(9);

      double foot_pos[2];
      if(step_idx == 1){
        foot_pos[0] = 0.;
        foot_pos[1] = foot_list_[1][1];
      } else{
        foot_pos[0] = (curr_pivot_list_[step_idx - 2])[0];
        foot_pos[1] = (curr_pivot_list_[step_idx - 2])[1];
      }

      double A[2];
      double B[2];
      _Set_local_AB_to_peak(step_idx, A, B);

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
      //local state -> global state
      _TF_to_global(step_idx - 1, com_full_state);
      return true;
    }
  }
}

double RRT_Analytic_Solver::GetFinTime(int step_idx) {
  if(step_idx ==0)
    return switching_time_list_[step_idx];
  else
    return switching_time_list_[step_idx] + apex_time_list_[step_idx - 1];
}

bool RRT_Analytic_Solver::GetFootPlacement(int sequence_idx,
                                           sejong::Vector & foot_placement){
  if(sequence_idx > num_sequence_){ return false; }
  else {
    foot_placement = (foot_list_[sequence_idx]).head(3);
    return true;
  }
}

bool RRT_Analytic_Solver::GetOrientation(int sequence_idx, sejong::Quaternion & ori){
  if(sequence_idx > num_sequence_){ return false; }
  else {
    sejong::convert((foot_list_[sequence_idx])[3], 0., 0., ori);
    // printf("%d th theta: %f\n", sequence_idx, foot_list_[sequence_idx][3]);
    // sejong::pretty_print(ori, std::cout, "ori_quat");
    return true;
  }
}

bool RRT_Analytic_Solver::_SolvePlanning(PlanningParam* param){

  bool planning(true);

  RRT_Param* _param = static_cast<RRT_Param*>(param);
  if(_param->planning){

    //PARAMETER DISTRIBUTING
    RRT_Node* temp = new RRT_Node(0,0,0);
    num_sequence_ = _param->sol_node->idx_;
    temp = _param->sol_node;
    // for(int i(0); i<_param->sol_node->idx_ - 1; ++i){
    for(int i(0); i<num_sequence_ - 1; ++i){
      temp->parent_->sol_child_ = temp;
      temp = temp->parent_;
    }
    temp = _param->root_node->sol_child_;

    // for(int i(2); i < _param->sol_node->idx_ + 1; ++i){
    for(int i(2); i < num_sequence_ + 1; ++i){
      foot_list_.push_back(temp->foot_glob_);
      apex_vel_decomp_.push_back(temp->apex_vel_decomp_prev_frame_);
      next_pivot_list_.push_back(temp->foot_prev_frame_);
      curr_pivot_list_.push_back(temp->foot_curr_frame_);
      SE3_list_.push_back(temp->SE3_);
      inv_SE3_list_.push_back(temp->inv_SE3_);
      initial_state_list_.push_back(temp->initial_state_prev_frame_);
      apex_state_list_.push_back(temp->apex_state_curr_frame_);
      apex_time_list_.push_back(temp->apex_time_);
      switching_time_list_.push_back(temp->switching_time_);

      if(i != _param->sol_node->idx_)
        temp = temp->sol_child_;
    }
    delete temp;
    _save_param();
  } else{
    _read_param_from_text(_param->curr_com_state[2]);
  }

  //SAVE for PLOTTING
  //_plotting_full_trajectory(_param);
  //exit(0);

  // Final: TODO
  std::cout<<"[PARAMETERS] DISTRIBUTED"<<std::endl;
  return true;
}

void RRT_Analytic_Solver::_read_param_from_text(double curr_com_state){

  std::string path(THIS_COM);
  std::ifstream foot_file;
  std::ifstream apex_vel_decomp_file;
  std::ifstream next_pivot_list_file;
  std::ifstream curr_pivot_list_file;
  std::ifstream SE3_list_file;
  std::ifstream inv_SE3_list_file;
  std::ifstream initial_state_list_file;
  std::ifstream apex_state_list_file;
  std::ifstream apex_time_list_file;
  std::ifstream switching_time_list_file;


  std::string output;
  double val(0);
  sejong::Vector four_dim(4);
  sejong::Matrix mat(3,3);
  sejong::Vector six_dim(6);
  sejong::Vect3 three_dim;
  std::string file_name = path+"experiment_data/_foot_list.txt"; 
  foot_file.open(file_name.c_str());
  file_name = path+"experiment_data/_apex_vel_decomp.txt"; 
  apex_vel_decomp_file.open(file_name.c_str());
  file_name = path+"experiment_data/_next_pivot_list.txt"; 
  next_pivot_list_file.open(file_name.c_str());
  file_name = path+"experiment_data/_curr_pivot_list.txt"; 
  curr_pivot_list_file.open(file_name.c_str());
  file_name = path+"srSimulator/build/_SE3_list.txt";
  SE3_list_file.open(file_name.c_str());
  file_name = (path+"srSimulator/build/_inv_SE3_list.txt");
  inv_SE3_list_file.open(file_name.c_str());
  file_name = path+"experiment_data/_initial_state_list.txt";
  initial_state_list_file.open(file_name.c_str());
  file_name =path+"experiment_data/_apex_state_list.txt";
  apex_state_list_file.open(file_name.c_str());
  file_name = path+"experiment_data/_apex_time_list.txt";
  apex_time_list_file.open(file_name.c_str());
  file_name =path+"experiment_data/_switching_time_list.txt";
  switching_time_list_file.open(file_name.c_str());

  inv_SE3_list_.clear();
  while(!inv_SE3_list_file.eof()){
    for(int i(0); i<3; ++i){
      for(int j(0); j<3; ++j){
        inv_SE3_list_file >> output;
        mat(i,j) = atof( output.c_str() );
      }
    }
    inv_SE3_list_.push_back(mat);
  }
  //std::cout<<inv_SE3_list_.size()<<std::endl;

  SE3_list_.clear();
  while(!SE3_list_file.eof()){
    for(int i(0); i<3; ++i){
      for(int j(0); j<3; ++j){
        SE3_list_file >> output;
        mat(i,j) = atof( output.c_str() );
      }
    }
    SE3_list_.push_back(mat);
  }
  //std::cout<<SE3_list_.size()<<std::endl;

  switching_time_list_.clear();
  while(!switching_time_list_file.eof()){
    switching_time_list_file >> output;
    val = atof( output.c_str() );
    switching_time_list_.push_back(val);
  }
  //std::cout<<switching_time_list_.size()<<std::endl;

  apex_time_list_.clear();
  while(!apex_time_list_file.eof()){
    apex_time_list_file >> output;
    val = atof( output.c_str() );
    apex_time_list_.push_back(val);
  }
  //std::cout<<apex_time_list_.size()<<std::endl;

  apex_state_list_.clear();
  while(!apex_state_list_file.eof()){
    for(int i(0); i<4; ++i){
      apex_state_list_file >> output;
      four_dim[i] = atof( output.c_str() );
    }
    apex_state_list_.push_back(four_dim);
  }
  //std::cout<<apex_state_list_.size()<<std::endl;

  initial_state_list_.clear();
  while(!initial_state_list_file.eof()){
    for(int i(0); i<6; ++i){
      initial_state_list_file >> output;
      six_dim[i] = atof( output.c_str() );
    }
    initial_state_list_.push_back(six_dim);
  }
  //std::cout<<initial_state_list_.size()<<std::endl;

  curr_pivot_list_.clear();
  while(!curr_pivot_list_file.eof()){
    for(int i(0); i<3; ++i){
      curr_pivot_list_file >> output;
      three_dim[i] = atof( output.c_str() );
    }
    curr_pivot_list_.push_back(three_dim);
  }
  //std::cout<<curr_pivot_list_.size()<<std::endl;

  next_pivot_list_.clear();
  while(!next_pivot_list_file.eof()){
    for(int i(0); i<3; ++i){
      next_pivot_list_file >> output;
      three_dim[i] = atof( output.c_str() );
    }
    next_pivot_list_.push_back(three_dim);
  }
  //std::cout<<next_pivot_list_.size()<<std::endl;

  apex_vel_decomp_.clear();
  while(!apex_vel_decomp_file.eof()){
    for(int i(0); i<3; ++i){
      apex_vel_decomp_file >> output;
      three_dim[i] = atof( output.c_str() );
    }
    apex_vel_decomp_.push_back(three_dim);
  }
  //std::cout<<apex_vel_decomp_.size()<<std::endl;

  foot_list_.clear();
  while(!foot_file.eof()){
    for(int i(0); i<4; ++i){
      foot_file >> output;
      four_dim[i] = atof( output.c_str() );
    }
    foot_list_.push_back(four_dim);
  }
  //std::cout<<foot_list_.size()<<std::endl;

  omega = sqrt(9.81/curr_com_state);
  num_sequence_ = foot_list_.size();
}

void RRT_Analytic_Solver::_save_param(){
  for(int i(0); i<foot_list_.size(); ++i){
    sejong::saveVector(foot_list_[i],"_foot_list");
    sejong::saveVector(initial_state_list_[i],"_initial_state_list");
  }

  for(int i(0); i<apex_vel_decomp_.size(); ++i){
    sejong::saveVector(apex_vel_decomp_[i],"_apex_vel_decomp");
    sejong::saveVector(next_pivot_list_[i],"_next_pivot_list");
    sejong::saveVector(curr_pivot_list_[i],"_curr_pivot_list");
  }
  std::ofstream se3;
  std::ofstream inv_se3;
  remove("_SE3_list_.txt");
  remove("_inv_SE3_list_.txt");
  se3.open("_SE3_list.txt");
  inv_se3.open("_inv_SE3_list.txt");
  for(int i(0); i<SE3_list_.size(); ++i){
    sejong::pretty_print(SE3_list_[i],se3,"","");
    sejong::pretty_print(inv_SE3_list_[i],inv_se3,"","");
    sejong::saveVector(apex_state_list_[i],"_apex_state_list");
    sejong::saveValue(apex_time_list_[i],"_apex_time_list");
    sejong::saveValue(switching_time_list_[i],"_switching_time_list");
  }
}

bool RRT_Analytic_Solver::_find_initial_root(const sejong::Vector & curr_com,
                                             const sejong::Vector & curr_foot,
                                             double nx_xp, double v_apex, RRT_Node * root,
                                             sejong::Vector & ini_state){
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

  double t_s = 1./omega * log (exp_omega_t);
  root->switching_time_ = floor(t_s*1000)*0.001;
  switching_time_list_.push_back(root->switching_time_);

  ini_state[4] = omega * (A * exp(omega * t_s) - B * exp(-omega * t_s));

  double xp(curr_foot[0]);
  double x0(curr_com[0]);
  double x0dot(curr_com[3]);

  double A_x(0.5 * ((x0 - xp) + 1./omega * x0dot ) );
  double B_x(0.5 * ((x0 - xp) - 1./omega * x0dot ) );

  ini_state[0] = A_x * exp(omega * t_s) + B_x * exp(-omega  * t_s) + xp;
  ini_state[3] = omega * (A_x * exp(omega * t_s) - B_x * exp(-omega  * t_s) );
  ini_state[2] = 1.0758;
  ini_state[5] = 0.;

  double x1(ini_state[0]);
  double x1dot(ini_state[3]);

  double A_nx(0.5 * ((x1 - nx_xp) + 1./omega * x1dot ) );
  double B_nx(0.5 * ((x1 - nx_xp) - 1./omega * x1dot ) );

  double t_apex = 1./(2 * omega) * log  ( - B_nx/ A_nx);
  root->apex_time_ = floor(t_apex*1000)*0.001;
  apex_time_list_.push_back(root->apex_time_);

  sejong::Vect3 nx_foot;
  nx_foot[0] = nx_xp;
  nx_foot[1] = _find_yp(ini_state[1], ini_state[4], t_apex);
  nx_foot[2] = 0.;

  root->foot_curr_frame_.tail(2) = nx_foot.tail(2);
  root->foot_glob_.head(3) = nx_foot;
  root->foot_glob_[3] = root->theta_;
  foot_list_.push_back(root->foot_glob_);

  return true;
}

double RRT_Analytic_Solver::_find_yp( double y0, double y0dot, double t_apex){
  double yp ( y0 - (1 + exp( 2* omega * t_apex))/ (1 - exp( 2* omega * t_apex)) * (y0dot/ omega) );
  return yp;
}

void RRT_Analytic_Solver::_get_yp_vel_constraint(double y0, double ydot0, double t_apex, double ydot_apex, double &yp){
  double C = 0.5 * ( y0 + 1/omega * ydot0 );
  double D = 0.5 * ( y0 - 1/omega * ydot0 );
  double X = omega * C * exp(omega*t_apex) -  omega * D * exp(-omega*t_apex);
  double Y = 0.5 * (omega * exp(-omega*t_apex) - omega * exp(omega*t_apex));
  yp = (ydot_apex - X)/Y;
}

void RRT_Analytic_Solver::_find_y_switching_state(double y0, double y0dot,
                                                  double t_switch,
                                                  double yp,
                                                  double & sw_pos,
                                                  double & sw_vel){
  double A (0.5 * ( (y0 - yp) + 1./omega * y0dot));
  double B (0.5 * ( (y0 - yp) - 1./omega * y0dot));

  sw_pos = A * exp(omega * t_switch) + B * exp(-omega * t_switch) + yp;
  sw_vel = omega * ( A * exp(omega * t_switch) - B * exp (-omega * t_switch));
}

double RRT_Analytic_Solver::_find_time(double x, double xdot,
                                       double x0, double x0dot,
                                       double xp){
  double A ( 0.5 * ((x0-xp) + 1/omega * x0dot) );
  double time = 1/omega * log ( (x + 1/omega * xdot - xp)/(2*A) );
  return time;
}

double RRT_Analytic_Solver::_find_velocity(double x, double xp, double x0, double x0dot){
  double sign(1.0);
  if(x0dot < 0) sign = -1.0;

  double xdot = sign * sqrt( pow(omega,2) * ( pow(x - xp, 2) - pow(x0 - xp, 2) ) + pow(x0dot, 2) );

  return xdot;
}
double RRT_Analytic_Solver::_switching_state_x(double xp1, double xp2,
                                               double x0, double x0dot,
                                               double v_apex){
  double C ( pow(x0 - xp1, 2.) + (pow(v_apex, 2.) - pow(x0dot, 2.))/pow(omega, 2.) );

  double x_switch( 0.5 * (C / (xp2 - xp1) + (xp1 + xp2) ) );

  return x_switch;
}

void RRT_Analytic_Solver::_PreProcessing(Terrain* terrain,
                                         CoMSurface* com_surf){
  terrain_ = terrain;
  com_surf_ = com_surf;
}

void RRT_Analytic_Solver::_get_SE3(double x, double y, double theta, sejong::Matrix &SE3){
  SE3 << cos(theta), -sin(theta), x,
    sin(theta), cos(theta), y,
    0, 0, 1;
}

void RRT_Analytic_Solver::_get_inv_SE3(double x, double y, double theta, sejong::Matrix &SE3_inv){
  SE3_inv << cos(theta), sin(theta),-(x*cos(theta) + y*sin(theta)),
    -sin(theta), cos(theta), -(y*cos(theta) - x*sin(theta)),
    0, 0, 1;
}


void RRT_Analytic_Solver::_get_next_xp_in_prev_frame(sejong::Vector curr_node, sejong::Vector next_node, double & next_xp_in_prev_frame){
  sejong::Matrix SE3_next(3,3);
  sejong::Matrix SE3_inv_prev(3,3);
  sejong::Vector origin_loc(3);
  sejong::Vector sol(3);
  origin_loc.setZero();
  _get_inv_SE3(curr_node[0], curr_node[1], curr_node[2], SE3_inv_prev);
  _get_SE3(next_node[0], next_node[1], next_node[2], SE3_next);
  origin_loc[2] = 1.0;
  sol = SE3_inv_prev * SE3_next * origin_loc;
  next_xp_in_prev_frame = sol[0];
}

void RRT_Analytic_Solver::_get_curr_xp_in_prev_frame(RRT_Node * node){
  sejong::Vector origin_loc(3);
  sejong::Vector sol(3);
  origin_loc.setZero();
  origin_loc[2] = 1.0;
  sol = node->parent_->inv_SE3_ * node->SE3_ * origin_loc;
  node->foot_prev_frame_ = sol; //only sol[0] is valid
}

void RRT_Analytic_Solver::_get_apex_vel_decomp_in_prev_frame(RRT_Node * node){
  node->apex_vel_decomp_prev_frame_[0] = VEL;
  node->apex_vel_decomp_prev_frame_[1] = VEL * cos( node->theta_ - node->parent_->theta_ );
  node->apex_vel_decomp_prev_frame_[2] = VEL * sin( node->theta_ - node->parent_->theta_ );
}

void RRT_Analytic_Solver::_TF_to_global(int node_idx, sejong::Vector & com_full_state){
  sejong::Vector glob_xy(3);
  sejong::Vector local_xy(3);
  local_xy[0] = com_full_state[0];
  local_xy[1] = com_full_state[1];
  local_xy[2] = 1.;
  glob_xy = SE3_list_[node_idx] * local_xy;

  sejong::Vector glob_xy_dot(3);
  sejong::Vector local_xy_dot(3);
  sejong::Vector local_origin(3);
  local_origin.setZero();
  local_origin[2] = 1.;
  local_xy_dot[0] = com_full_state[3];
  local_xy_dot[1] = com_full_state[4];
  local_xy_dot[2] = 1.;
  glob_xy_dot = SE3_list_[node_idx] * (local_xy_dot - local_origin);

  sejong::Vector glob_xy_ddot(3);
  sejong::Vector local_xy_ddot(3);
  local_origin.setZero();
  local_origin[2] = 1.;
  local_xy_ddot[0] = com_full_state[6];
  local_xy_ddot[1] = com_full_state[7];
  local_xy_ddot[2] = 1.;
  glob_xy_ddot = SE3_list_[node_idx] * (local_xy_ddot - local_origin);

  com_full_state[0] = glob_xy[0];
  com_full_state[1] = glob_xy[1];
  com_full_state[3] = glob_xy_dot[0];
  com_full_state[4] = glob_xy_dot[1];
  com_full_state[6] = glob_xy_ddot[0];
  com_full_state[7] = glob_xy_ddot[1];
}

void RRT_Analytic_Solver::_TF_to_global(RRT_Node * node, sejong::Vector & com_full_state){
  sejong::Vector glob_xy(3);
  sejong::Vector local_xy(3);
  local_xy[0] = com_full_state[0];
  local_xy[1] = com_full_state[1];
  local_xy[2] = 1.;
  glob_xy = node->parent_->SE3_ * local_xy;

  sejong::Vector glob_xy_dot(3);
  sejong::Vector local_xy_dot(3);
  sejong::Vector local_origin(3);
  local_origin.setZero();
  local_origin[2] = 1.;
  local_xy_dot[0] = com_full_state[3];
  local_xy_dot[1] = com_full_state[4];
  local_xy_dot[2] = 1.;
  glob_xy_dot = node->parent_->SE3_ * (local_xy_dot - local_origin);

  sejong::Vector glob_xy_ddot(3);
  sejong::Vector local_xy_ddot(3);
  local_origin.setZero();
  local_origin[2] = 1.;
  local_xy_ddot[0] = com_full_state[6];
  local_xy_ddot[1] = com_full_state[7];
  local_xy_ddot[2] = 1.;
  glob_xy_ddot = node->parent_->SE3_ * (local_xy_ddot - local_origin);

  com_full_state[0] = glob_xy[0];
  com_full_state[1] = glob_xy[1];
  com_full_state[3] = glob_xy_dot[0];
  com_full_state[4] = glob_xy_dot[1];
  com_full_state[6] = glob_xy_ddot[0];
  com_full_state[7] = glob_xy_ddot[1];
}

double RRT_Analytic_Solver::Get_Cost(std::vector<RRT_Node *>& node_vector){
  double tot_cost(0);
  Get_param(node_vector);
  std::vector<RRT_Node*>::iterator iter;
  for(iter = node_vector.begin() + 1; iter != node_vector.end(); ++iter){
    tot_cost += (*iter)->cost_to_parent_;
  }
  return tot_cost;
}

//previous get param
// void RRT_Analytic_Solver::Get_param(std::vector<RRT_Node *>& node_vector){
//   std::vector<RRT_Node*>::iterator iter;
//   sejong::Vector full_apex_state(9);
//   sejong::Vector apex_xy(3);
//   sejong::Vector apex_xy_next_frame(3);
//   sejong::Vector foot_xy(3);
//   sejong::Vector foot_xy_next_frame(3);

//   for(iter = node_vector.begin() + 1; iter != node_vector.end(); ++iter){
//     (*iter)->parent_ = *(iter - 1);
//     (*iter)->initial_state_prev_frame_ = sejong::Vector::Zero(6);
//     (*iter)->foot_curr_frame_ = sejong::Vector::Zero(3);
//     (*iter)->foot_prev_frame_ = sejong::Vector::Zero(3);
//     (*iter)->apex_state_curr_frame_ = sejong::Vector::Zero(4);
//     (*iter)->foot_glob_ = sejong::Vector::Zero(4);
//     (*iter)->SE3_ = sejong::Matrix(3,3);
//     (*iter)->inv_SE3_ = sejong::Matrix(3,3);
//     (*iter)->idx_ = (*iter)->parent_->idx_ + 1;
//     (*iter)->apex_vel_decomp_prev_frame_ = sejong::Vector::Zero(3);

//     _get_SE3((*iter)->x_, (*iter)->y_, (*iter)->theta_, (*iter)->SE3_);
//     _get_inv_SE3((*iter)->x_, (*iter)->y_, (*iter)->theta_, (*iter)->inv_SE3_);
//     _get_curr_xp_in_prev_frame(*iter);
//     _get_apex_vel_decomp_in_prev_frame(*iter);

//     (*iter)->initial_state_prev_frame_[0] = _switching_state_x((*iter)->parent_->foot_curr_frame_[0], (*iter)->foot_prev_frame_[0], (*iter)->parent_->apex_state_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[2], (*iter)->apex_vel_decomp_prev_frame_[1]);

//     (*iter)->initial_state_prev_frame_[3] = _find_velocity((*iter)->initial_state_prev_frame_[0], (*iter)->parent_->foot_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[2]);

//     (*iter)->switching_time_ = _find_time((*iter)->initial_state_prev_frame_[0], (*iter)->initial_state_prev_frame_[3], (*iter)->parent_->apex_state_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[2], (*iter)->parent_->foot_curr_frame_[0]);

//     (*iter)->apex_time_ = _find_time((*iter)->foot_prev_frame_[0], (*iter)->apex_vel_decomp_prev_frame_[1], (*iter)->initial_state_prev_frame_[0], (*iter)->initial_state_prev_frame_[3], (*iter)->foot_prev_frame_[0]);

//     (*iter)->cost_to_parent_ = (*iter)->switching_time_+(*iter)->apex_time_;

//     _find_y_switching_state((*iter)->parent_->apex_state_curr_frame_[1], (*iter)->parent_->apex_state_curr_frame_[3], (*iter)->switching_time_, (*iter)->parent_->foot_curr_frame_[1], (*iter)->initial_state_prev_frame_[1], (*iter)->initial_state_prev_frame_[4]);

//     _get_yp_vel_constraint((*iter)->initial_state_prev_frame_[1], (*iter)->initial_state_prev_frame_[4], (*iter)->apex_time_, (*iter)->apex_vel_decomp_prev_frame_[2], (*iter)->foot_prev_frame_[1]);


//     _get_apex_state_in_glob_frame((*iter), full_apex_state);
//     apex_xy = full_apex_state.head(3);
//     apex_xy[2] = 1.;
//     apex_xy_next_frame = (*iter)->inv_SE3_ * apex_xy;
//     (*iter)->apex_state_curr_frame_.head(2) = apex_xy_next_frame.head(2);
//     (*iter)->apex_state_curr_frame_[2] = (*iter)->apex_vel_decomp_prev_frame_[0];
//     (*iter)->apex_state_curr_frame_[3] = 0.;

//     (*iter)->foot_prev_frame_[2] = 1.;
//     (*iter)->foot_glob_.head(3) = (*iter)->parent_->SE3_ * (*iter)->foot_prev_frame_;
//     (*iter)->foot_glob_[2] = 0.;
//     (*iter)->foot_glob_[3] = (*iter)->theta_;

//     foot_xy = (*iter)->foot_glob_.head(3);
//     foot_xy[2] = 1.;
//     foot_xy_next_frame = (*iter)->inv_SE3_ * foot_xy;
//     (*iter)->foot_curr_frame_ = foot_xy_next_frame;
//   }
// }

void RRT_Analytic_Solver::Get_param(std::vector<RRT_Node *>& node_vector){
  std::vector<RRT_Node*>::iterator iter;
  sejong::Vector full_apex_state(9);
  sejong::Vector apex_xy(3);
  sejong::Vector apex_xy_vel(3);
  sejong::Vector apex_xy_next_frame(3);
  sejong::Vector foot_xy(3);
  sejong::Vector foot_xy_next_frame(3);

  for(iter = node_vector.begin() + 1; iter != node_vector.end(); ++iter){
    (*iter)->parent_ = *(iter - 1);
    (*iter)->initial_state_prev_frame_ = sejong::Vector::Zero(6);
    (*iter)->foot_curr_frame_ = sejong::Vector::Zero(3);
    (*iter)->foot_prev_frame_ = sejong::Vector::Zero(3);
    (*iter)->apex_state_curr_frame_ = sejong::Vector::Zero(4);
    (*iter)->foot_glob_ = sejong::Vector::Zero(4);
    (*iter)->SE3_ = sejong::Matrix(3,3);
    (*iter)->inv_SE3_ = sejong::Matrix(3,3);
    (*iter)->idx_ = (*iter)->parent_->idx_ + 1;
    (*iter)->apex_vel_decomp_prev_frame_ = sejong::Vector::Zero(3);

    _get_SE3((*iter)->x_, (*iter)->y_, (*iter)->theta_, (*iter)->SE3_);
    _get_inv_SE3((*iter)->x_, (*iter)->y_, (*iter)->theta_, (*iter)->inv_SE3_);
    _get_curr_xp_in_prev_frame(*iter);
    _get_apex_vel_decomp_in_prev_frame(*iter);

    (*iter)->initial_state_prev_frame_[0] = _switching_state_x((*iter)->parent_->foot_curr_frame_[0], (*iter)->foot_prev_frame_[0], (*iter)->parent_->apex_state_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[2], (*iter)->apex_vel_decomp_prev_frame_[1]);

    (*iter)->initial_state_prev_frame_[3] = _find_velocity((*iter)->initial_state_prev_frame_[0], (*iter)->parent_->foot_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[2]);

    (*iter)->switching_time_ = _find_time((*iter)->initial_state_prev_frame_[0], (*iter)->initial_state_prev_frame_[3], (*iter)->parent_->apex_state_curr_frame_[0], (*iter)->parent_->apex_state_curr_frame_[2], (*iter)->parent_->foot_curr_frame_[0]);

    (*iter)->apex_time_ = _find_time((*iter)->foot_prev_frame_[0], (*iter)->apex_vel_decomp_prev_frame_[1], (*iter)->initial_state_prev_frame_[0], (*iter)->initial_state_prev_frame_[3], (*iter)->foot_prev_frame_[0]);

    (*iter)->cost_to_parent_ = (*iter)->switching_time_+(*iter)->apex_time_;

    //test
    if((*iter)->cost_to_parent_<0){
      std::cout<<"==============this node==============="<<std::endl;
      std::cout<<"parent x,y,theat : "<<(*iter)->parent_->x_<<" "<<(*iter)->parent_->y_<<" "<<(*iter)->parent_->theta_<<std::endl;
      std::cout<<"x,y,theat : "<<(*iter)->x_<<" "<<(*iter)->y_<<" "<<(*iter)->theta_<<std::endl;
      sejong::pretty_print((*iter)->parent_->apex_state_curr_frame_, std::cout, "parent apex state curr frame");
      std::cout<<"foot prev frame (x) : "<<(*iter)->foot_prev_frame_[0]<<std::endl;
      std::cout<<"apex vel decomp prev frame (x) : "<<(*iter)->apex_vel_decomp_prev_frame_[1]<<std::endl;
      std::cout<<"switching_time : "<<(*iter)->switching_time_<<std::endl;
      std::cout<<"apex time : "<<(*iter)->apex_time_<<std::endl;
      std::cout<<"==============this node end==============="<<std::endl;
      exit(0);
    }
    //test end

    _find_y_switching_state((*iter)->parent_->apex_state_curr_frame_[1], (*iter)->parent_->apex_state_curr_frame_[3], (*iter)->switching_time_, (*iter)->parent_->foot_curr_frame_[1], (*iter)->initial_state_prev_frame_[1], (*iter)->initial_state_prev_frame_[4]);

    //test
    if( std::abs((*iter)->initial_state_prev_frame_[4]) < y_dot_des_ * cos((*iter)->theta_ - (*iter)->parent_->theta_)){
      //printf("computed v sw: %f \n",std::abs((*iter)->initial_state_prev_frame_[4]) );
      // printf("adjusted v sw: %f \n",y_dot_des_ * cos((*iter)->theta_ - (*iter)->parent_->theta_) );

      if( (*iter)->initial_state_prev_frame_[4] < 0){
        //(*iter)->initial_state_prev_frame_[4] = - y_dot_des_ * cos((*iter)->theta_ - (*iter)->parent_->theta_);

        (*iter)->initial_state_prev_frame_[4] += 0.6 * (- y_dot_des_ * cos((*iter)->theta_ - (*iter)->parent_->theta_) - (*iter)->initial_state_prev_frame_[4] );

      } else {
        //(*iter)->initial_state_prev_frame_[4] = y_dot_des_ * cos((*iter)->theta_ - (*iter)->parent_->theta_);
        (*iter)->initial_state_prev_frame_[4] += 0.6 * (y_dot_des_ * cos((*iter)->theta_ - (*iter)->parent_->theta_) - (*iter)->initial_state_prev_frame_[4] );

      }
      //printf("adjusted v sw: %f \n",std::abs((*iter)->initial_state_prev_frame_[4]) );
      //std::cout<<"adjusted"<<std::endl;
    }
    //test end

    _get_yp_vel_constraint((*iter)->initial_state_prev_frame_[1], (*iter)->initial_state_prev_frame_[4], (*iter)->apex_time_, (*iter)->apex_vel_decomp_prev_frame_[2], (*iter)->foot_prev_frame_[1]);


    (*iter)->foot_prev_frame_[2] = 1.;
    (*iter)->foot_glob_.head(3) = (*iter)->parent_->SE3_ * (*iter)->foot_prev_frame_;
    (*iter)->foot_glob_[2] = 0.;
    (*iter)->foot_glob_[3] = (*iter)->theta_;

    foot_xy = (*iter)->foot_glob_.head(3);
    foot_xy[2] = 1.;
    foot_xy_next_frame = (*iter)->inv_SE3_ * foot_xy;
    (*iter)->foot_curr_frame_ = foot_xy_next_frame;

    _get_apex_state_in_glob_frame((*iter), full_apex_state);
    apex_xy = full_apex_state.head(3);
    apex_xy[2] = 1.;
    apex_xy_next_frame = (*iter)->inv_SE3_ * apex_xy;
    (*iter)->apex_state_curr_frame_.head(2) = apex_xy_next_frame.head(2);
    // (*iter)->apex_state_curr_frame_[2] = (*iter)->apex_vel_decomp_prev_frame_[0];
    // (*iter)->apex_state_curr_frame_[3] = 0.;

    //test
    apex_xy_vel[0] = full_apex_state[3];
    apex_xy_vel[1] = full_apex_state[4];
    apex_xy_vel[2] = 0.;
    (*iter)->apex_state_curr_frame_.tail(2) = ((*iter)->inv_SE3_ * apex_xy_vel).head(2);
    //test end
  }
}

void RRT_Analytic_Solver::Set_root_node(RRT_Param* param, RRT_Node *root){
  _clear_vector();
  VEL = param->apex_vel;
  y_dot_des_ = param->y_dot_des;
  foot_list_.push_back(param->fixed_pivot);
  initial_state_list_.push_back(param->curr_com_state);

  root->SE3_ = sejong::Matrix(3,3);
  root->inv_SE3_ = sejong::Matrix(3,3);
  root->initial_state_prev_frame_ = sejong::Vector::Zero(6);
  root->foot_curr_frame_ = sejong::Vector::Zero(3);
  root->apex_state_curr_frame_ = sejong::Vector::Zero(4);
  root->foot_glob_ = sejong::Vector::Zero(4);

  root->x_ = param->root_x;
  root->y_ = param->root_y;
  root->theta_ = param->root_theta;
  root->idx_ = 1;
  _get_SE3(root->x_, root->y_, root->theta_, root->SE3_);
  _get_inv_SE3(root->x_, root->y_, root->theta_, root->inv_SE3_);
  SE3_list_.push_back(root->SE3_);
  inv_SE3_list_.push_back(root->inv_SE3_);

  omega = sqrt(9.81/param->curr_com_state[2]);

  sejong::Vector ini_state(6);
  sejong::Vector full_apex_state(9);
  double xp1;

  _find_initial_root(param->curr_com_state, param->fixed_pivot,
                     root->x_, param->apex_vel, root,
                     ini_state);

  root->initial_state_prev_frame_ = ini_state;
  initial_state_list_.push_back(ini_state);

  GetCoMState(1,root->apex_time_ , full_apex_state);
  root->apex_state_curr_frame_<< 0, full_apex_state[1], full_apex_state[3], full_apex_state[4];
  apex_state_list_.push_back(root->apex_state_curr_frame_);
  std::cout<<"[ROOT NODE] SETTED"<<std::endl;
}

void RRT_Analytic_Solver::_clear_vector(){
  foot_list_.clear();
  initial_state_list_.clear();
  SE3_list_.clear();
  inv_SE3_list_.clear();
  next_pivot_list_.clear();
  curr_pivot_list_.clear();
  apex_vel_decomp_.clear();
  switching_time_list_.clear();
  apex_time_list_.clear();
  apex_state_list_.clear();
}

void RRT_Analytic_Solver::_node_print(RRT_Node* node){
  std::cout<<"=============================================="<<std::endl;
  std::cout<<"node's idx: "<<node->idx_<<std::endl;
  std::cout<<"node's x: "<<node->x_<<std::endl;
  std::cout<<"node's y: "<<node->y_<<std::endl;
  std::cout<<"node's theta: "<<node->theta_<<std::endl;
  std::cout<<"node's SE3: "<<"\n"<<node->SE3_<<std::endl;
  std::cout<<"node's inv_SE3: "<<"\n"<<node->inv_SE3_<<std::endl;
  std::cout<<"node's switching_time: "<<node->switching_time_<<std::endl;
  std::cout<<"node's apex_time: "<<node->apex_time_<<std::endl;
  std::cout<<"node's foot_curr_frame: "<<"\n"<<node->foot_curr_frame_<<std::endl;
  std::cout<<"node's apex_state_curr_frame: "<<"\n"<<node->apex_state_curr_frame_<<std::endl;
  std::cout<<"node's foot_glob: "<<"\n"<<node->foot_glob_<<std::endl;
  std::cout<<"=============================================="<<std::endl;
}

void RRT_Analytic_Solver::_get_apex_state_in_glob_frame(RRT_Node* node, sejong::Vector & apex_full_state){
  apex_full_state = sejong::Vector::Zero(9);

  double foot_pos[2];
  foot_pos[0] = node->foot_prev_frame_[0];
  foot_pos[1] = node->foot_prev_frame_[1];

  double A[2];
  double B[2];
  A[0] = 0.5 * ( (node->initial_state_prev_frame_[0] - foot_pos[0]) + 1./omega * node->initial_state_prev_frame_[3]);
  B[0] = 0.5 * ( (node->initial_state_prev_frame_[0] - foot_pos[0]) - 1./omega * node->initial_state_prev_frame_[3]);

  A[1] = 0.5 * ( (node->initial_state_prev_frame_[1] - foot_pos[1]) + 1./omega * node->initial_state_prev_frame_[4]);
  B[1] = 0.5 * ( (node->initial_state_prev_frame_[1] - foot_pos[1]) - 1./omega * node->initial_state_prev_frame_[4]);

  double time(node->apex_time_);

  for (int i(0); i < 2; ++i){
    apex_full_state[i] = A[i] * exp(omega*time) + B[i] * exp(-omega*time) + foot_pos[i];
    apex_full_state[3 + i] = omega * (A[i] * exp(omega*time) - B[i] * exp(-omega*time));
    apex_full_state[6 + i] = omega * omega * (apex_full_state[i] - foot_pos[i]);
  }
  apex_full_state[2] = 1.0758;
  apex_full_state[5] = 0.;
  apex_full_state[8] = 0.;

  _TF_to_global(node, apex_full_state);
}
void RRT_Analytic_Solver::_plotting_full_trajectory(RRT_Param* param){

  RRT_Node ttemp = RRT_Node(0,0,0);
  ttemp = *(param->root_node);

  sejong::Vector node_pos(3);
  sejong::Vector com_full_state(9);

  for(int i(1); i < num_sequence_; ++i){
    node_pos[0] = ttemp.x_;
    node_pos[1] = ttemp.y_;
    node_pos[2] = ttemp.theta_;
    sejong::saveVector(node_pos, "RRT_node");

    ttemp = *(ttemp.sol_child_);
  }

  for(int i(0); i < foot_list_.size(); ++i){
    sejong::saveVector(foot_list_[i], "RRT_foot");
  }

  for(int i(0); i < switching_time_list_[0]*1000; ++i){
    GetCoMState(0, i*0.001, com_full_state);
    sejong::saveVector(com_full_state, "RRT_com");
  }

  for(int i(0); i < num_sequence_; ++i){
    for(int j(0); j < 1000 * (switching_time_list_[i+1] + apex_time_list_[i]); ++j){
      GetCoMState(i+1, j*0.001, com_full_state);
      sejong::saveVector(com_full_state, "RRT_com");
    }
  }
  std::cout<<"[DATA SAVED FOR PLOTTING]"<<std::endl;

}

bool RRT_Analytic_Solver::Is_close_to_goal(RRT_Param *param, RRT_Node *node){
  if( (node->x_ - param->goal_x) * (node->x_ - param->goal_x) < 0.1*0.1 && (node->y_ - param->goal_y) * (node->y_ - param->goal_y) < 0.1*0.1 && (node->theta_ - param->goal_theta) * (node->theta_ - param->goal_theta) < M_PI*0.25*M_PI*0.25  )
    return true;
  else
    return false;
}

