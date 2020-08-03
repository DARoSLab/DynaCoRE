#ifndef OBSTACLE_SETTING
#define OBSTACLE_SETTING

#include <Utils/wrap_eigen.hpp>
#include <iostream>

class MovementParam{
public:
  enum movement_axis { X_AXIS, Y_AXIS, Z_AXIS };
  enum joint_type { PRISMATIC, REVOLUTE };
  enum motion_plan { LINEAR, SINUSOID };

  movement_axis axis_;
  joint_type    joint_;
  motion_plan   plan_;
  double amplitude_, frequency_;
  double velocity_, offset_;
  sejong::Vect3 xyz_axis_offset_;

  MovementParam();
  MovementParam(movement_axis axis, joint_type joint, motion_plan plan,
                sejong::Vect3 xyz_axis_offset, double plan_amp_or_vel, double plan_freq_or_offset );
  MovementParam(movement_axis axis, joint_type joint, motion_plan plan,
                sejong::Vect3 xyz_axis_offset, double plan_amp, double plan_freq, double plan_offset );
};

class Obstacle{
public:
  Obstacle();
  Obstacle(const sejong::Vect3& init_pos, const sejong::Vect3& lwh);
  ~Obstacle();

  void add_movement( const MovementParam& param );
  void get_transform( const double& sim_time_, sejong::Transform& t ) const;
  void get_velocity( const double& sim_tim_, sejong::Vect3& vel );
  void get_position( const double& sim_tim_, sejong::Vector& pos );
  bool is_collision( const double& sim_time_, const double x, const double y, const double radius) const;
  void get_init_corner ( const int& index, sejong::Vect3& coordinates ) const ;
  void get_centroid( sejong::Vect3& coordinates );

  bool IsStatic;
  sejong::Vect3 init_pos_, lwh_;
  std::vector<MovementParam> movements;
protected:


};

#endif
