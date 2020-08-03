#include "Obstacle.h"
#include <Utils/utilities.h>

MovementParam::MovementParam()
{

  axis_ = X_AXIS;
  joint_ = REVOLUTE;
  plan_ = LINEAR;
  xyz_axis_offset_ = sejong::Vect3::Zero();
  velocity_ = M_PI/4.;
  offset_   = 0.;
}

MovementParam::MovementParam(movement_axis axis, joint_type joint, motion_plan plan,
              sejong::Vect3 xyz_axis_offset, double plan_amp_or_vel, double plan_freq_or_offset )
{
  axis_  = axis;
  joint_ = joint;
  plan_  = plan;
  xyz_axis_offset_ = xyz_axis_offset;

  switch (plan)
  {
    case (SINUSOID):
      amplitude_ = plan_amp_or_vel;
      frequency_ = plan_freq_or_offset;
      offset_    = 0.;
      break;
    case (LINEAR):
      velocity_ = plan_amp_or_vel;
      offset_ = plan_freq_or_offset;
      break;
  }
}

MovementParam::MovementParam(movement_axis axis, joint_type joint, motion_plan plan,
              sejong::Vect3 xyz_axis_offset, double plan_amp, double plan_freq, double plan_offset )
{
  axis_  = axis;
  joint_ = joint;
  plan_  = plan;
  xyz_axis_offset_ = xyz_axis_offset;

  switch (plan)
  {
    case (SINUSOID):
      amplitude_ = plan_amp;
      frequency_ = plan_freq;
      offset_    = plan_offset;
      break;
    case (LINEAR): // Should never happen
      velocity_ = 1.;
      offset_ = 1.;
      break;
  }
}

Obstacle::Obstacle()
{
  IsStatic = false;
  init_pos_ = sejong::Vect3::Zero();
  lwh_      = sejong::Vect3::Ones();
}

Obstacle::Obstacle(const sejong::Vect3& init_pos, const sejong::Vect3& lwh)
{
  IsStatic = false;
  init_pos_ = init_pos;
  lwh_ = lwh;
}

Obstacle::~Obstacle(){

}


void Obstacle::add_movement(const MovementParam& param) {
  movements.push_back(param);
}


void Obstacle::get_transform(const double& sim_time_, sejong::Transform& tf) const {
  tf = sejong::Transform::Identity();
  double pos_adj;

  // std::vector<const MovementParam>::iterator it;
  // for (it = movements.begin(); it < movements.end(); ++it)
  // {

  //   switch (it->plan_)
  //   {
  //     case MovementParam::SINUSOID:
  //       pos_adj = it->amplitude_ * ( 1.-cos( it->frequency_ * sim_time_ + it->offset_ ) );
  //       break;
  //     case MovementParam::LINEAR:
  //       pos_adj = it->velocity_ * sim_time_ + it->offset_;
  //       break;    }

  //   switch (it->joint_)
  //   {
  //     case MovementParam::REVOLUTE:
  //       tf *= Eigen::Translation<double,3>( init_pos_ + it->xyz_axis_offset_);
  //       tf *= Eigen::AngleAxis<double>(pos_adj, sejong::Vect3::Unit(it->axis_));
  //       tf *= Eigen::Translation<double,3>( -init_pos_ - it->xyz_axis_offset_);
  //       break;
  //     case MovementParam::PRISMATIC:
  //       tf *= Eigen::Translation<double,3>(pos_adj * sejong::Vect3::Unit(it->axis_));
  //       break;
  //   }
  // }

  for (int i(0); i < movements.size(); ++i)
  {
    switch (movements[i].plan_)
    {
      case MovementParam::SINUSOID:
        pos_adj = movements[i].amplitude_ * ( 1.-cos( movements[i].frequency_ * sim_time_ + movements[i].offset_ ) );
        break;
      case MovementParam::LINEAR:
        pos_adj = movements[i].velocity_ * sim_time_ + movements[i].offset_;
        break;    
    } 

    switch (movements[i].joint_)
    {
      case MovementParam::REVOLUTE:
        tf *= Eigen::Translation<double,3>( init_pos_ + movements[i].xyz_axis_offset_);
        tf *= Eigen::AngleAxis<double>(pos_adj, sejong::Vect3::Unit(movements[i].axis_));
        tf *= Eigen::Translation<double,3>( -init_pos_ - movements[i].xyz_axis_offset_);
        break;
      case MovementParam::PRISMATIC:
        tf *= Eigen::Translation<double,3>(pos_adj * sejong::Vect3::Unit(movements[i].axis_));
        break;
    }
  }
}


bool Obstacle::is_collision(const double& sim_time_, const double x, const double y, const double radius) const {
  sejong::Transform tf;
  get_transform(sim_time_, tf);
  tf = tf.inverse();

  sejong::Vect3 pos;
  pos << x, y, 0.;
  pos = tf * pos;

  double x_min_thre(init_pos_[0] - radius);
  double x_max_thre(init_pos_[0] + radius + lwh_[0]);
  double y_min_thre(init_pos_[1] - radius);
  double y_max_thre(init_pos_[1] + radius + lwh_[1]);

  // false = collision free
  // true  = collision occurs
  if( ((pos[0] - x_min_thre) * (pos[0] - x_max_thre) > 0) || ((pos[1] - y_min_thre) * (pos[1] - y_max_thre) > 0) )
    return false;
  else
    return true;
}


void Obstacle::get_init_corner( const int& index, sejong::Vect3& coordinates ) const
{
  switch(index)
  {
    case 0:  coordinates = init_pos_; break;
    case 1:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(1.,0.,0.) ); break;
    case 2:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(1.,1.,0.) ); break;
    case 3:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(0.,1.,0.) ); break;
    case 4:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(0.,0.,1.) ); break;
    case 5:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(1.,0.,1.) ); break;
    case 6:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(1.,1.,1.) ); break;
    case 7:  coordinates = init_pos_ + lwh_.cwiseProduct( sejong::Vect3(0.,1.,1.) ); break;
  }
}

void Obstacle::get_velocity( const double& sim_time_, sejong::Vect3& vel ){
  double dt(0.0001);
  sejong::Transform tf_1;
  sejong::Transform tf_2;
  get_transform(sim_time_, tf_1);
  get_transform(sim_time_+dt, tf_2);
  sejong::Vect3 delta_pos;
  sejong::Vect3 cent;
  get_centroid(cent);
  delta_pos = tf_2 * cent - tf_1 * cent;
  vel = delta_pos/dt;
}

void Obstacle::get_position( const double& sim_time_, sejong::Vector& pos ){
  pos.setZero();
  sejong::Transform tf;
  sejong::Vect3 delta_pos;
  sejong::Vect3 cent;
  get_transform(sim_time_, tf);
  get_centroid(cent);
  //pos.head(3) = tf * cent - cent;
  pos.head(3) = tf * cent;
  //r,p,y
  pos.tail(3) = (tf.rotation()).eulerAngles(0,1,2);
}

void Obstacle::get_centroid( sejong::Vect3& coordinates )
{
  coordinates = init_pos_ + lwh_/2.;
}



//
