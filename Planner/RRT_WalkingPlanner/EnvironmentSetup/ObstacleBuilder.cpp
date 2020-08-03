#include "ObstacleBuilder.h"



ObstacleBuilder::ObstacleBuilder()
{
  add_kiva_bots();
  add_table();
  add_rot_arm();
}

ObstacleBuilder::~ObstacleBuilder()
{}

std::vector<Obstacle> ObstacleBuilder::get_obstacle_list()
{
  return obs_list_;
}


void ObstacleBuilder::save_obs_vectors(const std::vector<Obstacle> &obs_list)
{
  save_obs_vectors(obs_list, 20.,0.05);
}

void ObstacleBuilder::save_obs_vectors(const std::vector<Obstacle> & obs_list, double sim_time, double step_size)
{
    sejong::Transform tf;

    sejong::Vector corners(9);
    sejong::Vect3 one_corner(3);

    corners.setConstant(obs_list.size());// First line should tell matlab how many obstacles to read per frame
    sejong::saveVector(corners, "obs_anim");  // First line should tell matlab how many obstacles to read per frame

    for(double t = 0.; t < sim_time; t += step_size)
    {
      for(std::vector<Obstacle>::const_iterator it = obs_list.begin(); it < obs_list.end(); ++it)
      {
        it->get_transform(t, tf);
        corners[0] = t;

        it->get_init_corner(0, one_corner);
        one_corner = tf * one_corner;
        corners[1] = one_corner[0];
        corners[2] = one_corner[1];

        it->get_init_corner(1, one_corner);
        one_corner = tf * one_corner;
        corners[3] = one_corner[0];
        corners[4] = one_corner[1];

        it->get_init_corner(2, one_corner);
        one_corner = tf * one_corner;
        corners[5] = one_corner[0];
        corners[6] = one_corner[1];

        it->get_init_corner(3, one_corner);
        one_corner = tf * one_corner;
        corners[7] = one_corner[0];
        corners[8] = one_corner[1];

        sejong::saveVector(corners, "obs_anim");
      }
    }
}

void ObstacleBuilder::add_kiva_bots()
{
  double amp(0.5);
  // double freq(8.*M_PI/10.);
  double freq(2.*M_PI/10.);
  double phase_lag(M_PI/10.);

  mov_param_1 = MovementParam(MovementParam::Y_AXIS, MovementParam::PRISMATIC, MovementParam::SINUSOID,
                              sejong::Vect3::Zero(), 2, freq , 0.);

  mov_param_2 = MovementParam(MovementParam::X_AXIS, MovementParam::PRISMATIC, MovementParam::SINUSOID,
                              sejong::Vect3::Zero(), 2., freq , 0.);

  mov_param_3 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::LINEAR, sejong::Vect3::Zero(), 0, M_PI/4);

  mov_param_4 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::LINEAR, sejong::Vect3::Zero(), 0, M_PI/2);

  mov_param_5 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::LINEAR, sejong::Vect3::Zero(), 0, -M_PI/4);

  init_pos << 2., -3., 0.;
  lwh      <<   0.445, 0.277, 0.17;
  obs = Obstacle( init_pos, lwh );
  obs.add_movement(mov_param_5);
  obs.add_movement(mov_param_2);
  obs_list_.push_back(obs);

  // init_pos << 15., -4., 0.;
  // lwh      <<   0.455, 0.277, 0.17;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_4);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

  // init_pos << 6.5, -0.5, 0.;
  // lwh      <<   0.445, 0.277, 0.17;
  // obs = Obstacle( init_pos, lwh );
  // mov_param_2.offset_ += phase_lag;
  // obs.add_movement(mov_param_3);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

  // init_pos << 9., 4., 0.;
  // lwh      <<   0.445, 0.277, 0.17;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_4);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

  // init_pos << 10., 0., 0.16;
  // lwh      <<   0.445, 0.277, 0.17;
  // mov_param_2.offset_ += phase_lag;
  // mov_param_2.offset_ += phase_lag;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_5);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

  //test
  // mov_param_5 = MovementParam(MovementParam::X_AXIS, MovementParam::PRISMATIC, MovementParam::LINEAR, sejong::Vect3::Zero(), 1., 0.);

  // mov_param_6 = MovementParam(MovementParam::Y_AXIS, MovementParam::PRISMATIC, MovementParam::SINUSOID, sejong::Vect3::Zero(), 0.5, M_PI/4.);

  // mov_param_7 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::SINUSOID, lwh/2, M_PI/8., M_PI/4.0, M_PI/4.0);

  // mov_param_8 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::LINEAR, lwh/2, 0, -M_PI/8.);

  // init_pos << 4., -0., 0.;
  // lwh      <<   0.445, 0.277, 0.17;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_5);
  // obs.add_movement(mov_param_6);
  // obs.add_movement(mov_param_7);
  // obs.add_movement(mov_param_8);
  // obs_list_.push_back(obs);
  //test


  // mov_param_1 = MovementParam(MovementParam::X_AXIS, MovementParam::PRISMATIC, MovementParam::LINEAR,
  //                             sejong::Vect3::Zero(), amp*freq, 0. );
  // mov_param_2 = MovementParam(MovementParam::X_AXIS, MovementParam::PRISMATIC, MovementParam::SINUSOID,
  //                             sejong::Vect3::Zero(), amp, freq , 0.);

  //pioneer2dx configuration: height 0.16, lwh = 0.445, 0.277, 0.17

  // init_pos << 12. , -4., 0.16;
  // lwh      <<   0.445, 0.277, 0.17;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_1);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

  // init_pos << 12., 0., 0.16;
  // lwh      <<   0.445, 0.277, 0.17;
  // mov_param_2.offset_ += phase_lag;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_1);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

  // init_pos << 12. ,4., 0.16;
  // lwh      <<   0.445, 0.277, 0.17;
  // mov_param_2.offset_ += phase_lag;
  // obs = Obstacle( init_pos, lwh );
  // obs.add_movement(mov_param_1);
  // obs.add_movement(mov_param_2);
  // obs_list_.push_back(obs);

}

void ObstacleBuilder::add_table()
{
  init_pos << -2., -1.5, 0.;
  lwh      << 11., 0.2, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);


  init_pos << 9., -2.5, 0.;
  lwh      << 0.2, 1.5, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);


  init_pos << 4.5, -4.5, 0.;
  lwh      << 7.5, 0.2, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << 5, -5.0, 0.;
  lwh      << 0.2, 1.9, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);


  init_pos << 1, -5.0, 0.;
  lwh      << 2.0, 0.2, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << -2., -8., 0.;
  lwh      << 2.0, 0.2, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << 0., -9.4, 0.;
  lwh      << 0.2, 2., 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);


  init_pos << -2, -12., 0.;
  lwh      << 3., 0.2, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << 5.2, -16., 0.;
  lwh      << 0.2, 4.5, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << 8., -12., 0.;
  lwh      << 4.0, 0.2, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << 9.0, -12., 0.;
  lwh      << 0.2, 6.0, 0.8;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);


  init_pos << 3.5, -9., 0.;
  lwh      << 1., 1., 2.;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

  init_pos << 3.5, -1.5, 0.;
  lwh      << 0.7, 1.6, 0.5;
  obs = Obstacle( init_pos, lwh );
  obs.IsStatic = true;
  obs_list_.push_back(obs);

}

void ObstacleBuilder::add_rot_arm()
{
  init_pos << 0., -14., 0.;
  lwh      <<   0.445, 0.277, 0.17;
  obs = Obstacle( init_pos, lwh );
  mov_param_1 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::SINUSOID, sejong::Vect3(lwh[0]/2,2,0.) , 2., 0.4, 0. );
  obs.add_movement(mov_param_1);
  obs_list_.push_back(obs);

  init_pos << 4.25, -10.75, 0.;
  lwh      <<   0.445, 0.277, 0.17;
  obs = Obstacle( init_pos, lwh );
  mov_param_1 = MovementParam(MovementParam::Z_AXIS, MovementParam::REVOLUTE, MovementParam::LINEAR, sejong::Vect3(lwh[0]/2,2,0.) , 0.3, 0. );
  obs.add_movement(mov_param_1);
  obs_list_.push_back(obs);
}
