#ifndef OBS_BUILD
#define OBS_BUILD

#include "LocomotionPlanner/EnvironmentSetup/Obstacle.h"
#include "Utils/wrap_eigen.hpp"
#include "Utils/utilities.h"


class ObstacleBuilder{
public:
  ObstacleBuilder();
  ~ObstacleBuilder();
  std::vector<Obstacle> get_obstacle_list();
  static void save_obs_vectors(const std::vector<Obstacle>&);
  static void save_obs_vectors(const std::vector<Obstacle> & , double sim_time, double step_size);

private:
  std::vector<Obstacle> obs_list_;

  void add_kiva_bots();
  void add_table();
  void add_rot_arm();

  sejong::Vect3 init_pos, lwh;
  MovementParam mov_param_1, mov_param_2, mov_param_3, mov_param_4;
  MovementParam mov_param_5, mov_param_6, mov_param_7, mov_param_8;
  Obstacle obs;
};


#endif
