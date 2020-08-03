#ifndef RRT_PLANNER
#define RRT_PLANNER

#include <Utils/wrap_eigen.hpp>
#include "RRT_Node.h"
#include "Configuration.h"

class RRT_Analytic_Solver;
class Obstacle;

class RRT_Planner{
public:
  // Initialize with a solver to compute path's cost according to dynamically feasible times
  RRT_Planner(const sejong::Vector& search_boundary,
              const RRT_Node& start, const RRT_Node& goal,
              const std::vector<Obstacle>& obstacle_list,
              RRT_Analytic_Solver * solver);

  // If initialized without a solver, it defaults to computing cost via euclidean distance between nodes on dubin's path
  RRT_Planner(const sejong::Vector& search_boundary,
              const RRT_Node& start, const RRT_Node& goal,
              const std::vector<Obstacle>& obstacle_list);
  ~RRT_Planner();


  enum path_type { RSR, RSL, LSL, LSR, UNKNOWN };

  sejong::Vector search_boundary_; // 4 dim: x_min, x_max, y_min, y_max
  sejong::Vector histogram_cost_compare_;
  double max_turn_, x_p_, safety_margin_, rad;

  std::vector<Obstacle> obstacle_list_;

  std::vector<RRT_Node*> tree_;
  RRT_Node* goal_;

  bool build_RRT(const int& num_samples);
  void get_nearest_neighbor_path(const RRT_Node& sample, RRT_Node*& nearest_neighbor,
                                         std::vector<RRT_Node*>& path);
  void rand_sample(RRT_Node& rand_node);
  void prune_for_collisions(std::vector<RRT_Node*>& path);
  // bool collision_check(const std::vector<RRT_Node*>& path);
  void get_path(const RRT_Node& start, const RRT_Node& end, const bool& calc_nodes,
                std::vector<RRT_Node*>& path, double& cost, path_type& soln_type);
  // void get_path_check_obs(const RRT_Node& start, const RRT_Node& end, std::vector<RRT_Node*>& path, double& cost);
  void get_solution(std::vector<RRT_Node*>& solution) const;
  void shortcut_solution(const int num_iterations, std::vector<RRT_Node*>& solution);
  bool append_path_to_tree(std::vector<RRT_Node*>& new_branch,
                           RRT_Node* parent_on_tree );
  void delete_tree_except_solution();
  void print_path(std::vector<RRT_Node*>& nodes);

protected:

  void _Init(const sejong::Vector& search_boundary,
             const RRT_Node& start, const RRT_Node& goal,
             const std::vector<Obstacle>& obstacle_list);

  RRT_Analytic_Solver * analytic_solver_;
  static const int POS = 1;
  static const int NEG = -1;

  bool found_solution_;


  void _fix_angle(double& angle, int POS_or_NEG);
  void _get_nodes_on_path(const double& turn_0, sejong::Vector& center_0,
                          const double& turn_1, sejong::Vector& center_1,
                          const RRT_Node& start, const RRT_Node& end,
                          const double& radius, std::vector<RRT_Node*>& nodes);
  double _get_dubins_path_length(const double& turn_0, sejong::Vector& center_0,
                               const double& turn_1, sejong::Vector& center_1,
                               const RRT_Node& start, const RRT_Node& end,
                               const double& radius);
  double _tmp_get_cost(std::vector<RRT_Node*>& nodes);
  double _get_cost(std::vector<RRT_Node*>& nodes);
  double _get_cost_eucl(std::vector<RRT_Node*>& nodes);
  double _get_time(std::vector<RRT_Node*>& nodes);

  void _keep_min_delete_tmp(std::vector<RRT_Node*>&     path, double&     cost,
                            std::vector<RRT_Node*>& tmp_path, double& tmp_cost);


  bool _is_out_of_bounds(const RRT_Node* node);

  void _get_normal_dist(double& variate1, double& variate2);
  void _get_geometric_dist(const double& p, int& variate);
  void _bound_shortcut_length(int& shortcut_length, const int& min_shortcut, const int& max_shortcut);

};


#endif
