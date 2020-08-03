#ifndef RRT_NODE
#define RRT_NODE

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.h>
#include <string>
#include <sstream>
#include <iomanip>

class RRT_Node{
public:
  int idx_;
  double x_, y_, theta_, cost_to_parent_, switching_time_, apex_time_;
  sejong::Vector initial_state_prev_frame_, foot_curr_frame_, foot_prev_frame_, apex_state_curr_frame_, foot_glob_, apex_vel_decomp_prev_frame_;
  sejong::Matrix SE3_, inv_SE3_;

  RRT_Node* parent_;
  RRT_Node* sol_child_;

  RRT_Node();
  // RRT_Node(const RRT_Node& n);
  RRT_Node(const double x, const double y, const double theta);
  ~RRT_Node(); // does not delete parent_ because parent_ could still be a valid part of the tree

  double get_cost_to_root() const;
  std::string to_string() const;
  void print_node() const;
  void save_node(std::string name) const;
  void save_node_2(std::string name) const;

  bool operator== (const RRT_Node &n) const;
  bool operator!= (const RRT_Node &n) const;

protected:
  static const double NULL_COST;// = 0.0;

};




#endif
