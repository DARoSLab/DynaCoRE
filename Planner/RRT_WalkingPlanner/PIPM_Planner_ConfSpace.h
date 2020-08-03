#ifndef PIPM_PLANNER_CONFSPACE
#define PIPM_PLANNER_CONFSPACE

#include "Planner.h"
#include "RRT_Node.h"

class RRT_Param : public PlanningParam{
public:
  sejong::Vector curr_com_state;
  bool is_initial;
  sejong::Vector fixed_pivot;
  double root_x, root_y, root_theta, apex_vel;
  double goal_x, goal_y, goal_theta;
  std::vector< sejong::Vector > node_list; //delete this later
  RRT_Node * root_node;
  RRT_Node * sol_node; // The last solution node
  double y_dot_des;
  bool planning;
};

class RRT_Analytic_Solver : public Planner{
public:
  RRT_Analytic_Solver();
  virtual ~RRT_Analytic_Solver();

  virtual bool GetCoMState(int step_idx, double time, sejong::Vector & com_full_state);
  virtual double GetFinTime(int step_idx) ;
  virtual bool GetFootPlacement(int sequence_idx,
                                sejong::Vector & foot_placement);
  virtual bool GetOrientation(int sequence_idx, sejong::Quaternion& ori);
  
  void Set_root_node(RRT_Param* param, RRT_Node* root);
  void Get_param(std::vector<RRT_Node *>& node_vector);
  bool Is_close_to_goal(RRT_Param* param, RRT_Node* node);
  //First node should be on of the tree
  double Get_Cost(std::vector<RRT_Node*>& node_vector);

  int num_sequence_;
  std::vector<sejong::Vector> foot_list_;//num_nodes + 1 (w.r.t global_frame)
  std::vector<sejong::Vector> apex_vel_decomp_;//num_nodes - 1 [0] : size, [1] : x, [2] : y
  std::vector<sejong::Vector> next_pivot_list_;//num_nodes - 1 (w.r.t prev_local_frame)
  std::vector<sejong::Vector> curr_pivot_list_;//num_nodes - 1 (w.r.t curr_local_frame)
  std::vector<sejong::Matrix> SE3_list_;//num_nodes
  std::vector<sejong::Matrix> inv_SE3_list_;//num_nodes
  std::vector<sejong::Vector> initial_state_list_;//num_nodes + 1 (w.r.t prev_local_frame)
  std::vector<sejong::Vector> apex_state_list_;//num_nodes (w.r.t next_local_frame)
  std::vector<double> apex_time_list_;//num_nodes
  std::vector<double> switching_time_list_;//num_nodes
  void _read_param_from_text(double curr_com_state);

protected:
  virtual bool _SolvePlanning(PlanningParam* );

  void _Set_local_AB_to_apex(int idx, double * A, double * B);
  void _Set_local_AB_to_peak(int idx, double * A, double * B);
  void _SetAB(int idx, double * A, double * B);

  virtual void _PreProcessing(Terrain* terrain,
                              CoMSurface* com_surf);

  // Calculation Function
  bool _find_initial_root(const sejong::Vector & curr_com,
                          const sejong::Vector & curr_foot,
                          double nx_xp, double v_apex, RRT_Node * root,
                          sejong::Vector & ini_state);

  double _find_yp( double y0, double y0dot, double t_apex);

  void _find_y_switching_state(double y0, double y0dot,
                               double t_switch,
                               double yp,
                               double & sw_pos,
                               double & sw_vel);

  double _find_time(double x, double xdot,
                    double x0, double x0dot, double xp);

  double _find_velocity(double x, double xp, double x0, double x0dot);

  double _switching_state_x(double xp1, double xp2,
                            double x0, double x0dot,
                            double v_apex);

  void _clear_vector();

  void _get_SE3(double x, double y, double theta, sejong::Matrix & SE3);
  void _get_inv_SE3(double x, double y, double theta, sejong::Matrix & SE3);
  void _get_next_xp_in_prev_frame(sejong::Vector curr_node, sejong::Vector next_node, double & next_xp_in_prev_frame);
  void _get_curr_xp_in_prev_frame(RRT_Node * node);
  void _get_apex_vel_decomp_in_prev_frame(RRT_Node * node);
  void _get_yp_vel_constraint(double y0, double ydot0, double t_apex, double ydot_apex, double & yp);
  void _TF_to_global(int node_idx, sejong::Vector & com_full_state);
  void _TF_to_global(RRT_Node* node, sejong::Vector & com_full_state);
  void _plotting_full_trajectory(RRT_Param* param);
  void _get_apex_state_in_glob_frame(RRT_Node* node, sejong::Vector & apex_full_state);
  void _node_print(RRT_Node* node);
  void _save_param();

  // std::vector<sejong::Vector> foot_list_;//num_nodes + 1 (w.r.t global_frame)
  // std::vector<sejong::Vector> apex_vel_decomp_;//num_nodes - 1 [0] : size, [1] : x, [2] : y
  // std::vector<sejong::Vector> next_pivot_list_;//num_nodes - 1 (w.r.t prev_local_frame)
  // std::vector<sejong::Vector> curr_pivot_list_;//num_nodes - 1 (w.r.t curr_local_frame)
  // std::vector<sejong::Matrix> SE3_list_;//num_nodes
  // std::vector<sejong::Matrix> inv_SE3_list_;//num_nodes
  // std::vector<sejong::Vector> initial_state_list_;//num_nodes + 1 (w.r.t prev_local_frame)
  // std::vector<sejong::Vector> apex_state_list_;//num_nodes (w.r.t next_local_frame)
  // std::vector<double> apex_time_list_;//num_nodes
  // std::vector<double> switching_time_list_;//num_nodes

  //int num_sequence_;
  double A[2]; // x, y
  double B[2]; // x, y
  double omega;
  double VEL;
  double yp_min, yp_max;
  double y_dot_des_;
};

#endif

