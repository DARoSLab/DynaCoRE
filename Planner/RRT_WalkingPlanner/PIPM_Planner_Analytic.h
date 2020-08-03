#ifndef PIPM_PLANNER_ANALYTIC
#define PIPM_PLANNER_ANALYTIC

#include "Planner.h"

class AnalyticPL_Param : public PlanningParam{
public:
  //(x, y, z, xdot, ydot, zdot)
  sejong::Vector curr_com_state;
  std::vector<double> xp_list;
  std::vector<double> apex_vel_list;
  std::vector<double> theta_list;
  bool is_initial;
  sejong::Vect3 fixed_pivot; // (xp, yp, zp)
};

class Planner_Analytic : public Planner{
public:
  Planner_Analytic();
  virtual ~Planner_Analytic();

  virtual bool GetCoMState(int step_idx, double time, sejong::Vector & com_full_state) ;
  virtual double GetFinTime(int step_idx) ;
  virtual bool GetFootPlacement(int sequence_idx,
                                sejong::Vector & foot_placement);
  virtual bool GetOrientation(int sequence_idx, sejong::Quaternion &ori);

protected:
  sejong::Vector _2D_Rotate(const sejong::Vector & vec2,
                            double theta);
  sejong::Quaternion _Quaternion_RotZ(double theta);

  void _ChangeFrame(const sejong::Vector & state,
                    const sejong::Vect3 & offset,
                    double theta,
                    sejong::Vector & changed_state);
  void _ChangeFrameBack(const sejong::Vector & state,
                        const sejong::Vect3 & offset,
                        double theta,
                        sejong::Vector & changed_state);
  void _ChangeFrameBack(const sejong::Vect3 & pos,
                        const sejong::Vect3 & offset,
                        double theta,
                        sejong::Vect3 & changed_pos);


  virtual bool _SolvePlanning(PlanningParam* );

  void _SetAB(int idx, double * A, double * B);

  virtual void _PreProcessing(Terrain* terrain,
                              CoMSurface* com_surf);

  // Calculation Function
  bool _find_initial(const sejong::Vector & curr_com,
                     const sejong::Vect3 & curr_foot,
                     double nx_xp, double v_apex,
                     sejong::Vector & ini_state);

  bool _find_switching(double xp, double v_apex, double ydot_des,
                       const sejong::Vector & curr_state,
                       const sejong::Vect3 & curr_foot,
                       sejong::Vector & switching_state,
                       sejong::Vect3 & nx_foot,
                       double & switching_time);
  double _find_yp( double y0, double y0dot, double t_apex);
  double _find_yp( double y0, double y0dot, double t_apex,double ydot_des);

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





  std::vector<sejong::Vector> foot_list_;
  std::vector<sejong::Quaternion> ori_list_;

  std::vector<sejong::Vector> initial_state_list_;
  std::vector<double> switching_time_list_;

  int num_sequence_;
  double A[2]; // x, y
  double B[2]; // x, y
  double omega;
};

#endif
