#ifndef PLANNER_H
#define PLANNER_H

#include <Utils/wrap_eigen.hpp>
#include <vector>

class Terrain;
class CoMSurface;

class PlanningParam{
public:
};

class Planner{
public:
  Planner();
  virtual ~Planner();

  bool DoLocomotionPlanning(Terrain* terrain,
                            CoMSurface* com_surf,
                            PlanningParam * planner_param);

  //(x, y, z, xdot, ydot, zdot, xddot, yddot, zddot)
  virtual bool GetCoMState(int step_idx, double time, sejong::Vector & com_full_state) = 0;
  virtual double GetFinTime(int step_idx) = 0;
  virtual bool GetFootPlacement(int sequence_idx,
                                sejong::Vector & foot_placement) = 0;
  virtual bool GetOrientation(int sequence_idx, sejong::Quaternion &ori) = 0;

 protected:

  virtual bool _SolvePlanning(PlanningParam* ) = 0;
  virtual void _PreProcessing(Terrain* terrain,
                              CoMSurface* com_surf) = 0;

  Terrain* terrain_;
  CoMSurface* com_surf_;
};

#endif
