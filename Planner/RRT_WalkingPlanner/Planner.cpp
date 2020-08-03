#include "Planner.h"
#include <Configuration.h>

Planner::Planner(){
}

Planner::~Planner(){
}
bool Planner::DoLocomotionPlanning(Terrain* terrain,
                                   CoMSurface* com_surf,
                                   PlanningParam * planner_param){
  _PreProcessing(terrain, com_surf);
  return _SolvePlanning(planner_param);
}
