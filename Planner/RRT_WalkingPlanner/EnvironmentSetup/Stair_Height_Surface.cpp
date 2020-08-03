#include "Stair_Height_Surface.h"

StairHeightSurface::StairHeightSurface(){

}
StairHeightSurface::~StairHeightSurface(){
}

double StairHeightSurface::getZ(double x, double y){
  if(x>0.24){
    com_height_ = 1/2.4*x + 1.075;
  return com_height_;
  }
  return 1.075;
}
double StairHeightSurface::getZdot(double x, double y, double xdot, double ydot){
  return 0.0;
}

double StairHeightSurface::getZddot(double x, double y, double xdot, double ydot){
  return 0.0;
}
