#include "Const_Height_Surface.h"

ConstHeightSurface::ConstHeightSurface(){

}
ConstHeightSurface::~ConstHeightSurface(){
}

double ConstHeightSurface::getZ(double x, double y){
  return com_height_;
}
double ConstHeightSurface::getZdot(double x, double y, double xdot, double ydot){
  return 0.0;
}

double ConstHeightSurface::getZddot(double x, double y, double xdot, double ydot){
  return 0.0;
}

void ConstHeightSurface::SetCoMHeight(double height){
  com_height_ = height;
}
