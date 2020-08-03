#ifndef STAIR_HEIGHT_SURFACE_H
#define STAIR_HEIGHT_SURFACE_H

#include "CoMSurface.h"

class StairHeightSurface : public CoMSurface{
public:
  StairHeightSurface();
  virtual ~StairHeightSurface();

  virtual double getZ(double x, double y);
  virtual double getZdot(double x, double y, double xdot, double ydot);
  virtual double getZddot(double x, double y, double xdot, double ydot);

protected:
  double com_height_;
};


#endif
