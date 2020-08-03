#ifndef CONST_HEIGHT_SURFACE_H 
#define CONST_HEIGHT_SURFACE_H 

#include "CoMSurface.h"

class ConstHeightSurface : public CoMSurface{
public:
  ConstHeightSurface();
  virtual ~ConstHeightSurface();

  virtual double getZ(double x, double y);
  virtual double getZdot(double x, double y, double xdot, double ydot);
  virtual double getZddot(double x, double y, double xdot, double ydot);

  void SetCoMHeight(double height);

protected:
  double com_height_;
};


#endif
