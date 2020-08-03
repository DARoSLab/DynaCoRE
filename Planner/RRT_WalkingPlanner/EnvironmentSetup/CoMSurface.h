#ifndef CoM_SURFACE_H
#define CoM_SURFACE_H

class CoMSurface{
public:
  CoMSurface(){}
  virtual ~CoMSurface(){}

  virtual double getZ(double x, double y) = 0;
  virtual double getZdot(double x, double y, double xdot, double ydot) = 0;
  virtual double getZddot(double x, double y, double xdot, double ydot) = 0;
};

#endif
