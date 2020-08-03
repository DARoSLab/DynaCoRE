#ifndef STAIR_TERRAIN_H
#define STAIR_TERRAIN_H

#include "Terrain.h"

class StairTerrain: public Terrain{
public:
  StairTerrain();
  virtual ~StairTerrain();

  virtual double getZp(double xp, double yp);
};

#endif
