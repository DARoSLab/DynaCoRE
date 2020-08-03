#ifndef FLAT_TERRAIN_H
#define FLAT_TERRAIN_H

#include "Terrain.h"

class FlatTerrain: public Terrain{
public:
    FlatTerrain();
    virtual ~FlatTerrain();

    virtual double getZp(double xp, double yp);
};

#endif
