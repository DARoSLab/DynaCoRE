#ifndef TERRAIN_H
#define TERRAIN_H

class Terrain{
public:
    Terrain(){}
    virtual ~Terrain(){}

    virtual double getZp(double xp, double yp) = 0;
};

#endif
