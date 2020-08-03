#include "FlatTerrain.h"
#include <stdio.h>

FlatTerrain::FlatTerrain():Terrain(){
    printf("[Flat Terrain] Built\n");
}
FlatTerrain::~FlatTerrain(){
}

double FlatTerrain::getZp(double xp, double yp){
  return 0.0;
}
