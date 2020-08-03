#include "StairTerrain.h"
#include <stdio.h>

StairTerrain::StairTerrain():Terrain(){
  printf("[Stair Terrain] Built\n");
}
StairTerrain::~StairTerrain(){
}

double StairTerrain::getZp(double xp, double yp){
  return (xp/0.24 - 1)*0.1;
}
