#include <stdio.h>
#include <iostream>
#include "IMU_Announcer.h"


int main (int argc, char ** argv){
    IMU_broadcast imu_cast;
    imu_cast.ip_addr_ = argv[1];
    
    imu_cast.start();
    while(true){
        usleep(1000);
    }
    return 0;
}
