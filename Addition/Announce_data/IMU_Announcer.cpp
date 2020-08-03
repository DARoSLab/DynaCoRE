#include "IMU_Announcer.h"
#include "announcer_protocol.h"
#include "Utils/comm_udp.h"

IMU_broadcast::IMU_broadcast(): Sejong_Thread(), socket_receive_(NULL), socket_send_(NULL){
}

void IMU_broadcast::run(void ){
    ann_protocol::imu_data data;
    static int iter = 0;
    while(true){
        COMM::receive_data(socket_receive_, PORT_HUME_SYSTEM_IMU, &data, sizeof(ann_protocol::imu_data), IP_ADDR_MYSELF);
        if(iter%100 == 1){
            printf("angular velocity (yaw, pitch, roll): %f, %f, %f \n",
                   data.ang_vel[0], data.ang_vel[1], data.ang_vel[2]);
            printf( "linear acceleration (x, y, z): %f, %f, %f \n",
                    data.lin_acc[0], data.lin_acc[1], data.lin_acc[2]);
        }
        COMM::send_data(socket_send_, PORT_ANNOUNCE_IMU, &data, sizeof(ann_protocol::imu_data), ip_addr_);
        ++iter;
	pthread_testcancel();
    }
}
