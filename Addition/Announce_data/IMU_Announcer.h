#ifndef IMU_Announcer
#define IMU_Announcer

#include "Utils/Sejong_Thread.h"


class IMU_broadcast : public Sejong_Thread {public:
    virtual ~IMU_broadcast(){}
    
    virtual void run(void );
    IMU_broadcast();    
    char* ip_addr_;
protected:
    int socket_receive_;
    int socket_send_;
};

#endif

