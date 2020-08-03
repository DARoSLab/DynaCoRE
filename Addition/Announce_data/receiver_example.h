#ifndef IMU_reciever
#define IMU_reciever

#include "Utils/Sejong_Thread.h"


class IMU_reciever : public Sejong_Thread {public:
    virtual ~IMU_reciever(){}
    
    virtual void run(void );
    IMU_reciever();    
    char* ip_addr_;
protected:
    int socket_recieve_;
};

#endif

