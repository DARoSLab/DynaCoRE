#ifndef TELLO_DYNACORE_CONTROL_DEFINITION
#define TELLO_DYNACORE_CONTROL_DEFINITION

#include <TELLO/TELLO_Definition.h>

#define TELLOConfigPath THIS_COM"DynaController/TELLO_Controller/TELLOTestConfig/"

class TELLO_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];
        double jpos[tello::num_act_joint];
        double jvel[tello::num_act_joint];
        double jtorque[tello::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class TELLO_Command{
    public:
        double jtorque_cmd[tello::num_act_joint];
        double jpos_cmd[tello::num_act_joint];
        double jvel_cmd[tello::num_act_joint];
};


#endif
