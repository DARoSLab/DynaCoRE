#ifndef Cheetah3_DYNACORE_CONTROL_DEFINITION
#define Cheetah3_DYNACORE_CONTROL_DEFINITION

#include <Cheetah3/Cheetah3_Definition.h>

#define Cheetah3ConfigPath THIS_COM"DynaController/Cheetah3_Controller/Cheetah3TestConfig/"

class Cheetah3_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];
        double jpos[cheetah3::num_act_joint];
        double jvel[cheetah3::num_act_joint];
        double jtorque[cheetah3::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class Cheetah3_Command{
    public:
        double jtorque_cmd[cheetah3::num_act_joint];
        double jpos_cmd[cheetah3::num_act_joint];
        double jvel_cmd[cheetah3::num_act_joint];
};


#endif
