#ifndef Humanoid_DYNACORE_CONTROL_DEFINITION
#define Humanoid_DYNACORE_CONTROL_DEFINITION

#include <Humanoid/Humanoid_Definition.h>

#define HumanoidConfigPath THIS_COM"DynaController/Humanoid_Controller/HumanoidTestConfig/"

class Humanoid_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];
        double jpos[humanoid::num_act_joint];
        double jvel[humanoid::num_act_joint];
        double jtorque[humanoid::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class Humanoid_Command{
    public:
        double jtorque_cmd[humanoid::num_act_joint];
        double jpos_cmd[humanoid::num_act_joint];
        double jvel_cmd[humanoid::num_act_joint];
};


#endif
