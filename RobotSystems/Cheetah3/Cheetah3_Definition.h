#ifndef Cheetah3_DEFINITION
#define Cheetah3_DEFINITION

namespace cheetah3{
    constexpr int num_q = 19;
    constexpr int num_qdot = 18;
    constexpr int num_act_joint = 12;
    constexpr int num_virtual = 6;
    //constexpr double servo_rate = 1.0/1500.0;
    constexpr double servo_rate = 1.0/1000.0;
};

namespace cheetah3_link{
    constexpr int body = 0;
    constexpr int imu = 1;
    constexpr int fr_Foot = 2;
    constexpr int fl_Foot = 3;
    constexpr int hr_Foot = 4;
    constexpr int hl_Foot = 5;
};

namespace cheetah3_joint{
    constexpr int virtual_X = 0;
    constexpr int virtual_Y = 1;
    constexpr int virtual_Z = 2;
    constexpr int virtual_Rx = 3;
    constexpr int virtual_Ry = 4;
    constexpr int virtual_Rz = 5;

    constexpr int fr_Abduction = 6;
    constexpr int fr_Hip = 7;
    constexpr int fr_Knee = 8;
    
    constexpr int fl_Abduction = 9;
    constexpr int fl_Hip = 10;
    constexpr int fl_Knee = 11;
   
    constexpr int hr_Abduction = 12;
    constexpr int hr_Hip = 13;
    constexpr int hr_Knee = 14;

    constexpr int hl_Abduction = 15;
    constexpr int hl_Hip = 16;
    constexpr int hl_Knee = 17;
    
    constexpr int virtual_Rw = 18;
};

#endif

