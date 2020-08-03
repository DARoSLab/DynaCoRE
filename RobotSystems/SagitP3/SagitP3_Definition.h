#ifndef SagitP3_DEFINITION
#define SagitP3_DEFINITION

namespace sagitP3_link{
    constexpr int hip_ground = 0;

    constexpr int l_hip_abd = 1;
    constexpr int l_hip_fe = 2;
    constexpr int l_thigh = 3;
    constexpr int l_shin = 4;
    constexpr int l_ankle = 5;
    constexpr int l_foot = 6;

    constexpr int r_hip_abd = 7;
    constexpr int r_hip_fe = 8;
    constexpr int r_thigh = 9;
    constexpr int r_shin = 10;
    constexpr int r_ankle = 11;
    constexpr int r_foot = 12;
}

namespace sagitP3_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    constexpr int Abduction_Left = 6;
    constexpr int Flexion_Left = 7;           
    constexpr int Hip_Yaw_Left_Passive = 8;    
    constexpr int Knee_Left = 9;
    constexpr int Ankle_Left = 10;
    constexpr int Ankle_Roll_Left_Passive = 11;

    constexpr int Abduction_Right = 12;
    constexpr int Flexion_Right = 13;
    constexpr int Hip_Yaw_Right_Passive = 14;
    constexpr int Knee_Right = 15;
    constexpr int Ankle_Right = 16;
    constexpr int Ankle_Roll_Right_Passive = 17;

    constexpr int  virtual_Rw = 18;

    //constexpr int Sagit_P3_Shin_Left_FT_Sensor = 19;     
    //constexpr int Sagit_P3_Thigh_Left_FT_Sensor = 20;   
    //constexpr int Sagit_P3_Shin_Right_FT_Sensor = 21;   
    //constexpr int Sagit_P3_Thigh_Right_FT_Sensor = 22; 
    //constexpr int Sagit_P3_Hip_FT_Sensor = 23;
    //constexpr int Sagit_P3_IMU_joint = 24;
}

namespace sagitP3{
    // Simple version
    constexpr int num_q = 19;
    constexpr int num_qdot = 18;
    constexpr int num_act_joint = 12;
    //constexpr int num_act_joint = 8;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 1./1500.;
    constexpr int num_leg_joint = 6; // How many joint in each leg
};
#endif
