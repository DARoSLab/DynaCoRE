#ifndef TELLO_DEFINITION
#define TELLO_DEFINITION

namespace tello_link{
    constexpr int torso = 0;
    constexpr int leftFoot = 5;
    constexpr int rightFoot = 10;
}

namespace tello_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    //constexpr int r_hip_yaw = 6;
    //constexpr int r_hip_roll = 7;
    //constexpr int r_hip_pitch = 8;
    //constexpr int r_knee = 9;
    //constexpr int r_ankle = 10;

    //constexpr int l_hip_yaw = 11;
    //constexpr int l_hip_roll = 12;
    //constexpr int l_hip_pitch = 13;
    //constexpr int l_knee = 14;
    //constexpr int l_ankle = 15;

    constexpr int l_hip_yaw = 6;
    constexpr int l_hip_roll = 7;
    constexpr int l_hip_pitch = 8;
    constexpr int l_knee = 9;
    constexpr int l_ankle = 10;

    constexpr int r_hip_yaw = 11;
    constexpr int r_hip_roll = 12;
    constexpr int r_hip_pitch = 13;
    constexpr int r_knee = 14;
    constexpr int r_ankle = 15;

    constexpr int  virtual_Rw = 16;
}

namespace tello{
    // Simple version
    constexpr int num_q = 17;
    constexpr int num_qdot = 16;
    constexpr int num_act_joint = 10;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;
    constexpr int num_leg_joint = 5;
};
#endif
