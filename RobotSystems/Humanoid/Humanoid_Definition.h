#ifndef Humanoid_DEFINITION
#define Humanoid_DEFINITION

namespace humanoid_link{
    constexpr int torso = 0;
    constexpr int rightFoot = 7;
    constexpr int leftFoot = 8;
}

namespace humanoid_joint{
    constexpr int  virtual_X = 0;
    constexpr int  virtual_Y = 1;
    constexpr int  virtual_Z = 2;
    constexpr int  virtual_Rx = 3;
    constexpr int  virtual_Ry = 4;
    constexpr int  virtual_Rz = 5;

    constexpr int l_leg_hpz = 6;
    constexpr int l_leg_hpx = 7;
    constexpr int l_leg_hpy = 8;
    constexpr int l_leg_kny = 9;
    constexpr int l_leg_aky = 10;

    constexpr int r_leg_hpz = 11;
    constexpr int r_leg_hpx = 12;
    constexpr int r_leg_hpy = 13;
    constexpr int r_leg_kny = 14;
    constexpr int r_leg_aky = 15;

    constexpr int l_arm_shx = 16;
    constexpr int l_arm_shy = 17;
    constexpr int l_arm_ely = 18;

    constexpr int r_arm_shx = 19;
    constexpr int r_arm_shy = 20;
    constexpr int r_arm_ely = 21;

    constexpr int  virtual_Rw = 22;
}

namespace humanoid{
    // Simple version
    constexpr int num_q = 23;
    constexpr int num_qdot = 22;
    constexpr int num_act_joint = 16;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;
    constexpr int num_leg_joint = 5;
    constexpr int upper_body_start_jidx = humanoid_joint::l_arm_shy;
    constexpr int num_upper_joint = 12;
};
#endif
