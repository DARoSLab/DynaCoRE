#ifndef MERCURY_INVERSE_KINEMATICS
#define MERCURY_INVERSE_KINEMATICS

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Mercury_InvKinematics{
    public:
        Mercury_InvKinematics();
        ~Mercury_InvKinematics();
        
        void getFootPosAtVerticalPosture(
                int link_id, const dynacore::Vect3 & leg_config,
                const dynacore::Vector & guess_Q, dynacore::Vect3 & foot_pos);

   
        void getLegConfigAtVerticalPosture(
                int link_id, const dynacore::Vect3 & target_pos,
                const dynacore::Vector & guess_Q, dynacore::Vector & config_sol);

        void getDoubleSupportLegConfig(const dynacore::Vector & current_Q,
                                       const dynacore::Quaternion & des_quat,
                                       const double & des_height, dynacore::Vector & config_sol);

        void getSingleSupportFullConfig(const dynacore::Vector & current_Q,
                                       const dynacore::Quaternion & des_quat,
                                       const double & des_height, 
                                       int swing_foot_,
                                       const dynacore::Vect3 & foot_pos,
                                       const dynacore::Vect3 & foot_vel,
                                       const dynacore::Vect3 & foot_acc,
                                       dynacore::Vector & config_sol,
                                       dynacore::Vector & qdot_cmd, 
                                       dynacore::Vector & qddot_cmd);


        void getSingleSupportFullConfigSeperation(const dynacore::Vector & current_Q,
                                       const dynacore::Quaternion & des_quat,
                                       const double & des_height, 
                                       int swing_foot_,
                                       const dynacore::Vect3 & foot_pos,
                                       const dynacore::Vect3 & foot_vel,
                                       const dynacore::Vect3 & foot_acc,
                                       dynacore::Vector & config_sol,
                                       dynacore::Vector & qdot_cmd, 
                                       dynacore::Vector & qddot_cmd);

    protected:
        int max_iter_;

        RigidBodyDynamics::Model* model_;
};

#endif
