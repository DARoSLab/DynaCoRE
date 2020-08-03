#ifndef Atlas_KIN_MODEL
#define Atlas_KIN_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Atlas_Kin_Model{
    public:
        Atlas_Kin_Model(RigidBodyDynamics::Model* model);
        ~Atlas_Kin_Model(void);

        void getPos(int link_id, dynacore::Vect3 & pos);
        // Return Quaternion
        void getOri(int link_id, dynacore::Quaternion & ori);
        void getLinearVel(int link_id, dynacore::Vect3 & vel);
        void getAngularVel(int link_id, dynacore::Vect3 & ang_vel);

        void getJacobian(int link_id, dynacore::Matrix & J);
        void getJDotQdot(int link_id, dynacore::Vector & JDotQdot);

        void getCoMJacobian  (dynacore::Matrix & J);
        void getCoMPos  (dynacore::Vect3 & com_pos);
        void getCoMVel (dynacore::Vect3 & com_vel);

        void getCentroidInertia(dynacore::Matrix & Icent){ Icent = Ig_; }
        void getCentroidJacobian(dynacore::Matrix & Jcent){ Jcent = Jg_; }
        void getCentroidVelocity(dynacore::Vector & centroid_vel);

        void UpdateKinematics(const dynacore::Vector & q, const dynacore::Vector & qdot);

        dynacore::Vect3 com_pos_;
        dynacore::Vector centroid_vel_;
    protected:
        void _UpdateCentroidFrame(const dynacore::Vector & q, const dynacore::Vector & qdot);
        void _UpdateCentroidFrame_diff(const dynacore::Vector & q, const dynacore::Vector & qdot);
        dynacore::Matrix Ig_;
        dynacore::Matrix Jg_;

        RigidBodyDynamics::Model* model_;
        unsigned int _find_body_idx(int id) const ;
};

#endif
