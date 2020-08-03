#ifndef OPENCHAIN_2DOF_MODEL
#define OPENCHAIN_2DOF_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class OC2_Dyn_Model;
class OC2_Kin_Model;

using namespace sejong;

class OC2Model{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static OC2Model* GetOC2Model();
    virtual ~OC2Model(void);

    bool getMassInertia(sejong::Matrix & A);

    bool getInverseMassInertia(sejong::Matrix & Ainv) ;
    bool getGravity(Vector & grav) ;
    bool getCoriolis(Vector & coriolis) ;

    void getCentroidJacobian(sejong::Matrix & Jcent);
    void getCentroidInertia(sejong::Matrix & Icent);
    void getPosition(const Vector & q,
                     int link_id, Vect3 & pos) ;
    void getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const ;
    void getCoMPosition(const Vector & q, Vect3 & com_pos, bool update= false);
    void getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel);

    void getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & J) const ;
    void getOrientation(const Vector & q,
                        int link_id, sejong::Quaternion & ori) ;
    void getVelocity(const Vector & q, const Vector &qdot,
                     int link_id, Vect3 & vel) ;
    void getAngVel(const Vector & q, const Vector & qdot,
                   int link_id, Vect3 & ang_vel);

    void getCentroidVelocity(sejong::Vector & centroid_vel);
    void getCoMJacobian(const Vector & q, sejong::Matrix & J);
    void UpdateKinematics(const Vector & q, const Vector &qdot);
    void UpdateModel(const sejong::Vector & q, const sejong::Vector & qdot);

protected:
    OC2_Dyn_Model* dyn_model_;
    OC2_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;
private:
    OC2Model();
};

#endif
