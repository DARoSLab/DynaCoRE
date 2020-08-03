#ifndef WALKER_2D_MODEL
#define WALKER_2D_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Walker2D_Dyn_Model;
class Walker2D_Kin_Model;

using namespace sejong;

class Walker2D_Model{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static Walker2D_Model* GetWalker2D_Model();
    virtual ~Walker2D_Model(void);

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
    Walker2D_Dyn_Model* dyn_model_;
    Walker2D_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;
private:
    Walker2D_Model();
};

#endif
