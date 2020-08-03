#ifndef HUME_MODEL
#define HUME_MODEL

#include "rbdl/rbdl.h"
#include "WBOSC/WBOSC_Model.h"


class Hume_Dyn_Model;
class Hume_Kin_Model;

using namespace sejong;

class HumeModel: public WBOSC_Model{
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static HumeModel* GetHumeModel();
    ~HumeModel(void);

    virtual bool InitializeDynamicsModel(const Vector & q, const Vector & qdot);
    virtual bool getInverseMassInertia(sejong::Matrix & Ainv) ;
    virtual bool getGravity(Vector & grav) ;
    virtual bool getCoriolis(Vector & coriolis) ;
    
    virtual void getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const ;
    
    virtual void getPosition(const Vector & q,
                             int link_id, Vect3 & pos) ;
    virtual void getOrientation(const Vector & q,
                                int link_id, sejong::Quaternion & ori) ;
    
    virtual void getVelocity(const Vector & q, const Vector &qdot,
                             int link_id, Vect3 & vel) ;
    virtual void getAngVel(const Vector & q, const Vector & qdot,
                           int link_id, Vect3 & ang_vel);

    void getCentroidJacobian(sejong::Matrix & J);
    void getCoMJacobian(const Vector & q, sejong::Matrix & J);
    void getCoMPosition(const Vector & q, Vect3 & com_pos, bool update= false);
    void getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel);
    void UpdateKinematics(const Vector & q, const Vector &qdot);
    void UpdateModel(const sejong::Vector & q, const sejong::Vector & qdot);

protected:
    Hume_Dyn_Model* dyn_model_;
    Hume_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;
private:
    HumeModel();
};

#endif
