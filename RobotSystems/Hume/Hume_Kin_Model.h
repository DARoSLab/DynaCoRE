#ifndef HUME_KIN_MODEL
#define HUME_KIN_MODEL

#include "utils/wrap_eigen.hpp"
#include <rbdl/rbdl.h>
#include "Configuration.h"

using namespace sejong;

class Hume_Kin_Model{
public:
    Hume_Kin_Model(RigidBodyDynamics::Model* model);
    ~Hume_Kin_Model(void);

    void getCentroidJacobian(sejong::Matrix & J);
    void getPosition(const Vector & q, int link_id, Vect3 & pos);
    // Return Quaternion 
    void getOrientation(const Vector & q, int link_id, sejong::Quaternion & ori);
    void getVelocity(const Vector & q, const Vector &qdot,
                     int link_id, Vect3 & vel);
    void getAngVel(const Vector & q, const Vector & qdot,
                   int link_id, Vect3 & ang_vel);

    void getJacobian(const Vector & q, int link_id, Matrix & J);

    void getCoMJacobian  (const Vector & q, Matrix & J) const;
    void getCoMPos  (const Vector &q, Vect3 & com_pos, bool update) const;
    void getCoMVel (const Vector & q, const Vector & qdot, Vect3 & com_vel) const;

protected:
    RigidBodyDynamics::Model* model_;
    unsigned int _find_body_idx(int id) const ;
};

#endif
