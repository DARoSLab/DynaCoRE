#ifndef OPENCHAIN_3D_KIN_MODEL
#define OPENCHAIN_3D_KIN_MODEL

#include <Utils/wrap_eigen.hpp>
#include <rbdl/rbdl.h>
#include <Configuration.h>

using namespace sejong;

class OC3_Kin_Model{
public:
  OC3_Kin_Model(RigidBodyDynamics::Model* model);
  ~OC3_Kin_Model(void);

  void getPosition(const Vector & q, int link_id, Vect3 & pos);
  // Return Quaternion
  void getOrientation(const Vector & q, int link_id, sejong::Quaternion & ori);
  void getVelocity(const Vector & q, const Vector &qdot,
                   int link_id, Vect3 & vel);
  void getAngVel(const Vector & q, const Vector & qdot,
                 int link_id, Vect3 & ang_vel);

  void getJacobian(const Vector & q, int link_id, Matrix & J);
  void getJacobianDot6D_Analytic(const Vector & q, const Vector & qdot, int link_id, Matrix & J);

  void getCoMJacobian  (const Vector & q, Matrix & J) const;
  void getCoMPos  (const Vector &q, Vect3 & com_pos, bool update) const;
  void getCoMVel (const Vector & q, const Vector & qdot, Vect3 & com_vel) const;

  void getCentroidInertia(sejong::Matrix & Icent){ Icent = Ig_; }
  void getCentroidJacobian(sejong::Matrix & Jcent){ Jcent = Jg_; }
  void getCentroidVelocity(sejong::Vector & centroid_vel);

  void UpdateKinematics(const sejong::Vector & q, const sejong::Vector & qdot);

  sejong::Vector com_pos_;
  sejong::Vector centroid_vel_;
protected:
  void _UpdateCentroidFrame(const sejong::Vector & q, const sejong::Vector & qdot);
  sejong::Matrix Ig_;
  sejong::Matrix Jg_;

  RigidBodyDynamics::Model* model_;
  unsigned int _find_body_idx(int id) const ;
};

#endif
