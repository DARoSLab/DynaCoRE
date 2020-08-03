#include "Val_WorldNode.hpp"
#include <Utils/utilities.hpp>

Val_WorldNode::Val_WorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {

    cmd_.resize(valkyrie::num_act_joint, 0.);
    sensor_data_ = new Valkyrie_SensorData();
    mSkel = world_->getSkeleton("valkyrie");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mActuatedTorque = Eigen::VectorXd::Zero(mDof - 6);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    interface_ = new Valkyrie_interface();
}

Val_WorldNode::Val_WorldNode(const dart::simulation::WorldPtr & world_, 
        osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    cmd_.resize(valkyrie::num_act_joint, 0.);
    sensor_data_ = new Valkyrie_SensorData();
    mSkel = world_->getSkeleton("valkyrie");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mActuatedTorque = Eigen::VectorXd::Zero(mDof - 6);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    interface_ = new Valkyrie_interface();
}
Val_WorldNode::Val_WorldNode(const dart::simulation::WorldPtr & world_, 
        osgShadow::ShadowTechnique * sm) :
    dart::gui::osg::WorldNode(world_, sm) {

    cmd_.resize(valkyrie::num_act_joint, 0.);
    sensor_data_ = new Valkyrie_SensorData();
    mSkel = world_->getSkeleton("valkyrie");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mActuatedTorque = Eigen::VectorXd::Zero(mDof - 6);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    interface_ = new Valkyrie_interface();
}



Val_WorldNode::~Val_WorldNode() {}

void Val_WorldNode::customPreStep() {
    mQ = mSkel->getPositions();
    mQdot = mSkel->getVelocities();
    mTorqueCommand.setZero();

    for(int i(6); i<valkyrie::num_qdot; ++i){
        sensor_data_->q[i] = mQ[i];
        sensor_data_->qdot[i] = mQdot[i];
    }
    for(int i(0); i<3; ++i){
        // X, Y, Z
        sensor_data_->q[i] = mQ[i+3];
        sensor_data_->qdot[i] = mQdot[i+3];

        // Rx, Ry, Rz
        sensor_data_->qdot[i+3] = mQdot[i];
    }
    dynacore::Vect3 so3;
    for(int i(0); i<3; ++i) so3[i] = mQ[i];

    dynacore::Quaternion quat;
    dynacore::convert(so3, quat);
    sensor_data_->q[3] = quat.x();
    sensor_data_->q[4] = quat.y();
    sensor_data_->q[5] = quat.z();
    sensor_data_->q[valkyrie::num_qdot] = quat.w();


    //double theta(1.3);

    //sensor_data_->q[3] = sin(theta/2.);
    //sensor_data_->q[4] = 0.;
    //sensor_data_->q[5] = 0.;
    //sensor_data_->q[valkyrie::num_qdot] = cos(theta/2.);
//size check
//double quat_norm = quat.x() * quat.x() 
    //+ quat.y() * quat.y() 
    //+ quat.z()* quat.z()
    //+ quat.w() * quat.w();
//printf("norm of quat:%f\n", quat_norm);

    static int count(0);
     ++count;
     if(count % 2 == 0)  interface_->GetCommand(sensor_data_, cmd_);
     //interface_->GetCommand(sensor_data_, cmd_);

    //dynacore::pretty_print(mQ, std::cout, "dart Q");
    //dynacore::pretty_print(sensor_data_->q, "dynacore Q", valkyrie::num_q);
    //dynacore::pretty_print(mSkel->getGravityForces(), std::cout, "dart gravity");
    //dynacore::pretty_print(mSkel->getForces(), std::cout, "dart forces");

    for(int i(0); i<valkyrie::num_act_joint; ++i){
        mTorqueCommand[i + 6] = cmd_[i];
    }
 
    mSkel->setForces(mTorqueCommand);
}

