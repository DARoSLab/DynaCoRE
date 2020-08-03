#include "NAO_WorldNode.hpp"
#include <Utils/utilities.hpp>
NAO_WorldNode::NAO_WorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) 
{
    cmd_.resize(nao::num_act_joint, 0.);
    sensor_data_ = new NAO_Exam_SensorData();
    mSkel = world_->getSkeleton("NaoH25V50");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mActuatedTorque = Eigen::VectorXd::Zero(mDof-6);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);

    interface_ = new NAO_Exam_interface();
}


NAO_WorldNode::NAO_WorldNode(const dart::simulation::WorldPtr & world_,
        osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) 
{
    cmd_.resize(nao::num_act_joint, 0.);
    sensor_data_ = new NAO_Exam_SensorData();
    mSkel = world_->getSkeleton("NaoH25V50");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mActuatedTorque = Eigen::VectorXd::Zero(mDof-6);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);

    interface_ = new NAO_Exam_interface();
}

NAO_WorldNode::~NAO_WorldNode() {}

void NAO_WorldNode::customPreStep() {
    //printf("custom preset\n");

    mQ = mSkel->getPositions();
    mQdot = mSkel->getVelocities();

    mTorqueCommand.setZero();

    //for(int i(0); i<6; ++i){
        //sensor_data_->q[i] = 0.;
        //sensor_data_->qdot[i] = 0.;
    //}
    //for(int i(0); i<nao::num_act_joint; ++i){
        //sensor_data_->q[i + 6] = mQ[i];
        //sensor_data_->qdot[i + 6] = mQdot[i];
    //}
     for(int i(0); i<nao::num_qdot; ++i){
        sensor_data_->q[i] = mQ[i];
        sensor_data_->qdot[i] = mQdot[i];
    }
    for(int i(0); i<6; ++i){
        sensor_data_->q[i] = 0.;
        sensor_data_->qdot[i] = 0.;
    }
 
     static int count(0);
     ++count;
     //if(count % 5 == 0)  interface_->GetCommand(sensor_data_, cmd_);
     interface_->GetCommand(sensor_data_, cmd_);

    //dynacore::pretty_print(mSkel->getGravityForces(), std::cout, "dart gravity");
    //dynacore::pretty_print(mSkel->getForces(), std::cout, "dart forces");

    for(int i(0); i<nao::num_act_joint; ++i){
        mTorqueCommand[i + 6] = cmd_[i];
    }
    mSkel->setForces(mTorqueCommand);
    //dynacore::pretty_print(mSkel->getForces(), std::cout, "dart forces");
    //printf("\n");
}
