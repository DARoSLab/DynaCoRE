#include "ValkyrieWorldNode.hpp"

ValkyrieWorldNode::ValkyrieWorldNode(const dart::simulation::WorldPtr & world_,
                                     osgShadow::MinimalShadowMap * msm):
    dart::gui::osg::WorldNode(world_, msm) {

    sensor_data_ = new Valkyrie_SensorData();
    cmd_ = new Valkyrie_Command();
    interface_ = new Valkyrie_interface();

    mSkel = world_->getSkeleton("valkyrie");
    mDof = mSkel->getNumDofs();
    mQ = Eigen::VectorXd::Zero(mDof);
    mQdot = Eigen::VectorXd::Zero(mDof);
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);

}

ValkyrieWorldNode::~ValkyrieWorldNode() {}

void ValkyrieWorldNode::customPreStep() {
    mQ = mSkel->getPositions();
    mQdot = mSkel->getVelocities();
    mTorqueCommand.setZero();

    for(int i(0); i<valkyrie::num_qdot; ++i){
        sensor_data_->jpos[i] = mQ[i+6];
        sensor_data_->jvel[i] = mQdot[i+6];
    }
    for(int i(0); i<3; ++i){
        // X, Y, Z
        //sensor_data_->q[i] = mQ[i+3];
        //sensor_data_->qdot[i] = mQdot[i+3];

        // Rx, Ry, Rz
        sensor_data_->imu_ang_vel[i] = mQdot[i];
    }
    //dynacore::Vect3 so3;
    //for(int i(0); i<3; ++i) so3[i] = mQ[i];

    //dynacore::Quaternion quat;
    //dynacore::convert(so3, quat);
    //sensor_data_->q[3] = quat.x();
    //sensor_data_->q[4] = quat.y();
    //sensor_data_->q[5] = quat.z();
    //sensor_data_->q[valkyrie::num_qdot] = quat.w();

    interface_->GetCommand(sensor_data_, cmd_);
    for(int i(0); i<valkyrie::num_act_joint; ++i)
        mTorqueCommand[i + 6] = cmd_->jtorque_cmd[i];

    mSkel->setForces(mTorqueCommand);
}

