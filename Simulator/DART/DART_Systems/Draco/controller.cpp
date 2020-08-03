#include "controller.hpp"

Controller::Controller(dart::dynamics::SkeletonPtr _skeleton) {
    mTime = 0.0;
    mStepTime = 0.001;
    mSkeleton = _skeleton;
    mInitTime = 0.01;
    mTorqueCommand.resize(mSkeleton->getNumDofs());
    mStateEstimator = new StateEstimator(mSkeleton);
}

Controller::~Controller() {}

void Controller::update() {

    if (mTime < mInitTime) {
        mStateEstimator->initialization();
        _initialize();
    } else {
        mStateEstimator->update();
        _computeCommand();
    }
    mSkeleton->setForces(mTorqueCommand);
    mTime += mStepTime;
}
