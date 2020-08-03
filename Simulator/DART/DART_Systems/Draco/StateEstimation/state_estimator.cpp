#include "state_estimator.hpp"
#include "state_provider.hpp"

StateEstimator::StateEstimator(dart::dynamics::SkeletonPtr _skel) {
    mSP = StateProvider::getStateProvider();
    mSkeleton = _skel;
}

StateEstimator::~StateEstimator() {}

void StateEstimator::initialization() {
    mSP->Q.resize(mSkeleton->getNumDofs());
    mSP->Qdot.resize(mSkeleton->getNumDofs());
    for (int i = 0; i < mSkeleton->getNumDofs(); ++i) {
        mSP->Q[i] = mSkeleton->getDof(i)->getPosition();
        mSP->Qdot[i] = mSkeleton->getDof(i)->getVelocity();
    }
}

void StateEstimator::update() {
    mSP->Q.resize(mSkeleton->getNumDofs());
    mSP->Qdot.resize(mSkeleton->getNumDofs());
    for (int i = 0; i < mSkeleton->getNumDofs(); ++i) {
        mSP->Q[i] = mSkeleton->getDof(i)->getPosition();
        mSP->Qdot[i] = mSkeleton->getDof(i)->getVelocity();
    }
}
