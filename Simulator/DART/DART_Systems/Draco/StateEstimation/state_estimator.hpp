#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "state_provider.hpp"
#include <dart/dart.hpp>

class StateEstimator
{
protected:
    StateProvider* mSP;
    dart::dynamics::SkeletonPtr mSkeleton;

public:
    StateEstimator(dart::dynamics::SkeletonPtr _skel);
    virtual ~StateEstimator();

    void initialization();
    void update();
};

#endif /* STATE_ESTIMATOR_H */
