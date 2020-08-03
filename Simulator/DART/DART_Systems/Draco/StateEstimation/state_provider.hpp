#ifndef STATE_PROVIDER_H
#define STATE_PROVIDER_H

#include <Eigen/Dense>

class StateProvider
{
private:
    StateProvider();

public:
    static StateProvider* getStateProvider();
    ~StateProvider();

    Eigen::VectorXd Q;
    Eigen::VectorXd Qdot;
};

#endif /* STATE_PROVIDER_H */
