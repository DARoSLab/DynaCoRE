#ifndef VALKYRIE_WORLD_NODE
#define VALKYRIE_WORLD_NODE

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Valkyrie_Controller/Valkyrie_interface.hpp>

class ValkyrieWorldNode : public dart::gui::osg::WorldNode
{
private:
    dart::dynamics::SkeletonPtr mSkel;

    Eigen::VectorXd mQ;
    Eigen::VectorXd mQdot;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    Valkyrie_SensorData* sensor_data_;
    Valkyrie_Command* cmd_;
    interface* interface_;

public:
    ValkyrieWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~ValkyrieWorldNode();

    void customPreStep() override;

};

#endif /* VALKYRIE_WORLD_NODE */
