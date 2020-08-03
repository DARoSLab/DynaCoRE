#ifndef MiniCheetah_WORLD_NODE
#define MiniCheetah_WORLD_NODE

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>
#include <osgShadow/LightSpacePerspectiveShadowMap>
//#include <MiniCheetah_Controller/MiniCheetah_interface.hpp>

class MiniCheetahWorldNode : public dart::gui::osg::WorldNode
{
private:
    dart::dynamics::SkeletonPtr mSkel;

    Eigen::VectorXd mQ;
    Eigen::VectorXd mQdot;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    //MiniCheetah_SensorData* sensor_data_;
    //MiniCheetah_Command* cmd_;
    //interface* interface_;

public:
    MiniCheetahWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~MiniCheetahWorldNode();

    void customPreStep() override;

};

#endif /* VALKYRIE_WORLD_NODE */
