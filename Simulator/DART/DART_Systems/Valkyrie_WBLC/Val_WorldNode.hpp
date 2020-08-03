#ifndef VALKYRIE_WBLC_WORLD_NODE
#define VALKYRIE_WBLC_WORLD_NODE

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Valkyrie_Controller/Valkyrie_interface.hpp>

class Val_WorldNode : public dart::gui::osg::WorldNode
{
private:
    dart::dynamics::SkeletonPtr mSkel;

    Eigen::VectorXd mQ;
    Eigen::VectorXd mQdot;
    Eigen::VectorXd mActuatedTorque;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    Valkyrie_SensorData * sensor_data_;
    std::vector<double> cmd_;

    interface* interface_;

public:
     Val_WorldNode(const dart::simulation::WorldPtr & world);
     Val_WorldNode(const dart::simulation::WorldPtr & world, 
            osgShadow::MinimalShadowMap *);
    Val_WorldNode(const dart::simulation::WorldPtr & world, 
            osgShadow::ShadowTechnique *);
    virtual ~Val_WorldNode();

    void customPreStep() override;

};

#endif /* VALKYRIE_WORLD_NODE */
