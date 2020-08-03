#ifndef NAO_WORLD_NODE
#define NAO_WORLD_NODE

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>
#include <NAO_Exam_Controller/NAO_Exam_interface.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

class NAO_WorldNode : public dart::gui::osg::WorldNode
{
private:
    dart::dynamics::SkeletonPtr mSkel;

    Eigen::VectorXd mQ;
    Eigen::VectorXd mQdot;
    Eigen::VectorXd mActuatedTorque;
    Eigen::VectorXd mTorqueCommand;

    NAO_Exam_SensorData * sensor_data_;
    std::vector<double> cmd_;

    int mDof;
    interface* interface_;

public:
     NAO_WorldNode(const dart::simulation::WorldPtr & world);
    NAO_WorldNode(const dart::simulation::WorldPtr & world, 
            osgShadow::MinimalShadowMap * );
    virtual ~NAO_WorldNode();

    void customPreStep() override;
};

#endif 
