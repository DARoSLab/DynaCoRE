#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "ValkyrieWorldNode.hpp"
#include <SIM_Configuration.h>
#include <Configuration.h>

class OneStepProgress : public osgGA::GUIEventHandler
{
public:
    OneStepProgress(ValkyrieWorldNode* worldnode): worldnode_(worldnode){  }

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& /*aa*/)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
            if (ea.getKey() == 'f') {
                printf("one step progress\n");
                //worldnode_->simulate(true);
                //usleep(1000);
                //worldnode_->simulate(false);
                worldnode_->customPreStep();
                worldnode_->getWorld()->step();
                worldnode_->customPostStep();

                printf("one step end\n");
                //worldnode_->getWorld()->step();
                return true;
            }
        }
        return false;
    }
    ValkyrieWorldNode* worldnode_;
};

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {

    //for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        //dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << bn->getName() << std::endl;
    //}

    //for (int i = 0; i < robot->getNumJoints(); ++i) {
        //dart::dynamics::Joint* joint = robot->getJoint(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << joint->getNumDofs() << std::endl;
    //}

    //for (int i = 0; i < robot->getNumDofs(); ++i) {
        //dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << dof->getName() << std::endl;
    //}

    std::cout << robot->getNumDofs() << std::endl;
    std::cout << robot->getNumJoints() << std::endl;
    std::cout << robot->getMassMatrix().rows() << std::endl;
    std::cout << robot->getMassMatrix().cols() << std::endl;
    exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    Eigen::VectorXd q(robot->getNumDofs());
    q.setZero();

    q[5] =1.135;

    Eigen::VectorXd idx_joint(14);
    // lower body
    idx_joint[0] = robot->getDof("leftHipPitch")->getIndexInSkeleton();
    idx_joint[1] = robot->getDof("rightHipPitch")->getIndexInSkeleton();
    idx_joint[2] = robot->getDof("leftKneePitch")->getIndexInSkeleton();
    idx_joint[3] = robot->getDof("rightKneePitch")->getIndexInSkeleton();
    idx_joint[4] = robot->getDof("leftAnklePitch")->getIndexInSkeleton();
    idx_joint[5] = robot->getDof("rightAnklePitch")->getIndexInSkeleton();

    // upper body
    idx_joint[6] = robot->getDof("rightShoulderPitch")->getIndexInSkeleton();
    idx_joint[7] = robot->getDof("rightShoulderRoll")->getIndexInSkeleton();
    idx_joint[8] = robot->getDof("rightElbowPitch")->getIndexInSkeleton();
    idx_joint[9] = robot->getDof("rightForearmYaw")->getIndexInSkeleton();
    idx_joint[10] = robot->getDof("leftShoulderPitch")->getIndexInSkeleton();
    idx_joint[11] = robot->getDof("leftShoulderRoll")->getIndexInSkeleton();
    idx_joint[12] = robot->getDof("leftElbowPitch")->getIndexInSkeleton();
    idx_joint[13] = robot->getDof("leftForearmYaw")->getIndexInSkeleton();

    Eigen::VectorXd config_joint(14);
    config_joint << -0.3, -0.3, 0.6, 0.6, -0.3, -0.3, 0.2, 1.1, 0.4, 1.5, -0.3, -1.1, -0.4, 1.5;

    for(int i(0); i<14; i++)
        q[idx_joint[i]] = config_joint[i];

    robot->setPositions(q);
}

int main() {
    // ================================
    // Generate world and add skeletons
    // ================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
            SIM_MODEL_PATH"/Valkyrie/ground_terrain.urdf");
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            SIM_MODEL_PATH"/Valkyrie/valkyrie_simple.urdf");

    world->addSkeleton(ground);
    world->addSkeleton(robot);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(1.0/2000);

    // =====================
    // Initial configuration
    // =====================
    _setInitialConfiguration(robot);

    // ================
    // Print Model Info
    // ================
    //_printRobotModel(robot);

    osg::ref_ptr<osgShadow::MinimalShadowMap> msm =
        new osgShadow::LightSpacePerspectiveShadowMapDB;

    float minLightMargin = 10.f;
    float maxFarPlane = 0;
    unsigned int texSize = 1024;
    unsigned int baseTexUnit = 0;
    unsigned int shadowTexUnit = 1;

    msm->setMinLightMargin( minLightMargin );
    msm->setMaxFarPlane( maxFarPlane );
    msm->setTextureSize( ::osg::Vec2s( texSize, texSize ) );
    msm->setShadowTextureCoordIndex( shadowTexUnit );
    msm->setShadowTextureUnit( shadowTexUnit );
    msm->setBaseTextureCoordIndex( baseTexUnit );
    msm->setBaseTextureUnit( baseTexUnit );

    // ================
    // Wrap a worldnode
    // ================

    osg::ref_ptr<ValkyrieWorldNode> node
        = new ValkyrieWorldNode(world, msm);
    node->setNumStepsPerCycle(30);

    // =====================
    // Create and Set Viewer
    // =====================

    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.switchHeadlights(false);
    ::osg::Vec3 p1(1.0, 0.2 , 1.5);
    msm->setLight(viewer.getLightSource(0)->getLight());
    viewer.getLightSource(0)->getLight()->setPosition(::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f)); 
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    viewer.addEventHandler(new OneStepProgress( node) );
    //viewer.record(THIS_COM"/experiment_data_video");

    std::cout << "=====================================" << std::endl;
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 1280, 960);
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3( 5.14,  3.28, 1.8)*1.1,
            ::osg::Vec3( 0.0,  0.0, 1.0),
            ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
