#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "MiniCheetahWorldNode.hpp"
#include <SIM_Configuration.h>
#include <Configuration.h>

class OneStepProgress : public osgGA::GUIEventHandler
{
public:
    OneStepProgress(MiniCheetahWorldNode* worldnode): worldnode_(worldnode){  }

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
    MiniCheetahWorldNode* worldnode_;
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

    q[5] = 1.0;

    int num_joint(12);
    Eigen::VectorXd idx_joint(num_joint);
    // Front Right Leg
    idx_joint[0] = robot->getDof("torso_to_abduct_fr_j")->getIndexInSkeleton();
    idx_joint[1] = robot->getDof("abduct_fr_to_thigh_fr_j")->getIndexInSkeleton();
    idx_joint[2] = robot->getDof("thigh_fr_to_knee_fr_j")->getIndexInSkeleton();

    // Front Left Leg
    idx_joint[3] = robot->getDof("torso_to_abduct_fl_j")->getIndexInSkeleton();
    idx_joint[4] = robot->getDof("abduct_fl_to_thigh_fl_j")->getIndexInSkeleton();
    idx_joint[5] = robot->getDof("thigh_fl_to_knee_fl_j")->getIndexInSkeleton();

    // Hind Right Leg
    idx_joint[6] = robot->getDof("torso_to_abduct_hr_j")->getIndexInSkeleton();
    idx_joint[7] = robot->getDof("abduct_hr_to_thigh_hr_j")->getIndexInSkeleton();
    idx_joint[8] = robot->getDof("thigh_hr_to_knee_hr_j")->getIndexInSkeleton();

    // Hind Left Leg
    idx_joint[9] = robot->getDof("torso_to_abduct_hl_j")->getIndexInSkeleton();
    idx_joint[10] = robot->getDof("abduct_hl_to_thigh_hl_j")->getIndexInSkeleton();
    idx_joint[11] = robot->getDof("thigh_hl_to_knee_hl_j")->getIndexInSkeleton();


    Eigen::VectorXd config_joint(num_joint);
    config_joint << 
        0.0, -0.3, 0.6, 0.0, -0.3, 0.6,
        0.0, -0.3, 0.6, 0.0, -0.3, 0.6;
        //0.0, -0.3, 0.3, 0.0, -0.3, 0.3,
        //0.0, -0.3, 0.3, 0.0, -0.3, 0.3;


    for(int i(0); i<num_joint; i++)
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
            SIM_MODEL_PATH"/MiniCheetah/ground_terrain.urdf");
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            SIM_MODEL_PATH"/MiniCheetah/mini_cheetah.urdf");

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

    osg::ref_ptr<MiniCheetahWorldNode> node
        = new MiniCheetahWorldNode(world, msm);
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
