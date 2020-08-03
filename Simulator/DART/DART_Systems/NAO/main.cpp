#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "NAO_WorldNode.hpp"
#include <SIM_Configuration.h>

class ChangeFOVHandler : public osgGA::GUIEventHandler
{
public:
    ChangeFOVHandler(osg::Camera* camera)
        : _camera(camera)
    {
        double fovy, aspectRatio, zNear, zFar;
        _camera->getProjectionMatrix().getPerspective(fovy, aspectRatio, zNear, zFar);
    }

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& /*aa*/)
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
        {
            if (ea.getKey() == '-' || ea.getKey() == '=' || ea.getKey() == '0')
            {
                double fovy, aspectRatio, zNear, zFar;
                _camera->getProjectionMatrix().getPerspective(fovy, aspectRatio, zNear, zFar);

                if (ea.getKey() == '=')
                {
                    fovy -= 5.0;
                }

                if (ea.getKey() == '-')
                {
                    fovy += 5.0;
                }

                if (ea.getKey() == '0')
                {
                    fovy = 45.0;
                }
                _camera->getProjectionMatrix().makePerspective(fovy, aspectRatio, zNear, zFar);
                return true;
            }
        }
        return false;
    }
    osg::ref_ptr<osg::Camera> _camera;
};
class OneStepProgress : public osgGA::GUIEventHandler
{
public:
    OneStepProgress(NAO_WorldNode* worldnode): worldnode_(worldnode){  }

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
    NAO_WorldNode* worldnode_;
};


void _printRobotModel(dart::dynamics::SkeletonPtr robot) {

    for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        std::cout << i << "th" << std::endl;
        std::cout << bn->getName() << std::endl;
    }

    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        std::cout << i << "th" << std::endl;
        std::cout << joint->getNumDofs() << std::endl;
    }

    for (int i = 0; i < robot->getNumDofs(); ++i) {
        dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        std::cout << i << "th" << std::endl;
        std::cout << dof->getName() << std::endl;
    }

    //std::cout << robot->getNumDofs() << std::endl;
    //std::cout << robot->getNumJoints() << std::endl;
    //std::cout << robot->getMassMatrix() << std::endl;
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    Eigen::VectorXd q(robot->getNumDofs());
    Eigen::VectorXd qdot(robot->getNumDofs());
    q.setZero();
    qdot.setZero();

    q[5] = 0.365;

    //Eigen::VectorXd idx_joint(14);
    //idx_joint[0] = robot->getDof("leftHipPitch")->getIndexInSkeleton();
    //idx_joint[1] = robot->getDof("rightHipPitch")->getIndexInSkeleton();
    //idx_joint[2] = robot->getDof("leftKneePitch")->getIndexInSkeleton();
    //idx_joint[3] = robot->getDof("rightKneePitch")->getIndexInSkeleton();
    //idx_joint[4] = robot->getDof("leftAnklePitch")->getIndexInSkeleton();
    //idx_joint[5] = robot->getDof("rightAnklePitch")->getIndexInSkeleton();

    //idx_joint[6] = robot->getDof("rightShoulderPitch")->getIndexInSkeleton();
    //idx_joint[7] = robot->getDof("rightShoulderRoll")->getIndexInSkeleton();
    //idx_joint[8] = robot->getDof("rightElbowPitch")->getIndexInSkeleton();
    //idx_joint[9] = robot->getDof("rightForearmYaw")->getIndexInSkeleton();
    //idx_joint[10] = robot->getDof("leftShoulderPitch")->getIndexInSkeleton();
    //idx_joint[11] = robot->getDof("leftShoulderRoll")->getIndexInSkeleton();
    //idx_joint[12] = robot->getDof("leftElbowPitch")->getIndexInSkeleton();
    //idx_joint[13] = robot->getDof("leftForearmYaw")->getIndexInSkeleton();

    //Eigen::VectorXd config_joint(14);
    //config_joint << -0.3, -0.3, 0.6, 0.6, -0.3, -0.3, 0.2, 1.1, 0.4, 1.5, -0.3, -1.1, -0.4, 1.5;

    //for(int i(0); i<14; i++)
        //q[idx_joint[i]] = config_joint[i];    

    robot->setPositions(q);
    robot->setVelocities(qdot);
}

int main() {
    //// Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    //auto ground = urdfLoader.parseSkeleton(
            //SIM_MODEL_PATH"/NAO_Model/ground_terrain.urdf");
     auto ground = urdfLoader.parseSkeleton(
            SIM_MODEL_PATH"/NAO_Model/ground.urdf");
     //dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            //SIM_MODEL_PATH"/NAO_Model/urdf/nao_V50.urdf");

   dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            SIM_MODEL_PATH"/NAO_Model/urdf/nao_simple.urdf");

   std::cout<<"resti: "<< ground->getBodyNode(0)->getRestitutionCoeff()<<std::endl;
   std::cout<<"friction: "<< ground->getBodyNode(0)->getFrictionCoeff()<<std::endl;

    world->addSkeleton(ground);
    world->addSkeleton(robot);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    //std::cout<<"time step: "<<world->getTimeStep()<<std::endl;
    world->setTimeStep(1.0/5000);

    // Initial configuration
    _setInitialConfiguration(robot);

    osg::ref_ptr<osgShadow::MinimalShadowMap> msm = NULL;
    msm = new osgShadow::LightSpacePerspectiveShadowMapDB;

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

    // Print Model Info
    //_printRobotModel(robot);

    //// Wrap a worldnode
    osg::ref_ptr<NAO_WorldNode> node = new NAO_WorldNode(world, msm);
    //osg::ref_ptr<NAO_WorldNode> node = new NAO_WorldNode(world);
    node->setNumStepsPerCycle(50);

    //// Create viewer
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    msm->setLight(viewer.getLightSource(0)->getLight());
    viewer.simulate(false);
    viewer.addEventHandler(new ChangeFOVHandler(viewer.getCamera()));
    viewer.addEventHandler(new OneStepProgress( node) );

    std::cout << "=====================================" << std::endl;
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 1280, 960);
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3( 2.57,  3.14, 1.5)*1.0,
            ::osg::Vec3( 0.00,  0.00, 0.50),
            ::osg::Vec3(-0.24, -0.25, 0.94));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
