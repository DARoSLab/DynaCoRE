#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <SIM_Configuration.h>
#include <dart/gui/osg/InteractiveFrame.hpp>
#include "WorldNode/draco_worldnode.hpp"

using namespace dart::gui::osg;

void setMeshColor(dart::dynamics::SkeletonPtr robot) {
    for(std::size_t i=0; i<robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for(auto shapeNode : shapeNodes) {
            std::shared_ptr<dart::dynamics::MeshShape> mesh =
                std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
                        shapeNode->getShape());
            if(mesh)
                mesh->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
        }
    }
}


void displayLinkFrames(
    const dart::simulation::WorldPtr& world, 
    const dart::dynamics::SkeletonPtr& robot)
{
  for(std::size_t i=0; i < robot->getNumBodyNodes(); ++i)
  {
      dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
      InteractiveFramePtr frame = std::make_shared<InteractiveFrame>
        (bn, bn->getName()+"/frame");

      for(const auto type : {InteractiveTool::ANGULAR, InteractiveTool::PLANAR})
          for(std::size_t i=0; i < 3; ++i)
              frame->getTool(type, i)->setEnabled(false);

      world->addSimpleFrame(frame);
      
  }
}


void displayJointFrames(
    const dart::simulation::WorldPtr& world, 
    const dart::dynamics::SkeletonPtr& robot)
{
  for(std::size_t i=0; i < robot->getNumBodyNodes(); ++i)
  {
      dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
      for(std::size_t j=0; j < bn->getNumChildJoints(); ++j)
      {
          const dart::dynamics::Joint* joint = bn->getChildJoint(j);
          const Eigen::Isometry3d offset = joint->getTransformFromParentBodyNode();

          InteractiveFramePtr frame = std::make_shared<InteractiveFrame>
              (bn,joint->getName()+"/frame", offset);

          for(const auto type : {InteractiveTool::ANGULAR, InteractiveTool::PLANAR})
              for(std::size_t i=0; i < 3; ++i)
                      frame->getTool(type, i)->setEnabled(false);

          world->addSimpleFrame(frame);
      }
  }
}




void _printModel(dart::dynamics::SkeletonPtr robot) {

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

    //exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    Eigen::VectorXd q(robot->getNumDofs());
    q.setZero();

    int num_joints=3;
    q[0] =0.000;

    Eigen::VectorXd idx_joint(num_joints);
     //lower body
    idx_joint[0] = robot->getDof("bodyPitch")->getIndexInSkeleton();
    idx_joint[1] = robot->getDof("kneePitch")->getIndexInSkeleton();
    idx_joint[2] = robot->getDof("ankle")->getIndexInSkeleton();

     //upper body
    Eigen::VectorXd config_joint(num_joints);
    //config_joint << 0.2, -0.1, 1.5;
    config_joint << 0.0, 0.0, 0.6 ;

    for(int i(0); i<num_joints; i++)
        q[idx_joint[i]] = config_joint[i];    

    robot->setPositions(q);
}

int main() {

    //// Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    auto ground= urdfLoader.parseSkeleton(SIM_MODEL_PATH"/draco/ground.urdf");
    auto draco =urdfLoader.parseSkeleton(SIM_MODEL_PATH"/draco/mk_draco.urdf");
    auto support =urdfLoader.parseSkeleton(SIM_MODEL_PATH"/draco/draco_support_3ds.urdf");

    //dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(robotmodel_path.c_str());
    world->addSkeleton(ground);
    world->addSkeleton(draco);
    world->addSkeleton(support);
    //draco->setPosition(0.0,0.0);

    //Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    Eigen::Vector3d gravity(0.0,0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(1.0/1000);

    // Initial configuration
    _setInitialConfiguration(draco);
     //Print Information
    _printModel(draco);

     //Set mesh color
    setMeshColor(draco);
    //setMeshColor(support);

    //// Wrap a worldnode
    osg::ref_ptr<DracoWorldNode> node
        = new DracoWorldNode(world);
    node->setNumStepsPerCycle(30);

    //displayLinkFrames(world,draco);
    displayJointFrames(world,draco);
    displayLinkFrames(world,support);
    displayJointFrames(world,support);

    //// Create viewer
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 0.57,  5.14, 2.04),
            ::osg::Vec3( 0.00,  0.00, 0.00),
            ::osg::Vec3(-0.55, -0.55, 0.95));
    //viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 2.57,  3.14, 1.64),
            //::osg::Vec3( 0.00,  0.00, 0.00),
            //::osg::Vec3(-0.24, -0.25, 0.94));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
