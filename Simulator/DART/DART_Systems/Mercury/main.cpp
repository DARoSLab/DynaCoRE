#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <SIM_Configuration.h>

#include "MercuryEventHandler.hpp"
#include "worldnode.hpp"

void setMeshColor(dart::dynamics::SkeletonPtr robot) {
    for(std::size_t i=0; i<robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        for(auto shapeNode : shapeNodes) {
            std::shared_ptr<dart::dynamics::MeshShape> mesh =
                std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
                        shapeNode->getShape());
            if(mesh) {
                mesh->setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR);
            }
        }
    }
}

// std::string GetCurrentWorkingDir(void)
// {
//     const int Max_Size=100;
//     char buff[Max_Size];
//     getcwd(buff,Max_Size);
//     //getcurrent location (default ../build)
//     std::string current_working_dir(buff);
//     // remove "build" string from current path
//     current_working_dir = current_working_dir.substr(0,current_working_dir.find_last_of("\\/"));

//     return current_working_dir;
// }

dart::simulation::WorldPtr loadWorld(){
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;

    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(SIM_MODEL_PATH
            "/Mercury_Model/ground_terrain_check.urdf");
    world->addSkeleton(ground);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(1.0/1500.);
    dart::dynamics::BodyNode* glink = ground->getBodyNode("ground_link");
    glink->setFrictionCoeff(1.0);
    //printf("glink friction: %f\n", glink->getFrictionCoeff());
    return world;
}

dart::dynamics::SkeletonPtr loadRobot(){
    dart::utils::DartLoader urdfLoader;
    // std::string current_path = GetCurrentWorkingDir();

    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            SIM_MODEL_PATH"/Mercury_Model/mercury_passive.urdf");

    // define passive ankle joints
    dart::dynamics::Joint* Ankle1 = robot->getJoint("ankle_r");
    dart::dynamics::Joint* Ankle2 = robot->getJoint("ankle_l");

    Ankle1->setActuatorType(dart::dynamics::Joint::PASSIVE);
    Ankle2->setActuatorType(dart::dynamics::Joint::PASSIVE);

    Ankle1->setSpringStiffness(0, 0.02);
    Ankle2->setSpringStiffness(0, 0.02);

    Ankle1->setDampingCoefficient(0, 0.005);
    Ankle2->setDampingCoefficient(0, 0.005);

    return robot;
}


void setInitialPose(dart::dynamics::SkeletonPtr robot){
    //robot->setPosition(5, 1.05);
    //robot->setPosition(5, 0.7);

    //Eigen::Vector3d JPos_init;
    //JPos_init[0] = 0.0;
    //JPos_init[1] = -0.95;
    //JPos_init[2] = 1.85;

     //JPos_init.setZero();

    //for(int i=0; i<3; i++){
        //robot->setPosition(i+6, JPos_init[i]);
        //robot->setPosition(i+6+4, JPos_init[i]);
    //}
}

int main(int argc, char* argv[]) {
    // Generate world and add skeletons
    dart::simulation::WorldPtr world = loadWorld();
    dart::dynamics::SkeletonPtr robot = loadRobot();
    world->addSkeleton(robot);

    // Initial configuration
    setInitialPose(robot);

    // Set mesh color
    setMeshColor(robot);

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


    // Create viewer
    dart::gui::osg::Viewer viewer;
    // Wrap a worldnode
    osg::ref_ptr<WorldNode> node = new WorldNode(world, msm);
    node->setNumStepsPerCycle(15*2);

    msm->setLight(viewer.getLightSource(0)->getLight());
    viewer.switchHeadlights(false);
    ::osg::Vec3 p1(2.0, 1.2 , 1.);
    viewer.getLightSource(0)->getLight()->setPosition(::osg::Vec4(p1[0], p1[1], p1[2], 0.0));

    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f)); 
    //set the clear mask for the camera
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);


    viewer.addEventHandler(new MercuryEventHandler(node));
    std::cout << viewer.getInstructions() << std::endl;
    //viewer.setUpViewInWindow(0, 0, 1280, 960);
    viewer.setUpViewInWindow(0, 0, 780, 460);
    //viewer.getCameraManipulator()->setHomePosition(
    //::osg::Vec3( 2.57,  3.14, 2.)*2.0,
    //::osg::Vec3( 0.00,  0.00, 1.00),
    //::osg::Vec3(-0.24, -0.25, 0.94));    
    //viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3( 6.14,  3.28, 1.6)*0.75,
            ::osg::Vec3( 0.30,  0.0, 0.6),
            ::osg::Vec3(-0.01, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
     //viewer.record(SIM_DATA_PATH"/mercury_recording");
    viewer.run();

}
