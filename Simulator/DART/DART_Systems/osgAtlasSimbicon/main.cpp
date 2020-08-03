/*
 * Copyright (c) 2011-2017, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "AtlasSimbiconWorldNode.hpp"
#include "AtlasSimbiconEventHandler.hpp"
#include "AtlasSimbiconWidget.hpp"
#include <SIM_Configuration.h>

//#include <osgShadow/ShadowedScene>
//#include <osgShadow/ShadowMap>
#include <osgShadow/LightSpacePerspectiveShadowMap>


int main()
{
    //osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene;
    //shadowedScene = new osgShadow::ShadowedScene;
    //osgShadow::ShadowSettings* settings = shadowedScene->getShadowSettings();
    //settings->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    //settings->setCastsShadowTraversalMask(CastsShadowTraversalMask);
    osg::ref_ptr<osgShadow::MinimalShadowMap> msm = NULL;

    msm = new osgShadow::LightSpacePerspectiveShadowMapDB;
    //shadowedScene->setShadowTechnique( msm.get() );

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

    //::osg::ref_ptr<osgShadow::MinimalShadowMap> msm = NULL;
    //msm = new osgShadow::LightSpacePerspectiveShadowMapDB;

    //msm->setMinLightMargin( minLightMargin );
    //msm->setMaxFarPlane( maxFarPlane );
    //msm->setTextureSize( ::osg::Vec2s( texSize, texSize ) );
    //msm->setShadowTextureCoordIndex( shadowTexUnit );
    //msm->setShadowTextureUnit( shadowTexUnit );
    //msm->setBaseTextureCoordIndex( baseTexUnit );
    //msm->setBaseTextureUnit( baseTexUnit );


    // Create a world
    dart::simulation::WorldPtr world(new dart::simulation::World);


    // Load ground and Atlas robot and add them to the world
    dart::utils::DartLoader urdfLoader;
    auto ground = urdfLoader.parseSkeleton(SIM_MODEL_PATH"/Atlas/ground.urdf");
    //auto ground = urdfLoader.parseSkeleton(SIM_MODEL_PATH"/Ground/ground_terrain.urdf");
    auto atlas = urdfLoader.parseSkeleton(SIM_MODEL_PATH"/Atlas/atlas_v3_no_head.urdf");
    //auto atlas = dart::utils::SdfParser::readSkeleton(
    //SIM_MODEL_PATH"/Atlas/atlas_v3_no_head.sdf");
    world->addSkeleton(ground);
    world->addSkeleton(atlas);

    //shadowedScene->addChild(atlas);
    // Set initial configuration for Atlas robot
    atlas->setPosition(0, -0.5 * dart::math::constantsd::pi());

    // Set gravity of the world
    world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

    // Wrap a WorldNode around it
    osg::ref_ptr<AtlasSimbiconWorldNode> node
        = new AtlasSimbiconWorldNode(world, atlas, msm);
    node->setNumStepsPerCycle(20);

    //node->addChild(shadowedScene);
    //shadowedScene->addChild(node);
    // Create a Viewer and set it up with the WorldNode
    dart::gui::osg::ImGuiViewer viewer;
    viewer.addWorldNode(node);
    msm->setLight(viewer.getLightSource(0)->getLight());
    //viewer.setSceneData(shadowedScene.get());
    // Add control widget for atlas
    //viewer.getImGuiHandler()->addWidget(
            //std::make_shared<AtlasSimbiconWidget>(&viewer, node.get()));
    //viewer.getLightSource(1)->setLocalStateSetModes(::osg::StateAttribute::OFF);

    // Pass in the custom event handler
    viewer.addEventHandler(new AtlasSimbiconEventHandler(node));

    // Set the dimensions for the window
    viewer.setUpViewInWindow(0, 0, 1280, 960);

    // Set the window name
    viewer.realize();
    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    windows.front()->setWindowName("Atlas Simbicon");

    // Adjust the viewpoint of the Viewer
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3d( 5.14,  3.28, 6.28)*1.0,
            ::osg::Vec3d( 1.00,  0.00, 0.00),
            ::osg::Vec3d( 0.00,  0.1, 0.00));
    // We need to re-dirty the CameraManipulator by passing it into the viewer
    // again, so that the viewer knows to update its HomePosition setting
    viewer.setCameraManipulator(viewer.getCameraManipulator());

    // Begin running the application loop
    viewer.run();
}
