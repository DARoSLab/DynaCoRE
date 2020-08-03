#include "draco_worldnode.hpp"

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {

    //mCtrl = new JPosController();
}

//DracoWorldNode::~DracoWorldNode() {}

void DracoWorldNode::customPreStep() {
    //mCtrl->update();
}
