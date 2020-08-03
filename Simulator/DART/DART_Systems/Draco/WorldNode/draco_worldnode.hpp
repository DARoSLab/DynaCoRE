#ifndef KRWORLDNODE_H
#define KRWORLDNODE_H

#include <dart/dart.hpp>
//#include <dart/io/io.hpp>
#include <dart/gui/osg/osg.hpp>

#include "../controller.hpp"

class DracoWorldNode : public dart::gui::osg::WorldNode
{
private:
    Controller* mCtrl;

public:
    DracoWorldNode(const dart::simulation::WorldPtr & world);
    //virtual ~DracoWorldNode();

    void customPreStep() override;
};

#endif /* KRWORLDNODE_H */
