#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/osg/osg.hpp>

#include "worldnode.hpp"

class MercuryEventHandler : public osgGA::GUIEventHandler
{
public:

  MercuryEventHandler(WorldNode* node);

  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

protected:

  WorldNode* mNode;

};
