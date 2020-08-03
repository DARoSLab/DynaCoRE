#include "MercuryEventHandler.hpp"
#include <stdio.h>

//==============================================================================
MercuryEventHandler::MercuryEventHandler(WorldNode* node) : mNode(node)
{

}

//==============================================================================
bool MercuryEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
  if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    if(ea.getKey() == 'h' || ea.getKey() == 'H')
    {
      //mNode->pelvis_hold = true;
      return true;
    }
    else if(ea.getKey() == 'c' || ea.getKey() == 'C')
    {
      //mNode->pelvis_hold = false;
      return true;
    }
  }


  // The return value should be 'true' if the input has been fully handled
  // and should not be visible to any remaining event handlers. It should be
  // false if the input has not been fully handled and should be viewed by
  // any remaining event handlers.
  return false;
}
