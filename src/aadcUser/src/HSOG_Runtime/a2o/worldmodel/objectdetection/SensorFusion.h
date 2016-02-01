#pragma once

#include "utils/concurrency/Runnable.h"
#include <boost/shared_ptr.hpp>
#include <string>
#include "IResource.h"
#include "utils/geometry/AlignedBoundingBox3D.h"
#include "../ICar.h"
#include "../IMovableObject.h"
#include "../IVisibleObject.h"
#include "../IRoadSign.h"

#include "../impl/VisibleObject.h"
#include "../impl/RoadSign.h"
#include "utils/geometry/Geometry.h"

#include "utils/roadsigns/ISignConstants.h"

namespace A2O {
  
 
class SensorFusion : public Runnable
{

public:
  SensorFusion(IResource *resource);
  
private:
  IResource *_resource;
  
  Polygon::Ptr map(AlignedBoundingBox3D::Ptr& box3d);
  
  std::vector<AlignedBoundingBox3D::Ptr> _recognizedBoxes;
  std::vector<IVisibleObject> _seenVisibleObjects;

  
  void run();
  std::vector<IVisibleObject::Ptr> getObjectsFromCam(const std::vector<AlignedBoundingBox3D::Ptr>& cameraData, 
						     std::vector<AlignedBoundingBox3D::Ptr>& knownBoxes);
  std::vector<IVisibleObject::Ptr> getObjectsFromDistanceSensors(const std::vector<AlignedBoundingBox3D::Ptr>& sensorData,
						      std::vector<AlignedBoundingBox3D::Ptr>& knownBoxes);
  void maintain(std::vector<AlignedBoundingBox3D::Ptr>& boxes);
  

};
};
