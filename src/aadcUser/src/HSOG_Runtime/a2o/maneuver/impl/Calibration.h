#pragma once

#include "Maneuver.h"


namespace A2O {

/**
 * This maneuver drives the car for one meter.
 * 
 * \author Stefan Glaser
 */
class Calibration : public Maneuver {
public:
  typedef boost::shared_ptr<Calibration> Ptr;
  typedef boost::shared_ptr<const Calibration> ConstPtr;
  
  Calibration(ICarModel::Ptr carModel,
	       IWorldModel::Ptr worldModel);
  virtual ~Calibration();
  
  virtual void perform();

  void setRadius(double radius)
  {
    _radius = radius;
  }
  
  void setSpeed(int speed)
  {
     _speed = speed;
  }
  
private:
  double _radius;
  int _speed;
  
  
 
  
  
};

}
