#pragma once

#include "maneuver/IManeuver.h"
#include "maneuver/IManeuverConstants.h"
#include "carmodel/ICarModel.h"
#include "worldmodel/IWorldModel.h"


namespace A2O {

/**
 * The Maneuver class is the basic class for all maneuvers of the car.
 * A maneuver represents some kind of control strategy to drive the car.
 * 
 * \author Stefan Glaser
 */
class Maneuver : public virtual IManeuver {
public:
  Maneuver(std::string name,
	   ICarModel::Ptr carModel,
	   IWorldModel::Ptr worldModel);
  virtual ~Maneuver();
  
  virtual const std::string& getName() const;
  virtual const ManeuverStatus getStatus() const;
  
protected:
  /** The name of the maneuver. */
  std::string _name;
  
  /** Reference to the car model. */
  ICarModel::Ptr _carModel;
  
  /** Reference to the world model. */
  IWorldModel::Ptr _worldModel;

  /** Status of the maneuver */
  ManeuverStatus _status;
};

}
