#include "Maneuver.h"


using namespace A2O;

Maneuver::Maneuver(std::string name,
		   ICarModel::Ptr carModel,
		   IWorldModel::Ptr worldModel)
      : _name(name), _carModel(carModel), _worldModel(worldModel)
{
	_status = ManeuverStatus::READY;
}

Maneuver::~Maneuver()
{

}

const std::string& Maneuver::getName() const
{
  return _name;
}

const ManeuverStatus Maneuver::getStatus() const
{
	return _status;
}
