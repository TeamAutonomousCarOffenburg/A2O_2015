#pragma once

#include "Maneuver.h"

namespace A2O {

class JuryStatus : public Maneuver {
public:
  typedef boost::shared_ptr<JuryStatus> Ptr;
  typedef boost::shared_ptr<const JuryStatus> ConstPtr;
  
  JuryStatus(ICarModel::Ptr carModel,
	       IWorldModel::Ptr worldModel);
  virtual ~JuryStatus();
  
  virtual void perform();

  void setStatus(const ManeuverStatus& status);
  void setCurrentManeuver(int currentManeuver);
  

  
private:
  ManeuverStatus _status;
  int _currentManeuver;
  
  
};

}
