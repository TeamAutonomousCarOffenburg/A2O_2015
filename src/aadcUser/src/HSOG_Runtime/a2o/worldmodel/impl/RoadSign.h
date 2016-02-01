#pragma once

#include "worldmodel/IRoadSign.h"
#include "VisibleObject.h"
#include "utils/roadsigns/ISignConstants.h"
#include "utils/geometry/Pose2D.h"

#include <boost/smart_ptr.hpp>


namespace A2O {

class RoadSign : public VisibleObject, public virtual IRoadSign {
public:
  typedef boost::shared_ptr<RoadSign> Ptr;
  typedef boost::shared_ptr<const RoadSign> ConstPtr;

  RoadSign(const SignType& type = SignType::Unknown,
	   const Pose2D& pose = Pose2D());
  ~RoadSign();

  virtual const SignType& getSignType() const;

  virtual void update(const Pose2D& pose) override;
  static const std::string getNameForSign(const SignType& type);
  int getSeenCount() const;

private:
  /** The type of road sign. */
  SignType _sign;
  int _seenCount = 0;
};

}
