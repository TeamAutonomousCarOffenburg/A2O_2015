#ifndef IRIGHTOFWAYANALYZER_H
#define IRIGHTOFWAYANALYZER_H

#include <boost/smart_ptr.hpp>

#include "CrossingType.h"
#include "RoadSignType.h"
#include "CarPositions.h"
#include "DriveDestination.h"
#include "Situation.h"

/*! Right of way analyzer interface
 *  Generates a situation description with right of way and the action to perform.
 */
class IRightOfWayAnalyzer
{
public:
    
  //! Analyzes the situation and generates a situation description.
  /*!
  \param crossingType Type of crossing.
  \param visibleRoadSign Type of visible road sign.
  \param carPositions Car positions.
  \param destination Drive destination.
  \return Situation description with right of way and action to perform.
  */
  virtual Situation_t AnalyzeSituation(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, CarPositions_t carPositions, DriveDestination_t destination) = 0;
};

#endif // IRIGHTOFWAYANALYZER_H
