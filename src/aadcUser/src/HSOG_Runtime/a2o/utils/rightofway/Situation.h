#ifndef SITUATION_H
#define SITUATION_H

#include "CrossingType.h"
#include "RoadSignType.h"
#include "CarPositions.h"
#include "DriveDestination.h"
#include "Action.h"

/*! Full situation description with right of way and action to perform */
typedef struct Situation
{
  /*! Type of crossing */
  CrossingType_t crossingType;
  
  /*! Type of visible road sign */
  RoadSignType_t visibleRoadSign;
  
  /*! Car positions */
  CarPositions_t carPositions;
  
  /*! My destination */
  DriveDestination_t destination;
  
  /*! Right of way car positions */
  CarPositions_t rightOfWay;
  
  /*! The action to perform */
  Action_t action;
  
} Situation_t;

#endif