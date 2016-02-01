#ifndef RIGHTOFWAYANALYZER_H
#define RIGHTOFWAYANALYZER_H

#include "IRightOfWayAnalyzer.h"

class RightOfWayAnalyzer : public IRightOfWayAnalyzer
{
public:
  
  Situation_t AnalyzeSituation(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, CarPositions_t carPositions, DriveDestination_t destination);
  
private:
  CarPositions _generateRightOfWay(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, CarPositions_t carPositions, DriveDestination_t destination);
  CarPositions _getRightBeforeLeft(CrossingType_t crossingType, CarPositions_t carPositions, DriveDestination_t destination);
  Action_t _generateAction(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, DriveDestination_t destination, CarPositions_t rightOfWay);
};

#endif // RIGHTOFWAYANALYZER_H
