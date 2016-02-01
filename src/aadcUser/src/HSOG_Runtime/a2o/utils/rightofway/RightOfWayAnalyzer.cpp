#include "RightOfWayAnalyzer.h"

Situation_t RightOfWayAnalyzer::AnalyzeSituation(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, CarPositions_t carPositions, DriveDestination_t destination)
{  
  Situation_t situation;
  
  // Set crossing type
  situation.crossingType = crossingType;
  
  // Set visible RoadSign
  situation.visibleRoadSign = visibleRoadSign;
  
  // Set car positions
  situation.carPositions = carPositions;
  
  // Set destination
  situation.destination = destination;
  
  // Set rightOfWay
  situation.rightOfWay = _generateRightOfWay(crossingType, visibleRoadSign, carPositions, destination);
  
  // Set action
  situation.action = _generateAction(crossingType, visibleRoadSign, destination, situation.rightOfWay);  
  
  return situation;
}


Action_t RightOfWayAnalyzer::_generateAction(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, DriveDestination_t destination, CarPositions_t rightOfWay)
{ 
  if(crossingType == NESWCrossing)
  {    
    // Forbid StVO20930, East
    if(visibleRoadSign == StVO20930RoadSign && destination == East)
      return Action::NotPermitted;
    
    // Forbid StVO20930, West
    if(visibleRoadSign == StVO20930RoadSign && destination == West)
      return Action::NotPermitted;
    
  }else if(crossingType == ESWCrossing)
  {
    // Forbid North
    if(destination == North)
      return Action::NotPermitted;
    
    // Forbid StVO20930, East
    if(visibleRoadSign == StVO20930RoadSign && destination == East)
      return Action::NotPermitted;
    
    // Forbid StVO20930, West
    if(visibleRoadSign == StVO20930RoadSign && destination == West)
      return Action::NotPermitted;
    
  }else if(crossingType == NESCrossing)
  {
    // Forbid  West
    if(destination == West)
      return Action::NotPermitted;
    
    // Forbid StVO20930, East
    if(visibleRoadSign == StVO20930RoadSign && destination == East)
      return Action::NotPermitted;
    
  }else if(crossingType == NSWCrossing)
  {
    // Forbid  East
    if(destination == East)
      return Action::NotPermitted;
    
    // Forbid StVO20930, West
    if(visibleRoadSign == StVO20930RoadSign && destination == West)
      return Action::NotPermitted;
 
  }
  
  // Check right of way
  if(rightOfWay.east || rightOfWay.north || rightOfWay.west)
  {
    // Wait!
    return Action::Wait; 
  }else
  {
    if(visibleRoadSign == StVO206RoadSign)
    {
      // Stop then drive!
      return Action::StopThenDrive;
    }else
    {
      // Drive!
      return Action::Drive;
    }
  }
}

CarPositions RightOfWayAnalyzer::_generateRightOfWay(CrossingType_t crossingType, RoadSignType_t visibleRoadSign, CarPositions_t carPositions, DriveDestination_t destination)
{
  CarPositions rightOfWay;
   
  // Init rightOfWay
  rightOfWay.north = false;
  rightOfWay.east = false;
  rightOfWay.west = false;
    
  // Tread NoRoadSign as StVO102RoadSign
  if(visibleRoadSign == NoRoadSign) visibleRoadSign = StVO102RoadSign;

  // Set rightOfWay (for all crossing types)
  switch(visibleRoadSign)
  {
    case StVO102RoadSign:
    case StVO20930RoadSign:
      // Right before Left!
      return _getRightBeforeLeft(crossingType, carPositions, destination);
    case StVO205RoadSign:
    case StVO206RoadSign:
      rightOfWay.north = carPositions.north;
      rightOfWay.east = carPositions.east;
      rightOfWay.west = carPositions.west;
      break;
    case StVO301RoadSign:
      if(destination == West) rightOfWay.north = carPositions.north;
      break;
    }
  
  return rightOfWay;
}

CarPositions RightOfWayAnalyzer::_getRightBeforeLeft(CrossingType_t crossingType, CarPositions_t carPositions, DriveDestination_t destination)
{
  CarPositions_t rightOfWay;
  
  // Init rightOfWay
  rightOfWay.north = false;
  rightOfWay.east = false;
  rightOfWay.west = false;
  
  // Case #3 (see documentation)
  if(carPositions.north == true && carPositions.east == false && carPositions.west == false)
  {
    if(destination == West)
    {
      rightOfWay.north = carPositions.north;
    }
    
    return rightOfWay;
  }
  
  // Case #4 (see documentation)
  if(carPositions.north == true && carPositions.east == false && carPositions.west == true)
  {
    if(destination == West)
    {
      rightOfWay.north = carPositions.north;
      
      // Special case (see documentation)
      if(crossingType == NESWCrossing)
      {
	rightOfWay.west = carPositions.west;
      }
    }
    
    return rightOfWay;
  }
  
  // Case #5 (see documentation)
  if(carPositions.north == false && carPositions.east == true && carPositions.west == false)
  {
    rightOfWay.east = carPositions.east;
    return rightOfWay;
  }
  
  // Case #6 (see documentation)
  if(carPositions.north == false && carPositions.east == true && carPositions.west == true)
  {
    rightOfWay.east = carPositions.east;
    return rightOfWay;
  }

  // Case #7 (see documentation)
  if(carPositions.north == true && carPositions.east == true && carPositions.west == false)
  {
    rightOfWay.north = carPositions.north;
    rightOfWay.east = carPositions.east;
    return rightOfWay;
  }
  
  // Case #8 (see documentation)
  if(carPositions.north == true && carPositions.east == true && carPositions.west == true)
  {
    rightOfWay.north = carPositions.north;
    rightOfWay.east = carPositions.east;
    rightOfWay.west = carPositions.west;
    return rightOfWay;
  }
  
  return rightOfWay;
}
