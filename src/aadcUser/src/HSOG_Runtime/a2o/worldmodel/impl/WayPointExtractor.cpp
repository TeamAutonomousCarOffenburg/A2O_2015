#include "WayPointExtractor.h"

#include "worldmodel/IDriveInstructionConstants.h"
#include <utils/geometry/Geometry.h>


using namespace A2O;

void WayPointExtractor::extractPath(Segment::Ptr currentSegment,
				    DriveInstructionManager::Ptr driveInstructionManager,
				    std::vector<std::pair<SegmentLink::Ptr, bool>>& linkPath,
				    std::vector<Pose2D>& wayPoints)
{
  Segment::Ptr runningSegment(currentSegment);
  std::pair<SegmentLink::Ptr, bool> exitOption;
  size_t instructionIndex = driveInstructionManager->getCurrentInstructionIndex();
  
//   // Try to place one way point in the past as first way point
//   if (runningSegment->getBeginLink()->hasSegmentBefore()) {
//     wayPoints.push_back(getPriorityPose(runningSegment->getBeginLink()->getSegmentBefore()->getBeginLink(), runningSegment->getBeginLink()));
//   }

  // Run through all known segments and plan way points along them as long as we have driving instructions left
  // (I added 5 to drive a bit further than we have a valid plan)
  while (runningSegment && instructionIndex < driveInstructionManager->getNumberOfInstructions() + 5) {
    // Determine end link
    if (runningSegment->isCrossing()) {
      exitOption = getExitOption(runningSegment, driveInstructionManager->getInstruction(instructionIndex));
    } else {
      exitOption.first = runningSegment->getStraightOption();
      exitOption.second = false;
    }

    // Add support pose of begin link, or the backprojected support pose of the end link
    // (based on priority) to the list of way points
    wayPoints.push_back(getPriorityPose(runningSegment->getBeginLink(), exitOption.first));
    
    if (runningSegment->getLength() > 1.81) {
      runningSegment->appendIntermediateWayPoints(int(runningSegment->getLength() - 0.8), wayPoints);
    }

    // Add exit option to link path
    linkPath.push_back(exitOption);
    
    // Progress to the next segment
    runningSegment = exitOption.first->getSegmentAfter();
  }

  // Extract way point at final link if we reached an open ended segment
  if (!runningSegment && exitOption.first) {
    wayPoints.push_back(exitOption.first->getGlobalSupportPose());
  }
}

void WayPointExtractor::extractWayPoints(Segment::Ptr currentSegment,
					 std::vector<Pose2D>& wayPoints)
{
  Segment::Ptr runningSegment(currentSegment);
  SegmentLink::Ptr exitLink;

  // Run through all known segments that have an intended exit link
  while (runningSegment && runningSegment->hasIntendedOption()) {
    exitLink = runningSegment->getIntendedOption();

    // Add support pose of begin link, or the backprojected support pose of the end link
    // (based on priority) to the list of way points
    wayPoints.push_back(getPriorityPose(runningSegment->getBeginLink(), exitLink));

    if (runningSegment->getLength() > 1.81) {
      runningSegment->appendIntermediateWayPoints(int(runningSegment->getLength() - 0.8), wayPoints);
    }

    // Progress to the next segment
    runningSegment = exitLink->getSegmentAfter();
  }

  // Extract way point at final link if we reached an open ended segment
  if (!runningSegment && exitLink) {
    wayPoints.push_back(exitLink->getGlobalSupportPose());
  }
}

Pose2D WayPointExtractor::getPriorityPose(SegmentLink::Ptr start,
					  SegmentLink::Ptr end)
{
  if (start->getSupportPriority() <= end->getSupportPriority()) {
    return start->getGlobalSupportPose();
  } else {
    Pose2D supportPose(end->getLocalSupportPose());
    return start->getPose() * Pose2D(-supportPose.x(), supportPose.y(), supportPose.getAngle());
  }
}

std::pair<SegmentLink::Ptr, bool> WayPointExtractor::getExitOption(Segment::Ptr crossing,
								   std::string driveInstruction)
{
  std::pair<SegmentLink::Ptr, bool> option;

  if (crossing->isLeftOptionAllowed()
      && (driveInstruction == DRIVE_INSTRUCTION::DRIVE_LEFT || driveInstruction == DRIVE_INSTRUCTION::PULL_OUT_LEFT)) {
    option.first = crossing->getLeftOption();
    option.second = true;
  } else if (crossing->isStraightOptionAllowed() && driveInstruction == DRIVE_INSTRUCTION::DRIVE_STRAIGHT) {
    option.first = crossing->getStraightOption();
    option.second = true;
  } else if (crossing->isRightOptionAllowed()
      && (driveInstruction == DRIVE_INSTRUCTION::DRIVE_RIGHT || driveInstruction == DRIVE_INSTRUCTION::PULL_OUT_RIGHT)) {
    option.first = crossing->getRightOption();
    option.second = true;
  } else {
    // We have no matching instruction for this crossing, decide for the first best option
    if (crossing->hasStraightOption()) {
      option.first = crossing->getStraightOption();
    } else if (crossing->hasLeftOption()) {
      option.first = crossing->getLeftOption();
    } else {
      option.first = crossing->getRightOption();
    }
    option.second = false;
  }

  return option;
}

void WayPointExtractor::extractLineSegments(const std::vector<Pose2D>& wayPoints,
					    std::vector<LineSegment>& segments)
{
  if (wayPoints.size() < 2) {
    return;
  }
  
  for (size_t i = 1; i < wayPoints.size(); i++) {
    Geometry::constructDoubleArc(wayPoints[i - 1], wayPoints[i], segments);
  }
}

Path2D::Ptr WayPointExtractor::extractPath(Segment::Ptr currentSegment)
{
  Path2D::Ptr path(boost::make_shared<Path2D>());
  Segment::Ptr runningSegment(currentSegment);
  SegmentLink::Ptr exitLink;

  // Run through all known segments that have an intended exit link
  while (runningSegment && runningSegment->hasIntendedOption()) {
    exitLink = runningSegment->getIntendedOption();

    // Add support pose of begin link, or the backprojected support pose of the end link
    // (based on priority) to the list of way points
    path->addWayPoint(getPriorityPose(runningSegment->getBeginLink(), exitLink));

    if (runningSegment->getLength() > 1.81) {
      runningSegment->appendIntermediateWayPoints(int(runningSegment->getLength() - 0.8), path);
    }

    // Progress to the next segment
    runningSegment = exitLink->getSegmentAfter();
  }

  // Extract way point at final link if we reached an open ended segment
  if (!runningSegment && exitLink) {
    path->addWayPoint(exitLink->getGlobalSupportPose());
  }
  
  return path;
}
