#pragma once

#include "utils/geometry/Pose2D.h"
#include <utils/geometry/LineSegment.h>
#include <utils/geometry/Path2D.h>
#include "worldmodel/street/Segment.h"
#include "DriveInstructionManager.h"

#include <vector>


namespace A2O {

/**
 * The WayPointExtractor extracts a list of 2D way points starting from the given street segment.
 *
 * \author Stefan Glaser
 */
class WayPointExtractor {
public:
  static void extractPath(Segment::Ptr currentSegment,
			  DriveInstructionManager::Ptr driveInstructionManager,
			  std::vector<std::pair<SegmentLink::Ptr, bool>>& linkPath,
			  std::vector<Pose2D>& wayPoints);
  
  static void extractWayPoints(Segment::Ptr currentSegment,
			       std::vector<Pose2D>& wayPoints);
  
  static void extractLineSegments(const std::vector<Pose2D>& wayPoints,
				  std::vector<LineSegment>& segments);
  
  static Path2D::Ptr extractPath(Segment::Ptr currentSegment);
  
protected:
  static Pose2D getPriorityPose(SegmentLink::Ptr start,
				SegmentLink::Ptr end);
  
  static std::pair<SegmentLink::Ptr, bool> getExitOption(Segment::Ptr crossing,
							 std::string driveInstruction);
};

}
