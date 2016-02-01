#pragma once

#include "SegmentLink.h"
#include "worldmodel/impl/RoadSign.h"
#include "worldmodel/impl/DriveInstructionManager.h"
#include "utils/geometry/Angle.h"
#include "utils/geometry/Pose2D.h"
#include "utils/geometry/Circle2D.h"
#include <utils/geometry/Path2D.h>

#include <boost/smart_ptr.hpp>
#include <vector>

namespace A2O {


/** \brief The Segment represents a piece of road.
 * The Segment class represents a piece of road with an circular/straight shape.
 *
 * \author Stefan Glaser
 */
class Segment
{
public:
  typedef boost::shared_ptr<Segment> Ptr;
  typedef boost::shared_ptr<const Segment> ConstPtr;
  
  Segment(const double& length,
	  const Angle& bendingAngle = Angle::Zero());
  ~Segment();
  
  void setBendingAngle(const Angle& bendingAngle);
  const Angle& getBendingAngle() const;
  
  void setLength(const double& lenght);
  const double& getLength() const;
  
  void set(const double& length,
	   const Angle& exitAngle);
 
  SegmentLink::Ptr getClosestLinkToPose(const Pose2D& pose) const;
  
  /** Returns the connected segment, that is placed at the pose */
  static Segment::Ptr getNextOrCurrentSegmentFromPose(Segment::Ptr currentSegment, const Pose2D& pose);

  
  SegmentLink::Ptr getBeginLink() const;
  SegmentLink::Ptr getLeftOption() const;
  SegmentLink::Ptr getStraightOption() const;
  SegmentLink::Ptr getRightOption() const;
  SegmentLink::Ptr getIntendedOption() const;
  
  const bool& consumesDriveInstruction() const;
  
  const std::vector<RoadSign::Ptr>& getRoadSigns() const;
  void attachRoadSigns(const std::vector<RoadSign::Ptr>& roadSigns);
  
  void appendIntermediateWayPoints(const int& howMany,
				   std::vector<Pose2D>& wayPoints) const;
  void appendIntermediateWayPoints(const int& howMany,
				   Path2D::Ptr path) const;
  
  bool hasLeftOption() const;
  bool hasStraightOption() const;
  bool hasRightOption() const;
  bool hasIntendedOption() const;
  
  bool isStraight() const;
  bool isCurve() const;
  bool isLeftCurve() const;
  bool isRightCurve() const;
  bool isCrossing() const;
  bool isXCrossing() const;
  bool isTCrossingLS() const;
  bool isTCrossingLR() const;
  bool isTCrossingSR() const;
  
  bool isLeftOptionAllowed() const;
  bool isStraightOptionAllowed() const;
  bool isRightOptionAllowed() const;
  
  void update(DriveInstructionManager::Ptr driveInstructionManager,
	      size_t instructionIndex);
  
  void onBeginLinkUpdated();
  
  // ===== Static create methods ==========
  static void redefineSegmentBasedOnDetections(Segment::Ptr segment);
  
  static Segment::Ptr createInitialSegment(const Pose2D& initialPose,
					   const double& length,
					   const Angle& bendingAngle = Angle::Zero());
  static Segment::Ptr createInitialTCrossingLR(const Pose2D& initialPose,
					       const Pose2D& initialSupportPose);
  static Segment::Ptr appendSegment(SegmentLink::Ptr fromLink,
				    const double& length,
				    const Angle& bendingAngle = Angle::Zero());
  
  static Segment::Ptr appendXCrossing(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendTCrossingLS(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendTCrossingLR(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendTCrossingSR(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendCurve1MLeft(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendCurve1MRight(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendCurve2MLeft(SegmentLink::Ptr fromLink);
  static Segment::Ptr appendCurve2MRight(SegmentLink::Ptr fromLink);
  
private:
  static Segment::Ptr appendSegmentInternal(SegmentLink::Ptr fromLink,
					    const double& length,
					    const Angle& bendingAngle = Angle::Zero(),
					    bool createStraightOption = true);
  
  void addDetection(std::pair<int, Pose2D>& detections,
		    const Pose2D& newDetection);
  void addDetection(std::pair<int, Circle2D>& detections,
		    const Circle2D& newDetection);
  
  static Pose2D LEFT_EXIT_POSE;
  static Pose2D RIGHT_EXIT_POSE;
  
  double _length;
  
  Angle _bendingAngle;
  
  SegmentLink::Ptr _begin;
  SegmentLink::Ptr _straightOption;
  SegmentLink::Ptr _leftOption;
  SegmentLink::Ptr _rightOption;
  
  SegmentLink::Ptr _intendedOption;
  bool _consumesDriveInstruction;
  
  std::vector<RoadSign::Ptr> _roadSigns;
};
  
}
