#pragma once

#include "utils/geometry/Pose2D.h"

#include <boost/shared_ptr.hpp>

namespace A2O
{
  
class Segment;

/** \brief The SegmentLink connects two road segments.
 *
 * \author Stefan Glaser
 */
class SegmentLink
{
public:
  typedef boost::shared_ptr<SegmentLink> Ptr;
  typedef boost::shared_ptr<const SegmentLink> ConstPtr;
  
  SegmentLink();
  SegmentLink(const Pose2D& pose,
	      const Pose2D& localSupportPose,
	      boost::shared_ptr<Segment> segmentBefore);
  ~SegmentLink();
  
  void setPose(const Pose2D& pose);
  const Pose2D& getPose() const;
  const Pose2D& getLocalSupportPose() const;
  const Pose2D getGlobalSupportPose() const;
  int getSupportPriority() const;
  
  boost::shared_ptr<Segment> getSegmentBefore() const;
  boost::shared_ptr<Segment> getSegmentAfter() const;
  
  void setSegmentAfter(boost::shared_ptr<Segment> segment);
  
  bool hasSegmentBefore() const;
  bool hasSegmentAfter() const;
  
  bool isLeftOption() const;
  bool isStraightOption() const;
  bool isRightOption() const;
  bool isIntendedOption() const;
  
  bool hasPassedLink(const Pose2D& pose) const;

protected:
  Pose2D _pose;
  
  const Pose2D _localSupportPose;
  
  boost::shared_ptr<Segment> _segmentBefore;
  
  boost::shared_ptr<Segment> _segmentAfter;
};

}
