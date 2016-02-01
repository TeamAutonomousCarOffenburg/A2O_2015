#pragma once

#include <string>


namespace A2O {

enum class ManeuverStatus{
  STARTUP = -2,
  ERROR = -1,
  READY = 0,
  RUNNING = 1,
  COMPLETE = 2
};

namespace MANEUVER {
  const static std::string HALT = "Halt";
  const static std::string GET_READY = "GetReady";
  const static std::string IS_READY = "IsReady";
  const static std::string DRIVE_VIRTUAL_LANE = "DriveVirtualLane";
  const static std::string DRIVE_WAY_POINTS = "DriveWayPoints";
  const static std::string DRIVE_WAY_POINTS_STRAIGHT = "DriveWayPointsStraight";
  const static std::string FOLLOW_LANE = "FollowLane";
  const static std::string ONE_METER = "OneMeter";
  const static std::string CALIBRATION = "Calibration";
  const static std::string SENDSTATUS_TO_JURY = "SendStatusToJury";
}
}
