#pragma once

enum RoadStatus {
	NONE,
	STRAIGHT,
	XCROSSROAD,
	TLEFTRIGHT,
	TLEFT,
	TRIGHT,
	SCRIGHT,
	SCLEFT,
	BCRIGHT,
	BCLEFT
};

class RoadStatusName {

	public:

		static const std::string getNameForRoadStatus(const RoadStatus& status)
		{
			switch(status){
				case NONE:
					return "NONE";
				case STRAIGHT:
					return "STRAIGHT";
				case XCROSSROAD:
					return "XCROSSROAD";
				case TLEFTRIGHT:
					return "TLEFTRIGHT";
				case TLEFT:
					return "TLEFT";
				case TRIGHT:
					return "TRIGHT";
				case SCRIGHT:
					return "SCRIGHT";
				case SCLEFT:
					return "SCLEFT";
				case BCRIGHT:
					return "BCRIGHT";
				case BCLEFT:
					return "BCLEFT";
				default:
					return "UNKNOWN";
			}
		}
};
