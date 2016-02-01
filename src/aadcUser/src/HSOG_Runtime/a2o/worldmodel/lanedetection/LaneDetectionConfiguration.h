#pragma once

struct LaneDetectionConfiguration{
	    int height = 480;
	    int width = 640;
	    int yOffset = 240;
	      
	    int rightLaneStartPos = 20;
	      
	    int CANNY_MIN_TRESHOLD = 150;
	    int CANNY_MAX_TRESHOLD = 250;
	      
	    int heightCamera = 215;
	    float focalVertical = 552.5;
	    float focalHorizontal = 552.5;
	    float xCenter = 319.5;
};