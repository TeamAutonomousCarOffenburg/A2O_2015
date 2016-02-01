/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-21 12:25:11#$ $Rev:: 35324   $
**********************************************************************/
/*          ^   x-axis   / 0° 
            |
            |
      ______|______
      |     |      |
      |     |      |
      |     |      |
    __|_____|______|_____>  y-axis  / 270°
      |     |      |
      |     |      |
      |     |      |
      |_____|______|
            |
*/
#ifndef M_PI
	#define M_PI       3.14159265358979323846f
#endif

#define CAR_WIDTH  30
#define CAR_HEIGHT 60
#define US_BEAM_DIM 400
#define US_BEAM_SPAN_ANGLE 30

#define US_FRONT_LEFT_XPOS 30
#define US_FRONT_LEFT_YPOS 20
#define US_FRONT_LEFT_ZPOS 2
#define US_FRONT_LEFT_ZROT 60

#define US_FRONT_CENTER_LEFT_XPOS 30
#define US_FRONT_CENTER_LEFT_YPOS 10
#define US_FRONT_CENTER_LEFT_ZPOS 2
#define US_FRONT_CENTER_LEFT_ZROT 30

#define US_FRONT_CENTER_XPOS 30
#define US_FRONT_CENTER_YPOS 0
#define US_FRONT_CENTER_ZPOS 2
#define US_FRONT_CENTER_ZROT 0

#define US_FRONT_CENTER_RIGHT_XPOS 30
#define US_FRONT_CENTER_RIGHT_YPOS -10
#define US_FRONT_CENTER_RIGHT_ZPOS 2
#define US_FRONT_CENTER_RIGHT_ZROT -30

#define US_FRONT_RIGHT_XPOS 30
#define US_FRONT_RIGHT_YPOS -20
#define US_FRONT_RIGHT_ZPOS 2
#define US_FRONT_RIGHT_ZROT -60

#define US_SIDE_RIGHT_XPOS 10
#define US_SIDE_RIGHT_YPOS -15
#define US_SIDE_RIGHT_ZPOS 2
#define US_SIDE_RIGHT_ZROT -90

#define US_SIDE_LEFT_XPOS 10
#define US_SIDE_LEFT_YPOS 15
#define US_SIDE_LEFT_ZPOS 2
#define US_SIDE_LEFT_ZROT 90

#define US_REAR_LEFT_XPOS -30
#define US_REAR_LEFT_YPOS 20
#define US_REAR_LEFT_ZPOS 2
#define US_REAR_LEFT_ZROT 120

#define US_REAR_CENTER_XPOS -30
#define US_REAR_CENTER_YPOS 0
#define US_REAR_CENTER_ZPOS 2
#define US_REAR_CENTER_ZROT 180

#define US_REAR_RIGHT_XPOS -30
#define US_REAR_RIGHT_YPOS -20
#define US_REAR_RIGHT_ZPOS 2
#define US_REAR_RIGHT_ZROT 240

#define SENSOR_RESERVED_XPOS 25
#define SENSOR_RESERVED_YPOS 0
#define SENSOR_RESERVED_ZPOS 8
#define SENSOR_RESERVED_ZROT 0
