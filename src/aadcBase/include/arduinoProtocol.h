/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-22 14:57:13#$ $Rev:: 35411   $
**********************************************************************/
#ifndef ARDUINOPROTOCOL_H
#define ARDUINOPROTOCOL_H


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 tUInt8 boundary */


/*! this headers includes all the struct which are transmitted between the odroid and the arduino. It contains the structs with the sensor data and with the actuator data */

// main frames:

/*! the ARDUINO start of frame identifier */
const tUInt8 ID_ARD_SOF = 0x55;
/*! the ARDUINO empty frame identifier */
const tUInt8 ID_ARD_EMPTY = 0x00;

const tUInt16 INIT_CRC = 0x8989;

/*! the ARDUINO dummy (placeholder) identifier */
const tUInt8 ID_ARD_DUMMY = 0xFF;

// sensor data packages:

/*! the ARDUINO identifier for steering_servo sensor*/
const tUInt8 ID_ARD_SENS_STEERING = 0x11;
/*! the tag for the steering angle*/
typedef struct tag_SensSteeringData
{
    tUInt16 ui16SteeringSens;                /*!< voltage of servo sensor */
} tSensSteeringData;


/*! the ARDUINO identifier for wheelencoders sensor */
const tUInt8 ID_ARD_SENS_WHEEL_RIGHT = 0x21;
const tUInt8 ID_ARD_SENS_WHEEL_LEFT = 0x22;
/*! the tag for the wheel encoder data*/
typedef struct tag_SensWheelData
{
    tUInt32 ui32WheelTach;            /*!< counter from right wheel */
	tInt8 i8WheelDir;
} tSensWheelData;


/*! the ARDUINO identifier for inertial measurement unit; range and interpretation is dependent on mode of IMU; */
const tUInt8 ID_ARD_SENS_IMU = 0x31;
/*! the tag for the wheel imudata data*/
typedef struct tag_ImuData
{ 
  tInt16 i16A_x;                    /*!< acceleration x-axis */
  tInt16 i16A_y;                    /*!< acceleration y-axis */
  tInt16 i16A_z;                     /*!< acceleration z-axis */
  
  tInt16 i16Q_w;                     /*!< attitude */
  tInt16 i16Q_x;                     /*!< attitude */
  tInt16 i16Q_y;                     /*!< attitude */
  tInt16 i16Q_z;                     /*!< attitude */
  
} tImuData;


/*! the ARDUINO identifier for ultrsasonic sensors */
const tUInt8 ID_ARD_SENS_US_FRONT_LEFT = 0x41;
const tUInt8 ID_ARD_SENS_US_FRONT_CENTER_LEFT = 0x42;
const tUInt8 ID_ARD_SENS_US_FRONT_CENTER_RIGHT = 0x43;
const tUInt8 ID_ARD_SENS_US_FRONT_RIGHT = 0x44;
const tUInt8 ID_ARD_SENS_US_FRONT_CENTER = 0x45;
const tUInt8 ID_ARD_SENS_US_SIDE_LEFT = 0x46;
const tUInt8 ID_ARD_SENS_US_SIDE_RIGHT = 0x47;
const tUInt8 ID_ARD_SENS_US_BACK_LEFT = 0x48;
const tUInt8 ID_ARD_SENS_US_BACK_RIGHT = 0x49;
const tUInt8 ID_ARD_SENS_US_BACK_CENTER = 0x4A;
/*! the tag for the ultra sonic data*/
typedef struct tag_UsData
{
    tUInt16 ui16UsData;                /*!< data from ultrasonic sensor front left; value in cm */
} tUsData;

/*! ARDUINO identifier, never use ID_ARD_SOF because of robust parsing and CRC*/
const tUInt8 ID_ARD_SENS_NEVER_USE_IT = ID_ARD_SOF; //0x55
/*! ARDUINO identifier, reserved for future usage */
const tUInt8 ID_ARD_SENS_RESERVED_FRONT = 0x56;
const tUInt8 ID_ARD_SENS_RESERVED_BACK = 0x57;


/*! the ARDUINO identifier for voltage measurements */
const tUInt8 ID_ARD_SENS_VOLT_SPEEDCONT = 0x61;
const tUInt8 ID_ARD_SENS_VOLT_MEAS = 0x62;
/*! the tag for the voltage data*/
typedef struct tag_VoltageData
{
    tUInt16 ui16VoltageData;                /*!< data from voltage measurement pins; has to be calculated */    
} tVoltageData;

/*! the ARDUINO identifier for ERRORs */
const tUInt8 ID_ARD_SENS_ERROR = 0x00; //Unknown Arduino Nr.

typedef struct tag_ErroData
{
    tUInt16 ui16ErrorNr;                /*!< Error Nr */    
} tErrorData;

enum ARD_ERROR{
	ERROR_SOF_NOT_FOUND = 1,
    ERROR_CRC_INVALID = 2,
	ERROR_REMOTE_DETECTED = 3,
	ERROR_NO_GYRO_DETECTED = 4,
};

const tUInt8 ID_ARD_SENSOR_INFO = 0x70;

/* every arduino has it own address */
const tUInt8 ARD_ADDRESS_UNKNOWN = 0;
const tUInt8 ARD_ADDRESS_SENSORSFRONT = 1;
const tUInt8 ARD_ADDRESS_SENSORSBACK = 2;
const tUInt8 ARD_ADDRESS_SENSORSSIDE = 3;
const tUInt8 ARD_ADDRESS_ACTORS = 4;

typedef struct tag_InfoData
{ 
	tUInt8 ui8ArduinoAddress;  //e.g. ARD_ADDRESS_SENSORSFRONT
	tUInt16 ui16ArduinoVersion;             
} tInfoData;


//actuator packages:



/*! the ARDUINO identifier for trigger watchdog */
const tUInt8 ID_ARD_ACT_WATCHDOG = 0xA1;
/*! the tag for the watchdog enable data*/
typedef struct tag_WatchdogData
{
    tUInt8 ui8IsTriggerd;                        /*!< trigger */
} tWatchdogData;

/*! the ARDUINO identifier for trigger watchdog */
const tUInt8 ID_ARD_ACT_EMERGENCY_STOP = 0xA2;
/*! the tag for the motor relais*/
typedef struct tag_EmergencyStopData
{
    tUInt8 ui8IsTriggerd;                        /*!< trigger */
} tEmergencyStopData;



/*! the ARDUINO identifier for steering_servo actuator */
const tUInt8 ID_ARD_ACT_STEER_SERVO = 0xB1;
/*! the ARDUINO identifier for acceleration servo actuator */
const tUInt8 ID_ARD_ACT_SPEED_CONTR = 0xC1;
/*! the tag for the acceleration servo data*/
typedef struct tag_ServoData
{
    tUInt8 ui8Angle;  /*!< angle of servo, range from 0...180, zero position is 90 */
} tServoData;

             
/*! the ARDUINO identifier for enable or disable the lights*/
const tUInt8 ID_ARD_ACT_LIGHT = 0xD1;

/*! these ids are used in the second byte to address the light */
const tUInt8 ID_ARD_ACT_LIGHT_MASK_HEAD = 0x01; 
//const tUInt8 ID_ARD_ACT_LIGHT_MASK_BACK = 0x02; //same Pin as ID_ARD_ACT_LIGHT_MASK_HEAD
const tUInt8 ID_ARD_ACT_LIGHT_MASK_BRAKE = 0x04; 
const tUInt8 ID_ARD_ACT_LIGHT_MASK_TURNLEFT = 0x08; 
const tUInt8 ID_ARD_ACT_LIGHT_MASK_TURNRIGHT = 0x10;
const tUInt8 ID_ARD_ACT_LIGHT_MASK_HAZZARD = 0x18;
const tUInt8 ID_ARD_ACT_LIGHT_MASK_REVERSE = 0x20;


/*! the tag for the light data*/
typedef struct tag_LightData
{
    tUInt8 ui8LightMask;                /*!< SIGNAL Nr. Please see defines*/
} tLightData;



/*! the main frame around every data package which comes from the sensors and actuator*/
typedef struct tag_ArduinoSensorHeader
{
    tUInt8   ui8SOF;                    /*!< the start of frame */
    tUInt8   ui8ID;                    /*!< identifier of package */
    tUInt32  ui32ArduinoTimestamp;        /*!< timestamp from arduino */
    tUInt8   ui8DataLength;            /*!< the length of the data */
} tArduinoSensorHeader;

/*! the main frame around every data package which comes from the sensors and actuator*/
typedef struct tag_ArduinoActorHeader
{
    tUInt8   ui8SOF;                    /*!< the start of frame */
    tUInt8   ui8ID;                    /*!< identifier of package */
} tArduinoActorHeader;

/*! the ARDUINO sensor communication frame with its data area */
typedef union tag_ArduionSensDataUnion
{
    tSensSteeringData      sSteeringData;        /*!< the steering angle data */
    tSensWheelData       sWheelEncData;        /*!< the wheel encoder data */
    tImuData                sImuData;            /*!< the inertial measurement unit data */
    tUsData                 sUsData;            /*!< the ultra sonic sensor data */
    tVoltageData            sVoltageData;        /*!< the voltage data */
    tErrorData			sErrorData;             /*!< the Error data */
	tInfoData			sInfoData;				/*!< the Info data */
} tArduinoSensDataUnion;

/*! the ARDUINO sensor communication frame with its data area */
typedef union tag_ArduionActDataUnion
{
	tLightData	sLightData;
	tServoData  sServoData;
	tEmergencyStopData sEmergencyStopData;
	tWatchdogData sWatchdogData;
   
} tArduinoActDataUnion;

/*! the tag for the arduino sensor frame*/
typedef struct tag_ArduinoSensorFrame
{
    tArduinoSensorHeader      sSensorHeader;    /*!< the header part of the data */
    tArduinoSensDataUnion   sData;        /*!<  variable length contains data structs*/
	tUInt16 ui16CRC;	
} tArduinoSensorFrame;

/*! the tag for the arduino actor frame*/
typedef struct tag_ArduinoActorFrame
{
    tArduinoActorHeader      sActorHeader;    /*!< the header part of the data */
    tUInt8   sData;        /*!<  fixed length contains data structs*/
	tUInt16 ui16CRC;	
} tArduinoActorFrame;


#pragma pack(pop)   /* restore original alignment from stack */

#endif // ARDUINOPROTOCOL_H







