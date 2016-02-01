// This is a generated file, changes to it may be overwritten in the future.

#pragma pack(push,1)
typedef struct
{
    tUInt8 ui8SOF;
    tUInt8 ui8ID;
    tUInt32 ui32ArduinoTimestamp;
    tUInt8 ui8DataLength;
    tUInt8 ui8Data[25];
} tArduinoData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tBool bEmergencyStop;
} tJuryEmergencyStop;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt8 i8ActionID;
    tInt16 i16ManeuverEntry;
} tJuryStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt8 i8StateID;
    tInt16 i16ManeuverEntry;
} tDriverStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tFloat32 f32Value;
} tSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tBool bValue;
} tBoolSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tUInt32 ui32WheelTach;
    tInt8 i8WheelDir;
} tWheelData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tFloat32 f32Q_w;
    tFloat32 f32Q_x;
    tFloat32 f32Q_y;
    tFloat32 f32Q_z;
    tFloat32 f32A_x;
    tFloat32 f32A_y;
    tFloat32 f32A_z;
} tInerMeasUnitData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32Imagesize;
    tInt16 i16Identifier;
} tRoadSign;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tFloat32 f32XPos;
    tFloat32 f32YPos;
    tFloat32 f32ZPos;
    tUInt8 ui8Desc;
} tGlobalSensorData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16Identifier;
    tFloat32 f32Imagesize;
    tFloat32 af32TVec[3];
    tFloat32 af32RVec[3];
} tRoadSignExt;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSignalValue tFrontLeft;
    tSignalValue tFrontCenterLeft;
    tSignalValue tFrontCenter;
    tSignalValue tFrontCenterRight;
    tSignalValue tFrontRight;
    tSignalValue tSideLeft;
    tSignalValue tSideRight;
    tSignalValue tRearLeft;
    tSignalValue tRearCenter;
    tSignalValue tRearRight;
} tUltrasonicStruct;
#pragma pack(pop)

// The following types are assumed to be known:
// tUInt8
// tUInt32
// tBool
// tInt8
// tInt16
// tFloat32

