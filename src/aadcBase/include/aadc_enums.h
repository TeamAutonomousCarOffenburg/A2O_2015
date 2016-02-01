#ifndef _AADC_ENUMS_H
#define _AADC_ENUMS_H
namespace SensorDefinition
{
    enum ultrasonicSensor{
        usFrontLeft = 1,
        usFrontCenterLeft,
        usFrontCenter,
        usFrontCenterRight,
        usFrontRight,
        usSideRight,
        usSideLeft,
        usRearLeft,
        usRearCenter,
        usRearRight
    };
    
    enum voltageSensor{
        measurementCircuit = 1,
        speedCntrlCircuit,
    };

    enum wheelSensor{
        wheelLeft=1,
        wheelRight,
    };
}

#endif