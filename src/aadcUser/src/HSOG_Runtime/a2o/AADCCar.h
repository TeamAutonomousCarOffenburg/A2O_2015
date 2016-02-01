#pragma once

#include <string>


namespace A2O {
namespace AADC_Car {

const static std::string LEFT_WHEEL_SPEED = "WH_WheelSpeed_Sensor_Left";
const static std::string RIGHT_WHEEL_SPEED = "WH_WheelSpeed_Sensor_Right";

const static std::string FRONT_CAMERA_1 = "Xtion_RGB";
const static std::string FRONT_DEPTH_CAMERA_1 = "Xtion_DEPTH";

const static std::string US_FRONT_LEFT = "US_Front_Left";
const static std::string US_FRONT_CENTER_LEFT = "US_Front_Center_Left";
const static std::string US_FRONT_CENTER = "US_Front_Center";
const static std::string US_FRONT_CENTER_RIGHT = "US_Front_Center_Right";
const static std::string US_FRONT_RIGHT = "US_Front_Right";

const static std::string US_SIDE_LEFT = "US_Side_Left";
const static std::string US_SIDE_RIGHT = "US_Side_Right";

const static std::string US_REAR_LEFT = "US_Rear_Left";
const static std::string US_REAR_CENTER = "US_Rear_Center";
const static std::string US_REAR_RIGHT = "US_Rear_Right";

const static std::string CAR_IMU = "CarIMU";
const static std::string CAR_GYRO = CAR_IMU;
const static std::string CAR_ACCELEROMETER = CAR_IMU;

const static std::string VOLT_POWER = "VOLT_power_circuit";
const static std::string VOLT_MEASUREMENT = "VOLT_measurement_circuit";

const static std::string MAIN_MOTOR = "MainMotor";
const static std::string STEERING_SERVO = "SteeringServo";

const static std::string HEAD_LIGHTS = "HeadLights";
const static std::string BACK_LIGHTS = "BackLights";
const static std::string BREAK_LIGHTS = "BreakLights";
const static std::string WARN_LIGHTS = "WarnLights";
const static std::string INDICATOR_LIGHTS_LEFT = "IndicatorLightsLeft";
const static std::string INDICATOR_LIGHTS_RIGHT = "IndicatorLightsRight";

const static std::string DRIVE_STATUS = "DriveStatus";
const static std::string JURY_COMMAND = "JuryCommand";
}
}
