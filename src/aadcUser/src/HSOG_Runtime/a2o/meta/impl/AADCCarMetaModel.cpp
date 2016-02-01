#include "AADCCarMetaModel.h"
#include "SensorConfig.h"
#include "DistanceSensorConfig.h"
#include "CameraConfig.h"
#include "ActuatorConfig.h"
#include "ServoDriveConfig.h"
#include "AxleConfig.h"
#include "AADCCar.h"

#include <math.h>

#define US_MIN 0.02
#define US_MAX 4
#define US_SCAN_WIDTH 0.15

using namespace A2O;
using namespace Eigen;
using namespace A2O::AADC_Car;

A2O::AADCCarMetaModel::AADCCarMetaModel()
      : CarMetaModel(0.02, STEERING_SERVO, MAIN_MOTOR)
{
  double wheelDiameter = 0.10669747385;
  double axleHeight = (wheelDiameter / 2) - _floorHeight;
  _frontAxle = boost::make_shared<AxleConfig>("frontAxle", Vector3d(0.36, 0, axleHeight), 0.24, wheelDiameter);
  _rearAxle = boost::make_shared<AxleConfig>("rearAxle", Vector3d(0, 0, axleHeight), 0.26, wheelDiameter, Angle::deg(4.5), LEFT_WHEEL_SPEED, RIGHT_WHEEL_SPEED);
  
  Vector3d zUnit(0, 0, 1);
  Vector3d xUnit(1, 0, 0);
  AngleAxisd leftOrientation(M_PI / 2, zUnit);
  AngleAxisd rightOrientation(-1 * M_PI / 2, zUnit);
  AngleAxisd backOrientation(M_PI, zUnit);

  double rad30 = 30.0 * M_PI / 180.0;
  double rad60 = 60.0 * M_PI / 180.0;
  double rad120 = 120.0 * M_PI / 180.0;
  
  // Add Gyroscope configs
  //_gyroConfigs.push_back(boost::make_shared<SensorConfig>(CAR_GYRO, Vector3d(0, 0, 0.09), AngleAxisd::Identity()));
  
  // Add Accelerometer configs
  //_accelerometerConfigs.push_back(boost::make_shared<SensorConfig>(CAR_ACCELEROMETER, Vector3d(0, 0, 0.09), AngleAxisd::Identity()));

  // Add IMU config
  AngleAxisd imuOrientation(M_PI, xUnit);
  imuOrientation = imuOrientation * rightOrientation;
  _imuConfigs.push_back(boost::make_shared<SensorConfig>(CAR_IMU, Vector3d(-0.06, 0, 0.01), imuOrientation));
  
  // Add InfraRed configs

  // Add Ultra Sound configs
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_FRONT_LEFT, Vector3d(0.45, 0.115, 0.02), AngleAxisd(rad60, zUnit), US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_FRONT_CENTER_LEFT, Vector3d(0.47, 0.055, 0.02), AngleAxisd(rad30, zUnit), US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_FRONT_CENTER, Vector3d(0.47, 0, 0.02), AngleAxisd::Identity(), US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_FRONT_CENTER_RIGHT, Vector3d(0.47, -0.055, 0.02), AngleAxisd(-rad30, zUnit), US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_FRONT_RIGHT, Vector3d(0.45, -0.115, 0.02), AngleAxisd(-rad60, zUnit), US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_SIDE_LEFT, Vector3d(0.09, 0.14, 0.02), leftOrientation, US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_SIDE_RIGHT, Vector3d(0.09, -0.14, 0.02), rightOrientation, US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_REAR_LEFT, Vector3d(-0.09, 0.115, 0.02), AngleAxisd(rad120, zUnit), US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_REAR_CENTER, Vector3d(-0.11, 0, 0.02), backOrientation, US_MIN, US_MAX, US_SCAN_WIDTH));
  _ultraSoundConfigs.push_back(boost::make_shared<DistanceSensorConfig>(US_REAR_RIGHT, Vector3d(-0.09, -0.115, 0.02), AngleAxisd(-rad120, zUnit), US_MIN, US_MAX, US_SCAN_WIDTH));
  
  // Add Rotation configs
  _rotationConfigs.push_back(boost::make_shared<SensorConfig>(RIGHT_WHEEL_SPEED, Vector3d(0, -0.123,  0), AngleAxisd::Identity()));
  _rotationConfigs.push_back(boost::make_shared<SensorConfig>(LEFT_WHEEL_SPEED, Vector3d(0, 0.123, 0), AngleAxisd::Identity()));
  
  // Add Voltage configs
  _voltageConfigs.push_back(boost::make_shared<SensorConfig>(VOLT_MEASUREMENT, Vector3d(0, 0, 0), AngleAxisd::Identity()));
  _voltageConfigs.push_back(boost::make_shared<SensorConfig>(VOLT_POWER, Vector3d(0, 0, 0), AngleAxisd::Identity()));
  
  // Add Camera configs
  _cameraConfigs.push_back(boost::make_shared<CameraConfig>(FRONT_CAMERA_1, Vector3d(0.282, 0.02, 0.186), AngleAxisd::Identity(), 640, 480, 552.5, 552.5));
  
  // Add DepthCamera configs
  _depthCameraConfigs.push_back(boost::make_shared<CameraConfig>(FRONT_DEPTH_CAMERA_1, Vector3d(0.282, 0.05, 0.186), AngleAxisd::Identity(), 320, 240, 277.0, 277.0));
  
  // Add ServoDrive configs
  _servoDriveConfigs.push_back(boost::make_shared<ServoDriveConfig>(STEERING_SERVO, -30., 30.));
  
  // Add Motor configs
  _motorConfigs.push_back(boost::make_shared<ActuatorConfig>(MAIN_MOTOR));
  
  // Add Light configs
  _lightConfigs.push_back(boost::make_shared<ActuatorConfig>(HEAD_LIGHTS));
  _lightConfigs.push_back(boost::make_shared<ActuatorConfig>(BACK_LIGHTS));
  _lightConfigs.push_back(boost::make_shared<ActuatorConfig>(BREAK_LIGHTS));
  _lightConfigs.push_back(boost::make_shared<ActuatorConfig>(WARN_LIGHTS));
  _lightConfigs.push_back(boost::make_shared<ActuatorConfig>(INDICATOR_LIGHTS_LEFT));
  _lightConfigs.push_back(boost::make_shared<ActuatorConfig>(INDICATOR_LIGHTS_RIGHT));

  // Add Status configs
  _statusConfigs.push_back(boost::make_shared<ActuatorConfig>(DRIVE_STATUS));
}

A2O::AADCCarMetaModel::~AADCCarMetaModel()
{

}
