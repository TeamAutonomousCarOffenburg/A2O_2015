#include "Camera.h"
#include "utils/geometry/Geometry.h"


#define MAX_DISTANCE 1000000

using namespace A2O;
using namespace Eigen;


Camera::Camera(ICameraConfig::ConstPtr config)
      : Sensor(config),
        _imageWidth(config->getFrameWidth()),
        _imageHeight(config->getFrameHeight()),
        _halfImageWidth(float(_imageWidth - 1) / 2),
        _halfImageHeight(float(_imageHeight - 1) / 2),
        _focalLengthX(config->getFocalLengthX()),
        _focalLengthY(config->getFocalLengthY())
{
}

Camera::~Camera()
{
}

const cv::Mat& Camera::getImage() const
{
  return _image;
}

const unsigned int& Camera::getFrameWidth() const
{
  return _imageWidth;
}

const unsigned int& Camera::getFrameHeight() const
{
  return _imageHeight;
}

const float& Camera::getFocalLengthX() const
{
  return _focalLengthX;
}

const float& Camera::getFocalLengthY() const
{
  return _focalLengthY;
}

const Vector3d Camera::getDirection(const int& pixelX,
				    const int& pixelY) const
{
  double y = std::tan((_halfImageWidth - pixelX) / _focalLengthX);
  double z = std::tan((_halfImageHeight - pixelY) / _focalLengthY);
  return Vector3d(1, y, z);
}

const Vector3d Camera::getDirection(const int& pixelX,
				    const int& pixelY,
				    const Quaterniond& upRot) const
{
  return Vector3d(upRot * getDirection(pixelX, pixelY));
}

const Vector3d Camera::getDirection(const int& pixelX,
				    const int& pixelY,
				    const Vector3d& up) const
{
  Quaterniond upRot;
  upRot.setFromTwoVectors(up, Vector3d::UnitZ());
  
  return getDirection(pixelX, pixelY, upRot);
}

const std::pair<int, int> Camera::getPixel(const Vector3d& direction) const
{
  if (direction(0) == 0) {
    return std::pair<int, int>(-1, -1);
  }
  
  int px = int(0.5 + _halfImageWidth - std::atan(direction(1) / direction(0)) * _focalLengthX);
  int py = int(0.5 + _halfImageHeight - std::atan(direction(2) / direction(0)) * _focalLengthY);
  
  return std::pair<int, int>(px, py);
}

const std::pair<int, int> Camera::getPixel(const Vector3d& direction,
					   const Quaterniond& upRot) const
{
  return getPixel(upRot * direction);
}

const std::pair<int, int> Camera::getPixel(const Vector3d& direction,
					   const Vector3d& up) const
{ 
  Quaterniond upRot;
  upRot.setFromTwoVectors(Vector3d::UnitZ(), up);
  
  return getPixel(upRot * direction);
}

const bool Camera::getBottomIntersection(const int& pixelX,
					 const int& pixelY,
					 const Quaterniond& upRot,
					 const double& bottomDistance,
					 Vector3d& result) const
{
  Vector3d dir = getDirection(pixelX, pixelY, upRot);
  
  if (dir(2) > -bottomDistance / MAX_DISTANCE) {
    result = dir * MAX_DISTANCE;
    return false;
  }
  
  double scale = (-1 / dir(2)) * bottomDistance;
  
  result = dir * scale;
  return true;
}

const bool Camera::getBottomIntersection(const int& pixelX,
					 const int& pixelY,
					 const Vector3d& up,
					 Vector3d& result) const
{
  Quaterniond upRot;
  upRot.setFromTwoVectors(up, Vector3d::UnitZ());
  
  return getBottomIntersection(pixelX, pixelY, upRot, Geometry::getNorm(up), result);
}

const bool Camera::toCarSystem(const int& pixelX,
			       const int& pixelY,
			       const Quaterniond& upRot,
			       const double& bottomDistance,
			       Vector3d& result) const
{
  bool success = getBottomIntersection(pixelX, pixelY, upRot, bottomDistance, result);
  
  result = _pose * result;
  
  return success;
}

const bool Camera::isInFOV(const Vector3d& direction) const
{
  if (direction(0) <= 0) {
    return false;
  }
  
  double horizontalAngle = std::atan(direction(1) / direction(0));
  double verticalAngle = std::atan(direction(2) / direction(0));
  
  if (horizontalAngle < -29 || horizontalAngle > 29) {
    return false;
  } else if (verticalAngle < -22.5 || verticalAngle > 22.5) {
    return false;
  }
  
  return true;
}

const bool Camera::update(IPerception::ConstPtr perception)
{
  ICameraPerceptor::ConstPtr camPerceptor = perception->getCameraPerceptor(_perceptorName);
  
  if (camPerceptor.get()) {
    _image = camPerceptor->getImage();
    _lastMeasurementTime = camPerceptor->getTime();
    return true;
  }
  
  return false;
}
