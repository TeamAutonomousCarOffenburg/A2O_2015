#include "CameraConfig.h"
#include <boost/concept_check.hpp>


using namespace A2O;


CameraConfig::CameraConfig(const std::string& name,
			   const Eigen::Vector3d& position,
			   const Eigen::AngleAxisd& orientation,
			   const unsigned int& widht,
			   const unsigned int& height,
			   const float& focalLengthX,
			   const float& focalLengthY)
      : CameraConfig(name, name, position, orientation, widht, height, focalLengthX, focalLengthY)
{
}

CameraConfig::CameraConfig(const std::string& name,
			   const std::string& perceptorName,
			   const Eigen::Vector3d& position,
			   const Eigen::AngleAxisd& orientation,
			   const unsigned int& widht,
			   const unsigned int& height,
			   const float& focalLengthX,
			   const float& focalLengthY)
      : SensorConfig(name, perceptorName, position, orientation),
        _imageWidth(widht),
        _imageHeight(height),
        _focalLengthX(focalLengthX),
        _focalLengthY(focalLengthY)
{
}

CameraConfig::~CameraConfig()
{
}

const unsigned int& CameraConfig::getFrameWidth() const
{
  return _imageWidth;
}

const unsigned int& CameraConfig::getFrameHeight() const
{
  return _imageHeight;
}

const float& CameraConfig::getFocalLengthX() const
{
  return _focalLengthX;
}

const float& CameraConfig::getFocalLengthY() const
{
  return _focalLengthY;
}
