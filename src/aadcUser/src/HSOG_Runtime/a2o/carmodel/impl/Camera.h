#pragma once

#include "Sensor.h"
#include "meta/ICameraConfig.h"
#include "carmodel/ICamera.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * The Camera class represents an arbitrary camera.
 * 
 * \author Stefan Glaser
 */
class Camera : public Sensor, public virtual ICamera {
public:
  typedef boost::shared_ptr<Camera> Ptr;
  typedef boost::shared_ptr<const Camera> ConstPtr;

  Camera(ICameraConfig::ConstPtr config);
  virtual ~Camera();
  
  virtual const cv::Mat& getImage() const;
  virtual const unsigned int& getFrameWidth() const;
  virtual const unsigned int& getFrameHeight() const;
  virtual const float& getFocalLengthX() const;
  virtual const float& getFocalLengthY() const;
  
  virtual const Eigen::Vector3d getDirection(const int& pixelX,
					     const int& pixelY) const;
  virtual const Eigen::Vector3d getDirection(const int& pixelX,
					     const int& pixelY,
					     const Eigen::Vector3d& up) const;
  virtual const Eigen::Vector3d getDirection(const int& pixelX,
					     const int& pixelY,
					     const Eigen::Quaterniond& upRot) const;
  virtual const std::pair<int, int> getPixel(const Eigen::Vector3d& direction) const;
  virtual const std::pair<int, int> getPixel(const Eigen::Vector3d& direction,
					     const Eigen::Vector3d& up) const;
  virtual const std::pair<int, int> getPixel(const Eigen::Vector3d& direction,
					     const Eigen::Quaterniond& upRot) const;
  virtual const bool getBottomIntersection(const int& pixelX,
					   const int& pixelY,
					   const Eigen::Quaterniond& upRot,
					   const double& bottomDistance,
					   Eigen::Vector3d& result) const;
  virtual const bool getBottomIntersection(const int& pixelX,
					   const int& pixelY,
					   const Eigen::Vector3d& up,
					   Eigen::Vector3d& result) const;
  virtual const bool toCarSystem(const int& pixelX,
				 const int& pixelY,
				 const Eigen::Quaterniond& upRot,
				 const double& bottomDistance,
				 Eigen::Vector3d& result) const;
  virtual const bool isInFOV(const Eigen::Vector3d& direction) const;
  
  virtual const bool update(IPerception::ConstPtr perception);
  
protected:
  /** The pixel with camera image. */
  const unsigned int _imageWidth;
  
  /** The pixel with camera image. */
  const unsigned int _imageHeight;
  
  /** The half pixel with camera image. */
  const float _halfImageWidth;
  
  /** The half pixel with camera image. */
  const float _halfImageHeight;
  
  /** The focal length in x (horizontal) direction. */
  const float _focalLengthX;
  
  /** The focal length in y (vertical) direction. */
  const float _focalLengthY;
  
  /** The camera image. */
  cv::Mat _image;
};

}
