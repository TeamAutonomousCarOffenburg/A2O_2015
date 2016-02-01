#pragma once

#include "ISensor.h"

#include <opencv/cv.h>
#include <boost/smart_ptr.hpp>


namespace A2O {
  
/**
 * Interface for a Camera sensor.
 * 
 * \author Stefan Glaser
 */
class ICamera : public virtual ISensor {
public:
  typedef boost::shared_ptr<ICamera> Ptr;
  typedef boost::shared_ptr<const ICamera> ConstPtr;

  virtual ~ICamera(){};
  
  /** Retrieve the most recent camera image.
    * \returns the most recent camera image
    */
  virtual const cv::Mat& getImage() const = 0;
  
  /** Retrieve the width of the camera image.
    * \returns the camera image width
    */
  virtual const unsigned int& getFrameWidth() const = 0;
  
  /** Retrieve the height of the camera image.
    * \returns the camera image height
    */
  virtual const unsigned int& getFrameHeight() const = 0;
  
  /** Retrieve the focal length in x (horizontal) direction of the camera.
    * \returns the focal length in x direction
    */
  virtual const float& getFocalLengthX() const = 0;
  
  /** Retrieve the focal length in y (vertial) direction of the camera.
    * \returns the focal length in y direction
    */
  virtual const float& getFocalLengthY() const = 0;
  
  /** Transform the given x, y pixel coordinate to a direction vector, intersecting the plane one meter in front of the camera.
    * \returns a vector pointing into the position that corresponds to the intersection point
    * of the ray resulting in a pixel with the plane one meter in front of the camera.
    */
  virtual const Eigen::Vector3d getDirection(const int& pixelX,
					     const int& pixelY) const = 0;
  
  /** Transform the given x, y pixel coordinate to a direction vector,
    * intersecting the plane one meter in front of the camera, corrected with the upRot rotation.
    * \returns a vector pointing into the position that corresponds to the intersection point
    * of the ray resulting in a pixel with the plane one meter in front of the camera, corrected with the given upRot ratation.
    */
  virtual const Eigen::Vector3d getDirection(const int& pixelX,
					     const int& pixelY,
					     const Eigen::Quaterniond& upRot) const = 0;
  
  /** Transform the given x, y pixel coordinate to a direction vector, intersecting the plane upwards one meter in front of the camera.
    * \returns a vector pointing into the position that corresponds to the intersection point
    * of the ray resulting in a pixel with the plane one meter upwards in front of the camera.
    */
  virtual const Eigen::Vector3d getDirection(const int& pixelX,
					     const int& pixelY,
					     const Eigen::Vector3d& up) const = 0;
  
  /** Retrieve the pixel coodrinate of a given direction vactor.
    * \returns the pixel coordinate corresponding to the given direction vector.
    */
  virtual const std::pair<int, int> getPixel(const Eigen::Vector3d& direction) const = 0;
  
  /** Retrieve the pixel coodrinate of a given direction vactor, corrected by the given upRot rotation.
    * \returns the pixel coordinate corresponding to the given direction vector, corrected by the given upRot rotation.
    */
  virtual const std::pair<int, int> getPixel(const Eigen::Vector3d& direction,
					     const Eigen::Quaterniond& upRot) const = 0;
  
  /** Retrieve the pixel coodrinate of a given direction vactor, corrected by the given up vector.
    * \returns the pixel coordinate corresponding to the given direction vector, corrected by the given up vector.
    */
  virtual const std::pair<int, int> getPixel(const Eigen::Vector3d& direction,
					     const Eigen::Vector3d& up) const = 0;
  
  /** Calculate the bottom intersection point of an given pixel, camera up rotation and bottom distance.
    * \returns the intersection point of the pixel-ray with the bottom
    */
  virtual const bool getBottomIntersection(const int& pixelX,
					   const int& pixelY,
					   const Eigen::Quaterniond& upRot,
					   const double& bottomDistance,
					   Eigen::Vector3d& result) const = 0;
  
  /** Calculate the bottom intersection point of an given pixel and camera up-vector.
    * \returns the intersection point of the pixel-ray with the bottom
    */
  virtual const bool getBottomIntersection(const int& pixelX,
					   const int& pixelY,
					   const Eigen::Vector3d& up,
					   Eigen::Vector3d& result) const = 0;
  
  // Make toCarSystem methods of super class available, since we overload it here
  using ISensor::toCarSystem;
  /** Transform the given pixel coordinate to 3D space relative to the car system. */
  virtual const bool toCarSystem(const int& pixelX,
				 const int& pixelY,
				 const Eigen::Quaterniond& upRot,
				 const double& bottomDistance,
				 Eigen::Vector3d& result) const = 0;
  
  /** Check wether the given direction is inside the field of view of the camera.
    * \returns true, if the given direction is within the field of view of the camera, false otherwise
    */
  virtual const bool isInFOV(const Eigen::Vector3d& direction) const = 0;
};

}
