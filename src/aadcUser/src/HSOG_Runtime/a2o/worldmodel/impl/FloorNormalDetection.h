#pragma once

#include "carmodel/IPointCloudSensor.h"
#include <eigen3/Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>



namespace A2O
{
class FloorNormalDetection {

public:
		FloorNormalDetection(double defaultCameraHeight);
		Eigen::Vector3d calculateFloorNormal(IPointCloudSensor::ConstPtr pointCloudSensor);
		pcl::PointCloud<pcl::PointXYZ>::Ptr _lastCloud;

private:
		boost::circular_buffer<Eigen::Vector3d> _floorNormals;
		Eigen::Vector3d getAverageOfLastMeasurements();

};
}
