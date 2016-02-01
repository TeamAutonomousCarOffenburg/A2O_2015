cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.2 REQUIRED)
#find_package(aruco)


if(aruco_FOUND)
	set(ARUCO_DEPENDED_FILES
		"worldmodel/signdetection/BitCodeSignDetection.h"
		"worldmodel/signdetection/BitCodeSignDetection.cpp"
		)
	add_definitions(-DARUCO_FOUND)
else(aruco_FOUND)
	message(STATUS "aruco lib not found please install v1.2.5")
endif()

add_library(${A2O_LIB}

	AADCCar.h

	utils/pcl/DepthImageToPointCloudConverter.h
	utils/pcl/utils.h
	utils/geometry/Angle.h
	utils/geometry/Angle.cpp
	utils/geometry/Circle2D.h
	utils/geometry/Circle2D.cpp
	utils/geometry/Geometry.h
	utils/geometry/Geometry.cpp
	utils/geometry/Line2D.h
	utils/geometry/Line2D.cpp
	utils/geometry/LineSegment.h
	utils/geometry/LineSegment.cpp
	utils/geometry/Path2D.h
	utils/geometry/Path2D.cpp
	utils/geometry/Pose2D.h
	utils/geometry/Polygon.h
	utils/geometry/Polygon.cpp
	utils/geometry/Pose2D.cpp
	utils/geometry/Pose3D.h
	utils/geometry/Pose3D.cpp
	utils/geometry/Cosine.h
	utils/geometry/Cosine.cpp
	utils/geometry/BoundingBox2D.h
	utils/geometry/BoundingBox2D.cpp
	utils/geometry/AlignedBoundingBox3D.h
	utils/geometry/AlignedBoundingBox3D.cpp
	utils/geometry/IManeuverGeometry.h
	utils/geometry/drive/Driveable.h
	utils/geometry/drive/Driveable.cpp
	utils/geometry/drive/DriveGeometry.h
	utils/geometry/drive/DriveGeometry.cpp
	utils/geometry/drive/DriveCalculator.h
	utils/geometry/drive/DriveCalculator.cpp
	
	utils/value/ValueUtil.h
	utils/value/ValueUtil.cpp
	utils/value/FuzzyCompare.h
	utils/value/FuzzyCompare.cpp
	utils/value/Distribution.h
	utils/value/Distribution.cpp
	
	utils/roadsigns/ISignConstants.h

	meta/ICarMetaModel.h
	meta/ISensorConfig.h
	meta/ICameraConfig.h
	meta/IActuatorConfig.h
	meta/IServoDriveConfig.h
	meta/IAxleConfig.h
	meta/IDistanceSensorConfig.h

	meta/impl/CarMetaModel.h
	meta/impl/CarMetaModel.cpp
	meta/impl/SensorConfig.h
	meta/impl/SensorConfig.cpp
	meta/impl/CameraConfig.h
	meta/impl/CameraConfig.cpp
	meta/impl/ActuatorConfig.h
	meta/impl/ActuatorConfig.cpp
	meta/impl/ServoDriveConfig.h
	meta/impl/ServoDriveConfig.cpp
	meta/impl/AxleConfig.h
	meta/impl/AxleConfig.cpp
	meta/impl/AADCCarMetaModel.h
	meta/impl/AADCCarMetaModel.cpp
	meta/impl/DistanceSensorConfig.h
	meta/impl/DistanceSensorConfig.cpp

	perception/IPerception.h
	perception/IPerceptor.h
	perception/IValuePerceptor.h
	perception/IGyroPerceptor.h
	perception/IIMUPerceptor.h
	perception/IJuryPerceptor.h
	perception/IAccelerometerPerceptor.h
	perception/ICameraPerceptor.h
	perception/IPointCloudPerceptor.h
	perception/IWheelTickPerceptor.h

	perception/impl/Perception.h
	perception/impl/Perception.cpp
	perception/impl/Perceptor.h
	perception/impl/ValuePerceptor.h
	perception/impl/GyroPerceptor.h
	perception/impl/JuryPerceptor.h
	perception/impl/AccelerometerPerceptor.h
	perception/impl/CameraPerceptor.h
	perception/impl/PointCloudPerceptor.h
	perception/impl/IMUPerceptor.h
	perception/impl/WheelTickPerceptor.h

	action/IAction.h
	action/IEffector.h
	action/IValueEffector.h
	action/IManeuverStatusEffector.h

	action/impl/Action.h
	action/impl/Action.cpp
	action/impl/Effector.h
	action/impl/ValueEffector.h
	action/impl/ManeuverStatusEffector.h

	carmodel/ICarModel.h
	carmodel/IWheel.h
	carmodel/IAxle.h
	carmodel/ISensor.h
	carmodel/IDistanceSensor.h
	carmodel/ITachometer.h
	carmodel/IAccelerometer.h
	carmodel/IGyroscope.h
	carmodel/IIMU.h
	carmodel/ICamera.h
	carmodel/IPointCloudSensor.h
	carmodel/IActuator.h
	carmodel/IServoDrive.h
	carmodel/IMotor.h
	carmodel/ILight.h
	carmodel/IManeuverStatusActuator.h

	carmodel/impl/CarModel.h
	carmodel/impl/CarModel.cpp
	carmodel/impl/Axle.h
	carmodel/impl/Axle.cpp
	carmodel/impl/Wheel.h
	carmodel/impl/Wheel.cpp
	carmodel/impl/Sensor.h
	carmodel/impl/Sensor.cpp
	carmodel/impl/Gyroscope.h
	carmodel/impl/Gyroscope.cpp
	carmodel/impl/Accelerometer.h
	carmodel/impl/Accelerometer.cpp
	carmodel/impl/IMU.h
	carmodel/impl/IMU.cpp
	carmodel/impl/DistanceSensor.h
	carmodel/impl/DistanceSensor.cpp
	carmodel/impl/Tachometer.h
	carmodel/impl/Tachometer.cpp
	carmodel/impl/Camera.h
	carmodel/impl/Camera.cpp
	carmodel/impl/PointCloudSensor.h
	carmodel/impl/PointCloudSensor.cpp
	carmodel/impl/Actuator.h
	carmodel/impl/Actuator.cpp
	carmodel/impl/ServoDrive.h
	carmodel/impl/ServoDrive.cpp
	carmodel/impl/Motor.h
	carmodel/impl/Motor.cpp
	carmodel/impl/Light.h
	carmodel/impl/Light.cpp
	carmodel/impl/ManeuverStatusActuator.h
	carmodel/impl/ManeuverStatusActuator.cpp

	worldmodel/IWorldModel.h
	worldmodel/ISignDetection.h
	worldmodel/IJuryConstants.h
	worldmodel/ILaneDetection.h
	worldmodel/ILaneDetectionModel.h
	worldmodel/IDriveInstructionConstants.h
	worldmodel/IRoadSign.h
	worldmodel/IVisibleObject.h
	worldmodel/IMovableObject.h
	worldmodel/ICar.h
	worldmodel/IThisCar.h

	worldmodel/impl/DriveInstructionManager.h
	worldmodel/impl/DriveInstructionManager.cpp
	worldmodel/impl/FloorNormalDetection.h
	worldmodel/impl/FloorNormalDetection.cpp
	worldmodel/impl/RoadSign.h
	worldmodel/impl/RoadSign.cpp
	worldmodel/impl/VersionIdGenerator.h
	worldmodel/impl/VersionIdGenerator.cpp
	worldmodel/impl/WorldModel.h
	worldmodel/impl/WorldModel.cpp
	worldmodel/impl/WorldModelLogDecorator.h
	worldmodel/impl/WorldState.h
	worldmodel/impl/WorldState.cpp
	worldmodel/impl/WayPointExtractor.h
	worldmodel/impl/WayPointExtractor.cpp
	worldmodel/impl/VisibleObject.h
	worldmodel/impl/VisibleObject.cpp
	worldmodel/impl/MovableObject.h
	worldmodel/impl/MovableObject.cpp
	worldmodel/impl/Car.h
	worldmodel/impl/Car.cpp
	worldmodel/impl/ThisCar.h
	worldmodel/impl/ThisCar.cpp
	
	worldmodel/street/Segment.h
	worldmodel/street/Segment.cpp
	worldmodel/street/SegmentLink.h
	worldmodel/street/SegmentLink.cpp
	worldmodel/street/SegmentDetector.h
	worldmodel/street/SegmentDetector.cpp
	worldmodel/street/ISegmentDetection.h
	worldmodel/street/SegmentDetection.h
	worldmodel/street/StraightDetection.h
	worldmodel/street/StraightDetection.cpp
	worldmodel/street/Curve2Detection.h
	worldmodel/street/Curve2Detection.cpp
	worldmodel/street/CrossingDetection.h
	worldmodel/street/CrossingDetection.cpp

	worldmodel/signdetection/DummySignDetection.h
	worldmodel/signdetection/ISign.h
	worldmodel/signdetection/SignResult.h
	${ARUCO_DEPENDED_FILES}
	
	worldmodel/odometry/WheelTickOdometry.h
	worldmodel/odometry/WheelTickOdometry.cpp
	worldmodel/odometry/SteeringAngleOdometry.h
	worldmodel/odometry/SteeringAngleOdometry.cpp
	worldmodel/odometry/GyroOdometry.h
	worldmodel/odometry/GyroOdometry.cpp


	worldmodel/lanedetection/openCvLane.h
	worldmodel/lanedetection/openCvLane.cpp
	worldmodel/lanedetection/LaneDetectionLogDecorator.h
	worldmodel/lanedetection/LaneDetectionConfiguration.h
	worldmodel/lanedetection/ScanLine.h
	worldmodel/lanedetection/ScanLine.cpp
	worldmodel/lanedetection/Line.h
	worldmodel/lanedetection/Line.cpp
	worldmodel/lanedetection/LineDetection.h
	worldmodel/lanedetection/LineDetector.h
	worldmodel/lanedetection/LineDetector.cpp
	worldmodel/lanedetection/InitialAlignment.h
	worldmodel/lanedetection/RoadStatus.h
	worldmodel/lanedetection/Detection.h
	worldmodel/lanedetection/CurveDetection.h

	worldmodel/IObjectDetection.h
	worldmodel/objectdetection/ObjectDetection.h
	worldmodel/objectdetection/ObjectDetection.cpp
	worldmodel/objectdetection/PointCloudWorker.h
	worldmodel/objectdetection/PointCloudWorker.cpp
	worldmodel/objectdetection/SensorFusion.h
	worldmodel/objectdetection/SensorFusion.cpp
	worldmodel/objectdetection/IResource.h

	utils/concurrency/BlockingQueue.h
	utils/concurrency/Runnable.h
	utils/concurrency/Stopwatch.h
	utils/concurrency/Worker.h

	utils/logger/IEventLogger.h
	utils/logger/EventLogger.h
	utils/logger/EventLogger.cpp
	utils/logger/LogReplayer.h
	utils/logger/LogReplayer.cpp
	
	utils/logger/TimeTaker.cpp
	utils/logger/TimeTaker.h

	utils/driveinstructionreader/DriveInstructionReader.h
	utils/driveinstructionreader/DriveInstructionReader.cpp

	maneuver/IManeuver.h
	maneuver/IManeuverConstants.h

	maneuver/impl/Maneuver.h
	maneuver/impl/Maneuver.cpp
	maneuver/impl/JuryStatus.h
	maneuver/impl/JuryStatus.cpp
	maneuver/impl/HaltManeuver.h
	maneuver/impl/HaltManeuver.cpp
	maneuver/impl/GetReadyManeuver.h
	maneuver/impl/GetReadyManeuver.cpp
	maneuver/impl/IsReadyManeuver.h
	maneuver/impl/IsReadyManeuver.cpp
	maneuver/impl/FollowLaneManeuver.h
	maneuver/impl/FollowLaneManeuver.cpp
	maneuver/impl/DriveVirtualLane.h
	maneuver/impl/DriveVirtualLane.cpp
	maneuver/impl/DriveWayPoints.h
	maneuver/impl/DriveWayPoints.cpp
	maneuver/impl/DriveWayPointsStraight.h
	maneuver/impl/DriveWayPointsStraight.cpp
	maneuver/impl/OneMeter.cpp
	maneuver/impl/OneMeter.h
	maneuver/impl/Calibration.cpp
	maneuver/impl/Calibration.h



	decision/IDecisionMaker.h

	decision/impl/DecisionMaker.h
	decision/impl/DecisionMaker.cpp
	decision/impl/CalibrationDecisionMaker.h
	decision/impl/CalibrationDecisionMaker.cpp
	decision/impl/AADCDecisionMaker.h
	decision/impl/AADCDecisionMaker.cpp


	general/IComponentFactory.h
	general/ComponentFactory.h
	general/ComponentFactory.cpp
	general/LogComponentFactory.h
	general/A2ORuntime.h
	general/A2ORuntime.cpp
	)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIR}) 
include_directories(./)

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${A2O_LIB} ${OpenCV_LIBS})
target_link_libraries(${A2O_LIB} ${Boost_LIBRARIES})
target_link_libraries(${A2O_LIB} ${PCL_COMMON_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})

if (aruco_FOUND)
	include_directories(${aruco_INCLUDE_DIR})
	target_link_libraries(${A2O_LIB} ${aruco_LIBS})
endif()

#add_executable(replayer
#	utils/logger/main.cpp
#)
#target_link_libraries(replayer
#	${A2O_LIB})

set(LIBRARY_OUTPUT_PATH	${CMAKE_CURRENT_SOURCE_DIR})

# install commands
adtf_install_library(${A2O_LIB} ${AADC_INSTALL_DIR_LIBS})
