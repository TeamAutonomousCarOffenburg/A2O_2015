cmake_minimum_required(VERSION 2.8)

if(ADTF_FOUND)
	# needed for aruco on car, since its is in /opt...
	set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

	find_package(PCL 1.2 REQUIRED)
	find_package(OpenCV REQUIRED)

	include_directories(${OpenCV_INCLUDE_DIR})
	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(../a2o/)
	include_directories(../tools/)

	adtf_add_filter(camera_calibration
		filters/cameraCalibration.cpp
		filters/cameraCalibration.h
	)
	target_link_libraries(camera_calibration ${OpenCV_LIBS})

	adtf_add_filter(tsignalvalue_generator
		filters/tsignalvalue_generator.h
		filters/tsignalvalue_generator.cpp
	)

	adtf_use_qt(QtCore QtGui)
	if(QT_FOUND)

		set(HEADER_MOC
			filters/runtime_filter.h
			../tools/world/WorldViewer.h
			)
	
	adtf_add_moc(hsog_runtime_service
		${HEADER_MOC}
		)
	
	adtf_add_qrc(hsog_runtime_service
		../tools/world/resource.qrc
		)

	adtf_add_filter(hsog_runtime_service
		${HEADER_MOC}
		../tools/world/WorldViewer.cpp
		../tools/world/renderer/IWorldRenderer.h
		../tools/world/renderer/GridRenderer.h
		../tools/world/renderer/GridRenderer.cpp
		../tools/world/renderer/StreetRenderer.h
		../tools/world/renderer/StreetRenderer.cpp
		../tools/world/renderer/ThisCarRenderer.h
		../tools/world/renderer/ThisCarRenderer.cpp
		
		../tools/tutils/PanelTransformation.h
		../tools/tutils/PanelTransformation.cpp
		../tools/tutils/RenderUtil.h
		../tools/tutils/RenderUtil.cpp

		decoder/ADTFPinMessageDecoder.h
		decoder/ADTFPinMessageDecoder.cpp
		decoder/InputPin.h
		decoder/Decoder.h
		decoder/Decoder.cpp
		decoder/GyroDecoder.h
		decoder/GyroDecoder.cpp
		decoder/AccelerometerDecoder.h
		decoder/AccelerometerDecoder.cpp
		decoder/IMUDecoder.h
		decoder/IMUDecoder.cpp
		decoder/ValueGroupDecoder.h
		decoder/ValueGroupDecoder.cpp
		decoder/DepthImageDecoder.h
		decoder/DepthImageDecoder.cpp
		decoder/CameraDecoder.h
		decoder/CameraDecoder.cpp
		decoder/JuryDecoder.h
		decoder/JuryDecoder.cpp
		decoder/WheelTickDecoder.h
		decoder/WheelTickDecoder.cpp

		encoder/ADTFPinMessageEncoder.h
		encoder/ADTFPinMessageEncoder.cpp
		encoder/OutputPin.h
		
		filters/runtime_filter.h
		filters/runtime_filter.cpp
	)
	
	target_link_libraries(hsog_runtime_service 
		${A2O_LIB}
#		${PCL_LIBRARIES}
		)
	adtf_install_plugin(hsog_runtime_service ${AADC_INSTALL_DIR})
	
	endif(QT_FOUND)


	adtf_add_filter(hsog_rate_limit
		encoder/ADTFPinMessageEncoder.h
		encoder/ADTFPinMessageEncoder.cpp
		encoder/OutputPin.h
		filters/rate_limit.h
		filters/rate_limit.cpp
	)
	target_link_libraries(hsog_rate_limit ${A2O_LIB})
	
	# install the filters 
	adtf_install_plugin(tsignalvalue_generator ${AADC_INSTALL_DIR})
	adtf_install_plugin(hsog_rate_limit ${AADC_INSTALL_DIR})
	adtf_install_plugin(camera_calibration ${AADC_INSTALL_DIR})

else(ADTF_FOUND)
	message(WARNING "adtf not found, not compiling filters")
endif(ADTF_FOUND)