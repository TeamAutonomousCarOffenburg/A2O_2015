cmake_minimum_required(VERSION 2.8)
find_package(GTest)

if(GTEST_FOUND)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g3 -O0 -Wall -fPIC" )
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
		OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)
		message(STATUS "GCC Version >= 4.9: enabling extra flags")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize=undefined -fdiagnostics-color=auto" )
	endif (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)

	ADD_SUBDIRECTORY (/usr/src/gtest ${CMAKE_CURRENT_BINARY_DIR}/gtest)
	enable_testing()
	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} )

	find_package(PCL 1.2 REQUIRED)
	find_package(OpenCV REQUIRED)

	add_executable(exampletest test.cpp)
	target_link_libraries(exampletest gtest gtest_main)	

	include_directories(
	${PCL_INCLUDE_DIRS}
	../a2o/
	)

	add_executable(alltests
		WorldModelTest.cpp
		AngleTest.cpp
		Pose2DTest.cpp
		Pose3DTest.cpp
		Circle2DTest.cpp
		GeometryTest.cpp
		Line2DTest.cpp
		LineSegmentTest.cpp
		DriveVirtualLaneTest.cpp
		WheelTickOdometryTest.cpp
		SteeringAngleOdometryTest.cpp
		SegmentTest.cpp
		CosineTest.cpp
		DriveGeometryTest.cpp
		ScanLineTest.cpp
		LineTest.cpp
		LineDetectorTest.cpp
	)
	target_link_libraries(alltests gtest gtest_main)	
	add_test(AllTestsInFoo alltests)

	add_library( liba2op STATIC IMPORTED )
	set_target_properties(liba2op PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../a2o/liba2op.a)
	target_link_libraries(alltests liba2op)
	target_link_libraries(alltests ${OpenCV_LIBS})

else(GTEST_FOUND)
	message("Please install gtest")
endif(GTEST_FOUND)
