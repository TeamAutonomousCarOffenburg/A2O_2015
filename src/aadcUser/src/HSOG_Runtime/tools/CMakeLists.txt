cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Qt4 REQUIRED)
set(QT_USE_QTGUI TRUE)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
include(${QT_USE_FILE})

add_library(${A2O_TOOLS_PROJECT}

	tutils/RenderUtil.h
	tutils/RenderUtil.cpp
	tutils/PanelTransformation.h
	tutils/PanelTransformation.cpp
	
	world/renderer/IWorldRenderer.h
	world/renderer/StreetRenderer.h
	world/renderer/StreetRenderer.cpp
	world/renderer/ThisCarRenderer.h
	world/renderer/ThisCarRenderer.cpp

	world/WorldViewer.h
	world/WorldViewer.cpp
	)

include_directories(${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIR}
	../a2o/
	./
	)

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${A2O_TOOLS_PROJECT} ${A2O_LIB})
target_link_libraries(${A2O_TOOLS_PROJECT} ${OpenCV_LIBS})
target_link_libraries(${A2O_TOOLS_PROJECT} ${Boost_LIBRARIES})
target_link_libraries(${A2O_TOOLS_PROJECT} ${PCL_COMMON_LIBRARIES})
target_link_libraries(${A2O_TOOLS_PROJECT} ${QT_LIBRARIES})

add_executable(a2oviz
	main.cpp
	world/resource.qrc
)
target_link_libraries(a2oviz ${A2O_TOOLS_PROJECT})

set(LIBRARY_OUTPUT_PATH	${CMAKE_CURRENT_SOURCE_DIR})
