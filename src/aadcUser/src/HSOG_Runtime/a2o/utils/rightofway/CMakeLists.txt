cmake_minimum_required(VERSION 2.6)
project(rightofway)

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)

add_executable(rightofway main.cpp RightOfWayAnalyzer.cpp RightOfWayAnalyzerTests.cpp RightOfWayAnalyzerNESWTests.cpp RightOfWayAnalyzerESWTests.cpp RightOfWayAnalyzerNESTests.cpp RightOfWayAnalyzerNSWTests.cpp)

install(TARGETS rightofway RUNTIME DESTINATION bin)
