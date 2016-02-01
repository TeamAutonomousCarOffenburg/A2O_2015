### PROJECT SETTINGS ####
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

project(aev_connectionlib)

## target settings ##
find_package (AEV_CONNECTION REQUIRED)

# Qt for GUI
find_package(Qt4 COMPONENTS QtCore QtGui QtMain QtXml)
if(MSVC)
    set(QT_USE_QTMAIN TRUE)
endif(MSVC)
include(${QT_USE_FILE})

set_property(GLOBAL PROPERTY USE_FOLDERS true)

# define target name
set(EXECUTABLE_NAME aadc_juryModule_ConnectionLib)

#we build this example only if qt was found 
if(NOT QT_FOUND)
    message(FATAL_ERROR "Qt is required to build this executable.")
endif(NOT QT_FOUND)

# define macro to Qt-moc files (borrowed from ADTF)
macro(add_moc NAME)
    set(_stdafx)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/stdafx.h)
      set(_stdafx -fstdafx.h)
    endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/stdafx.h)

    set(BUILD_MOC true)
    set(${NAME}_MOC_TMP)
    foreach(_header ${ARGN})
    if(_header STREQUAL NO_BUILD)
        set(BUILD_MOC false)
    else(_header STREQUAL NO_BUILD)
        get_filename_component(_headername ${_header} NAME_WE)
        get_filename_component(_headerfile ${_header} ABSOLUTE)
        set(_mocfile ${CMAKE_CURRENT_BINARY_DIR}/moc_${_headername}.cpp)
        add_custom_command(
            OUTPUT ${_mocfile}
            COMMAND ${QT_MOC_EXECUTABLE}
            ARGS ${_stdafx} -f${_headerfile} ${_headerfile} -o ${_mocfile}
            MAIN_DEPENDENCY ${_headerfile}
        )
        set(${NAME}_MOC_TMP ${${NAME}_MOC_TMP} ${_mocfile})
    endif(_header STREQUAL NO_BUILD)    
    endforeach(_header ${ARGN})

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    if(BUILD_MOC)
    set(${NAME}_MOC ${${NAME}_MOC} ${${NAME}_MOC_TMP})
    endif(BUILD_MOC)
endmacro(add_moc NAME)

macro(add_ui NAME)
    set(${NAME}_UI_TMP)
    foreach(_uifile ${ARGN})
        get_filename_component(_uiheader ${_uifile} NAME_WE)
        get_filename_component(_uifile ${_uifile} ABSOLUTE)
        set(_uiheader ${CMAKE_CURRENT_BINARY_DIR}/ui_${_uiheader}.h)
        add_custom_command(
            OUTPUT ${_uiheader}
            COMMAND ${QT_UIC_EXECUTABLE}
            ARGS -o ${_uiheader} ${_uifile}
            MAIN_DEPENDENCY ${_uifile}
        )
        set(${NAME}_UI_TMP ${${NAME}_UI_TMP} ${_uiheader})
        include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
    endforeach(_uifile ${ARGN})

    set(${NAME}_UI ${${NAME}_UI_TMP})
endmacro(add_ui NAME)

# set files for moc
set(HEADER_MOC
    cJuryModule.h
    )
        
# moc the files
add_moc(${EXECUTABLE_NAME}
        ${HEADER_MOC}
        )
        
add_ui(${EXECUTABLE_NAME}
        ./jury_module.ui)

# add the aadcBase/include folder
include_directories(../../../include)


# add executable target
add_executable(${EXECUTABLE_NAME} WIN32   #name of the executable to create
                stdafx.h                       #all necessary header/sourcefiles
                stdafx.cpp
                main.cpp
                coder_description.h	
                cJuryModule.cpp
                cJuryModule.h
                ${HEADER_MOC}
                ${${EXECUTABLE_NAME}_UI}
                ${${EXECUTABLE_NAME}_MOC}
                )
                
# set target properties
if(NOT WIN32)
    set_target_properties(${EXECUTABLE_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN/")
endif(NOT WIN32)

# link against needed libs
target_link_libraries(${EXECUTABLE_NAME} 
                        ${AEV_CONNECTION_LIB} 
                        ${AEV_CONNECTION_LIBRARY_EXTERNAL_LIBS} 
                        ${AEV_CONNECTION_EXECUTABLE_LIBS}
                        ${QT_LIBRARIES})

# install the executable
aev_connection_install_target(${EXECUTABLE_NAME} bin)

# install the connection lib shared libraries
if (WIN32)
    install(FILES
        ${AEV_CONNECTION_DIR}/lib/${AEV_CONNECTION_LIB}d_${AEV_CONNECTION_LIB_VERSION}.dll
        DESTINATION bin/debug
        CONFIGURATIONS Debug
    )
    install(FILES
        ${AEV_CONNECTION_DIR}/lib/${AEV_CONNECTION_LIB}_${AEV_CONNECTION_LIB_VERSION}.dll
        DESTINATION bin
        CONFIGURATIONS RelWithDebInfo Release
    )
    
    install(FILES ${QT_BINARY_DIR}/QtGui4.dll DESTINATION bin CONFIGURATIONS RelWithDebInfo Release MinSizeRel)
    install(FILES ${QT_BINARY_DIR}/QtCore4.dll DESTINATION bin CONFIGURATIONS RelWithDebInfo Release MinSizeRel)
    install(FILES ${QT_BINARY_DIR}/QtGuid4.dll DESTINATION bin/debug CONFIGURATIONS Debug)
    install(FILES ${QT_BINARY_DIR}/QtCored4.dll DESTINATION bin/debug CONFIGURATIONS Debug)
    install(FILES ${QT_BINARY_DIR}/QtXml4.dll DESTINATION bin CONFIGURATIONS RelWithDebInfo Release MinSizeRel)
    install(FILES ${QT_BINARY_DIR}/QtXmld4.dll DESTINATION bin/debug CONFIGURATIONS Debug)
else (WIN32)
    install(FILES
        ${AEV_CONNECTION_DIR}/lib/lib${AEV_CONNECTION_LIB}d_${AEV_CONNECTION_LIB_VERSION}.so
        DESTINATION bin/debug
        CONFIGURATIONS Debug
    )
    install(FILES
        ${AEV_CONNECTION_DIR}/lib/lib${AEV_CONNECTION_LIB}_${AEV_CONNECTION_LIB_VERSION}.so
        DESTINATION bin
        CONFIGURATIONS RelWithDebInfo Release
    )
    file(GLOB ADTF_QTCORE 
            ${QT_LIBRARY_DIR}/libQtCore*.so* )
    file(GLOB ADTF_QTGUI 
            ${QT_LIBRARY_DIR}/libQtGui*.so* )
    file(GLOB ADTF_QTXML 
            ${QT_LIBRARY_DIR}/libQtXml.so* )
    set(ADTF_QT_LIBS ${ADTF_QTCORE} ${ADTF_QTGUI} ${ADTF_QTXML})
    install(FILES ${ADTF_QT_LIBS} DESTINATION bin CONFIGURATIONS RelWithDebInfo Release MinSizeRel)
    install(FILES ${ADTF_QT_LIBS} DESTINATION bin CONFIGURATIONS Debug)
endif (WIN32)
