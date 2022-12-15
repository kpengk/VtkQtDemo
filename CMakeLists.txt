cmake_minimum_required (VERSION 3.17)

project(vtk_qt_demo VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)
find_package(VTK REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

if(VTK_FOUND)
	message (STATUS "VTK_VERSION: ${VTK_VERSION}")
	message(STATUS " VTK_FOUND =  ${VTK_FOUND}" )
	message(STATUS " VTK_MAJOR_VERSION = ${VTK_MAJOR_VERSION}" )
	message(STATUS " VTK_MINOR_VERSION = ${VTK_MINOR_VERSION}" )
	message(STATUS " VTK_BUILD_VERSION = ${VTK_BUILD_VERSION}" )
	message(STATUS " VTK_LIBRARY_DIRS  = ${VTK_LIBRARY_DIRS}" )
	message(STATUS " VTK_LIBRARIES  = ${VTK_LIBRARIES}" )
else()
	message("VTK NOT FOUND: ${VTK_NOT_FOUND_MESSAGE}")
	return()
endif()

set(target_name vtk_qt_demo)

file(GLOB SRCFILE *.h *.hpp *.cpp *.ui)

qt_add_executable(${target_name} MANUAL_FINALIZATION ${SRCFILE})
target_link_libraries(${target_name} PRIVATE ${VTK_LIBRARIES} Qt6::Widgets glfw)
vtk_module_autoinit(TARGETS ${target_name} MODULES ${VTK_LIBRARIES})
