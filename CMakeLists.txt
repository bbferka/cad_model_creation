cmake_minimum_required(VERSION 2.8.3)
project(model_creation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x  -Wno-deprecated -DBT_USE_DOUBLE_PRECISION")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
pcl_ros
pcl_conversions
pcl_msgs
geometry_msgs
message_generation
eigen
camera_calibration_parsers
 cv_bridge
  image_transport
)

set(VTK_DIR "/usr/include/vtk-5.8")
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

include(${VTK_USE_FILE})

##add_service_files(
##	FILES
##	SendStampedPose.srv
##)

#generate_messages(   
 # DEPENDENCIES
  #geometry_msgs
  #std_msgs
#)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pcl_tutorials
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib

)




include_directories(SYSTEM
${camera_calibration_parsers_INCLUDE_DIRS}
)
include_directories(include
${catkin_INCLUDE_DIRS}
${opencv_INCLUDE_DIRS}
${model_creation_SOURCE_DIR}/include
)


include_directories(/usr/include/vtk-5.8)
#############################################################################
 ## Hack for QT Creator to include all relevant header and descriptor files ##
 #############################################################################
 #find all header and descriptor files
 execute_process(COMMAND find ${PROJECT_SOURCE_DIR}  -path "${PROJECT_SOURCE_DIR}/include/*" -name "*.h*" -or -name "*.cpp" OUTPUT_VARIABLE RS_FILES_STRING)
 #cmake string to list; has to be done because otherwise add_custom_target would handle    RS_FILES_STRING as one single file, which leads to an error
 string(REPLACE "\n" ";" RS_FILES_LIST ${RS_FILES_STRING})
 add_custom_target(project_additional_files
   SOURCES
   ${RS_FILES_LIST}
 )


## Declare a cpp library
#add_library(aruco
#              src/aruco/arucofidmarkers.cpp
#              src/aruco/cvdrawingutils.cpp
#              src/aruco/cameraparameters.cpp
#              src/aruco/board.cpp
#              src/aruco/marker.cpp
#              src/aruco/boarddetector.cpp
#              src/aruco/markerdetector.cpp
#)
#target_link_libraries(aruco ${catkin_LIBRARIES})

add_executable(main src/main.cpp src/FileReader.cpp src/Estimate_Transformation.cpp)
target_link_libraries(main
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )

add_executable(outline_remove src/outline_remove.cpp)
target_link_libraries(outline_remove
   ${catkin_LIBRARIES} ${VTK_LIBRARIES}
 )
add_executable(interactive_icp src/interactive_icp.cpp)
target_link_libraries(interactive_icp
   ${catkin_LIBRARIES} ${VTK_LIBRARIES}
 )
add_executable(interactive_transform src/interactive_transform.cpp)
target_link_libraries(interactive_transform
   ${catkin_LIBRARIES} ${VTK_LIBRARIES}
 )

add_executable(create_mesh src/create_mesh.cpp)
target_link_libraries(create_mesh
   ${catkin_LIBRARIES} ${VTK_LIBRARIES}
 )
add_executable(estimate_transformation src/estimate_transformation.cpp)
target_link_libraries(estimate_transformation
   ${catkin_LIBRARIES} ${VTK_LIBRARIES}
 )

add_executable(camera_calibration src/camera_calibration.cpp)
target_link_libraries(camera_calibration
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )

add_executable(generate_cam_info src/generate_cam_info.cpp)
target_link_libraries(generate_cam_info
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES} ${camera_calibration_parsers_LIBRARIES}
 )
add_executable(Recognition src/Recognition.cpp)
target_link_libraries(Recognition
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )
add_executable(manual_transform src/manual_transform.cpp)
target_link_libraries(manual_transform
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )

add_executable(aruco_matrix_gen src/aruco_matrix_gen.cpp )
target_link_libraries(aruco_matrix_gen aruco
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )
add_executable(Registration src/Registration.cpp src/FileReader.cpp src/Estimate_Transformation.cpp)
target_link_libraries(Registration
   ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES}
 )
