cmake_minimum_required(VERSION 2.8.3)
project(tabletop_object_detector)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_stl_containers
  geometry_msgs
  household_objects_database_msgs
  message_generation
  moveit_core
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  shape_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(include ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(FILES
  Table.msg
  TabletopDetectionResult.msg
  )

add_service_files(FILES
  AddModelExclusion.srv
  NegateExclusions.srv
  TabletopObjectRecognition.srv
  ClearExclusionsList.srv
  TabletopDetection.srv
  TabletopSegmentation.srv
  )

generate_messages(DEPENDENCIES
  household_objects_database_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    eigen_stl_containers
    geometry_msgs
    household_objects_database_msgs
    message_runtime
    moveit_core
    pcl_ros
    pcl_conversions
    roscpp
    sensor_msgs
    shape_msgs
    std_msgs
    tf
    visualization_msgs
  DEPENDS eigen boost
  INCLUDE_DIRS include
  LIBRARIES
    marker_generator
    tabletop_model_fitter
)

add_library(marker_generator src/marker_generator.cpp )
target_link_libraries(marker_generator ${catkin_LIBRARIES})
add_dependencies(marker_generator ${PROJECT_NAME}_gencpp)

add_library(tabletop_model_fitter
  src/model_fitter.cpp
  src/iterative_distance_fitter.cpp
)
target_link_libraries(tabletop_model_fitter ${catkin_LIBRARIES})
add_dependencies(tabletop_model_fitter ${PROJECT_NAME}_gencpp)

add_executable(tabletop_segmentation src/tabletop_segmentation.cpp )
target_link_libraries(tabletop_segmentation marker_generator ${catkin_LIBRARIES})

add_executable(tabletop_object_recognition
  src/tabletop_object_recognition.cpp
  src/marker_generator.cpp
)
target_link_libraries(tabletop_object_recognition tabletop_model_fitter ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_executable(tabletop_complete_node src/tabletop_complete_node.cpp)
target_link_libraries(tabletop_complete_node ${catkin_LIBRARIES})
add_dependencies(tabletop_complete_node ${PROJECT_NAME}_gencpp)

add_executable(ping_tabletop_segmentation src/ping_tabletop_segmentation.cpp)
target_link_libraries(ping_tabletop_segmentation ${catkin_LIBRARIES})
add_dependencies(ping_tabletop_segmentation ${PROJECT_NAME}_gencpp)

add_executable(ping_tabletop_node src/ping_tabletop_node.cpp)
target_link_libraries(ping_tabletop_node ${catkin_LIBRARIES})
add_dependencies(ping_tabletop_node ${PROJECT_NAME}_gencpp)

add_executable(publish_database_object src/publish_database_object.cpp)
target_link_libraries(publish_database_object ${catkin_LIBRARIES})
add_dependencies(publish_database_object ${PROJECT_NAME}_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS
    marker_generator
    tabletop_model_fitter
    tabletop_segmentation
    tabletop_object_recognition
    tabletop_complete_node
    ping_tabletop_segmentation
    ping_tabletop_node
    publish_database_object
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

