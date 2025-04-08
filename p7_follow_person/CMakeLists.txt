cmake_minimum_required(VERSION 3.8)
project(p7_follow_person)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
    std_msgs
    vision_msgs
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/p7_follow_person/FollowPersonNode.cpp
  src/p7_follow_person/PIDController.cpp
  src/p7_follow_person/Node_3d_to_tf.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(followPerson src/followPerson_main.cpp)
ament_target_dependencies(followPerson ${dependencies})
target_link_libraries(followPerson ${PROJECT_NAME})

add_executable(coor3d2tf src/coor3d_to_tf_main)
ament_target_dependencies(coor3d2tf ${dependencies})
target_link_libraries(coor3d2tf ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  followPerson
  coor3d2tf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
