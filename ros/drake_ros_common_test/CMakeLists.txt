cmake_minimum_required(VERSION 2.8.3)
project(drake_ros_common_test)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  drake_ros_common
  roscpp)

catkin_package()

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS
#   #DEPENDS
# )

## Specifies additional locations of header files.
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# file(GLOB ${PROJECT_NAME}_SRCS *.cc)
# add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#         FILES_MATCHING PATTERN "*.h")

# message(STATUS "=============================================================")
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# message(STATUS "=============================================================")

# Add the unit tests.
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # find_package(drake_ros_common REQUIRED)
  # include_directories(${CMAKE_INSTALL_PREFIX}/include)
  add_rostest_gtest(
      parameter_server_test
      parameter_server_test.test
      parameter_server_test.cc)
  target_link_libraries(parameter_server_test
      ${catkin_LIBRARIES})
  set_target_properties(parameter_server_test
      PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()