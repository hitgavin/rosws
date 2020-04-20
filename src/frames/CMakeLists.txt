cmake_minimum_required(VERSION 2.8.3)
project(frames)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )
  
include_directories(include)

add_library(${PROJECT_NAME}
  include/frames.h
  src/frames.cpp
  )
  
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

if(CMAKE_ENABLE_TESTING)
  catkin_add_gtest(test_frames test/test_frames.cpp)
  target_link_libraries(test_frames
    ${PRJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )