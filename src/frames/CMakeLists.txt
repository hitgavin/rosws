cmake_minimum_required(VERSION 2.8.3)
project(frames)

add_compile_options(-std=c++11)

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
  
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
  DEPENDS
  )

# No gtest won't work in my pc
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_frames test/test_frames.cpp)
  target_link_libraries(test_frames
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
  )
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


