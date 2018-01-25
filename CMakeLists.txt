cmake_minimum_required(VERSION 2.8.10)

project(wasp_registration)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(Boost COMPONENTS filesystem regex)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS})

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})
add_definitions(${CERES_DEFINITIONS})

include_directories(include)

ADD_EXECUTABLE(load_and_view_data include/utils.h
                                 src/utils.cpp
                                 src/load_and_view_data.cpp
)

TARGET_LINK_LIBRARIES(load_and_view_data ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

ADD_EXECUTABLE(icp_registration include/utils.h
                                 src/utils.cpp
                                 src/icp_registration.cpp
)

TARGET_LINK_LIBRARIES(icp_registration ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

ADD_EXECUTABLE(sift_registration include/utils.h
                                 src/utils.cpp
                                 src/sift_registration.cpp
)

TARGET_LINK_LIBRARIES(sift_registration ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
