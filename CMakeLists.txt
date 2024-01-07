cmake_minimum_required(VERSION 3.5)
project(hybrid_a_star)

add_definitions(-DWITHOUT_NUMPY)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

find_package(Python3 COMPONENTS Development)
find_package(Eigen3 REQUIRED NO_MODULE)
# find_package (glog 0.6.0 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenCV 4.5 REQUIRED)


# THE LOCATION OF OUTPUT BINARIES
set(CMAKE_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_DIR})

include_directories(
        ${CMAKE_SOURCE_DIR}/include/
        ${Python3_INCLUDE_DIRS}
        ${jsoncpp_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

file(GLOB PROJECT_HDRS
        ${PROJECT_SOURCE_DIR}/include/global/*.hpp
        ${PROJECT_SOURCE_DIR}/include/rs_convert/*.hpp
        ${PROJECT_SOURCE_DIR}/include/hybrid_a_star/*.h
)

file(GLOB PROJECT_SRCS
        ${PROJECT_SOURCE_DIR}/src/global/*.cpp
        ${PROJECT_SOURCE_DIR}/src/rs_convert/*.cpp
        ${PROJECT_SOURCE_DIR}/src/hybrid_a_star/*.cpp
)

source_group("Header Files" FILES ${PROJECT_HDRS})
source_group("Source Files" FILES ${PROJECT_SRCS})

# OUTPUT LIBRARIES
add_library(${PROJECT_NAME} SHARED
        ${PROJECT_HDRS}
        ${PROJECT_SRCS}
)

# Link the application with OpenCV libraries
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        -ljsoncpp
        Eigen3::Eigen
        # glog::glog
        #${Python3_LIBRARIES}
        )

# Declare the executable target built from the sources
add_executable(test_reeds_shepp
        ${PROJECT_SOURCE_DIR}/test/test_reeds_shepp.cpp
)

# Link the application with OpenCV libraries
target_link_libraries(test_reeds_shepp
        Eigen3::Eigen
        ${PROJECT_NAME}
)

# Declare the executable target built from the sources
add_executable(test_ha
        test/test_hybrid_A_star.cpp)

# Link the application with OpenCV libraries
target_link_libraries(test_ha
        Eigen3::Eigen
        ${PROJECT_NAME}
        -ljsoncpp
        )
