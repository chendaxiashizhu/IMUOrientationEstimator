cmake_minimum_required(VERSION 2.8.3)
project(IMUOrientationEstimator)
# Set C++14
add_compile_options(-std=c++14)

# Release mode.
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
# MYNTEYE
find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

# 包含了所有的头文件
include_directories(include
${EIGEN3_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED 
    src/Initializer.cpp
    src/Estimator.cpp
    src/Propagator.cpp
    src/Update.cpp
    src/MahonyAHRSupdate.cpp
)
target_link_libraries(${PROJECT_NAME} 
    ${EIGEN3_LIBS}
    mynteye)
    
add_executable(RunMyntEye example/RunMyntEye.cpp)
target_link_libraries(RunMyntEye ${PROJECT_NAME})

add_executable(Mahonyfilter example/Mahonyfilter.cpp)
target_link_libraries(Mahonyfilter ${PROJECT_NAME})

add_executable(runall example/all.cpp)
target_link_libraries(runall ${PROJECT_NAME})

