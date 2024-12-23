#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.0)
add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)


project(rogio_threads)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  genmsg
  roscpp
  rospy
  std_msgs
  image_transport
  sensor_msgs
)


## CUDA
find_package(CUDA REQUIRED)
IF (CUDA_FOUND)
    include_directories("${CUDA_INCLUDE_DIRS}")
    # Optional: Specify the arch of your CUDA hardware here
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++14 -gencode arch=compute_86,code=sm_86)
ENDIF ()

find_package(Pangolin REQUIRED)
#set(OpenCV_INCLUDE_DIRS "/usr/local/include/usr/local/include/opencv")
#set(OpenCV_DIR  "/usr/local/share/OpenCV")

find_package(OpenCV 4.5.0  REQUIRED
NO_MODULE # should be optional, tells CMake to use config mode
PATHS /usr/local # look here
NO_DEFAULT_PATH) # and don't look anywhere else
#find_package(OpenCV REQUIRED)
if (OpenCV_INCLUDE_DIRS)
    message(${OpenCV_INCLUDE_DIRS})
    message(${OpenCV_DIR})
    include_directories("${OpenCV_INCLUDE_DIRS}")
    link_directories(${OpenCV_DIR}/lib)
endif (OpenCV_INCLUDE_DIRS)


## Eigen3
find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif (EIGEN3_INCLUDE_DIR)



generate_messages(
DEPENDENCIES std_msgs
)

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Weffc++")

# Targets
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)



catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rogio_threads  src/main.cpp)


include_directories("${PROJECT_INCLUDE_DIR}" "${PROJECT_SOURCE_DIR}/cuda/include" "${PROJECT_SOURCE_DIR}/cuda/include/voxel_hash")
file(GLOB rogio_SRCS "${PROJECT_SOURCE_DIR}/cuda/*.cu" "${PROJECT_SOURCE_DIR}/*.cpp")

cuda_add_library(rogio_threads_lib STATIC "${rogio_SRCS}")
target_link_libraries(rogio_threads ${catkin_LIBRARIES})
target_link_libraries(rogio_threads "${OpenCV_LIBS}" rogio_threads_lib)
target_link_libraries(rogio_threads ${Pangolin_LIBRARIES})
