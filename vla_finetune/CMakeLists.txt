cmake_minimum_required(VERSION 3.12)

project(tdpa-two-robots CXX)

#list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED HINTS ~/libfranka/build)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net)
# find_package(gram_savitzky_golay REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(nlohmann_json REQUIRED)

find_path(ZSTD_INCLUDE_DIR zstd.h)
find_library(ZSTD_LIBRARY NAMES zstd)
if(NOT ZSTD_INCLUDE_DIR OR NOT ZSTD_LIBRARY)
  message(FATAL_ERROR "libzstd development files are required for lossless depth recording")
endif()

### Franka Dynamics Library
add_library(Dynamics STATIC)

target_sources(Dynamics PRIVATE
  include/Dynamics.cpp
)

target_link_libraries(Dynamics Eigen3::Eigen)

### Franka Observers Library
add_library(Observers STATIC)

target_sources(Observers PRIVATE
  include/Observers.cpp
  
)

target_link_libraries(Observers Eigen3::Eigen Dynamics)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  src/examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen)

set(EXAMPLES
  TelePandaTDPA2010AsWhole
)

### test realsense camera ##################################################
add_executable(TestRealSense
    src/test_realsense.cpp
)

target_include_directories(TestRealSense PUBLIC
    src
)

target_link_libraries(TestRealSense
    realsense2
    ${OpenCV_LIBS}
)
############################################################################






foreach(code ${EXAMPLES})
  add_executable(${code} src/${code}.cpp)
  target_include_directories(${code} PUBLIC
    include
    src 
    ${OpenCV_INCLUDE_DIRS} 
    ${ZSTD_INCLUDE_DIR}

  )
  target_link_libraries(${code} Franka::Franka examples_common Eigen3::Eigen Threads::Threads nlohmann_json::nlohmann_json Poco::Net Dynamics ${OpenCV_LIBS} realsense2 ${ZSTD_LIBRARY})
endforeach()
