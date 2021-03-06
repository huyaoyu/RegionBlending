# cmake needs this line
cmake_minimum_required(VERSION 3.8)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_BUILD_TYPE Debug)

# Define project name
project(TestBlending)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV 4 REQUIRED PATHS $ENV{OPENCV_ROOT}/lib/cmake/opencv4)

set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS filesystem)
# find_package(Boost REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "Boost_FOUND: ${Boost_FOUND}")
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")

# Eigen.
INCLUDE_DIRECTORIES( $ENV{EIGEN_ROOT} )

# Find PROJ4.
set(CMAKE_CROSSCOMPILING FALSE)
find_package(PROJ4 5.2.0)
message(STATUS "PROJ4_LIBRARIES: ${PROJ4_LIBRARIES}")

# json.
find_package(nlohmann_json REQUIRED)
message(STATUS "nlohmann_json_FOUND: ${nlohmann_json_FOUND}")

# Search for source files.
FILE(GLOB SRC_FILES *.cpp)

message(STATUS "SRC_FILES: ${SRC_FILES}")

# Declare the executable target built from your sources
add_executable(TestBlending_exe ${SRC_FILES})

# Link your application with OpenCV libraries
target_link_libraries(TestBlending_exe PRIVATE nlohmann_json::nlohmann_json ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PROJ4_LIBRARIES})
