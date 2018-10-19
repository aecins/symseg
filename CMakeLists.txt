cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(symseg)
set (CMAKE_BUILD_TYPE Release)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../bin")
add_definitions("-Wall -pedantic")

### ----------------------------------------------------------------------------
### PCL
### ----------------------------------------------------------------------------

find_package(PCL 1.8.0 REQUIRED)
message (STATUS "")
if (PCL_FOUND)
  message (STATUS " -----------------------")
  message (STATUS " Found PCL version ${PCL_VERSION}")
  message (STATUS "     * include dirs: ${PCL_INCLUDE_DIRS}")
  message (STATUS "     * library dirs: ${PCL_LIBRARY_DIRS}")
  message (STATUS "     * libraries:    ${PCL_LIBRARIES}")
else()
  message (FATAL_ERROR : "PCL NOT FOUND!")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### ----------------------------------------------------------------------------
### Boost
### ----------------------------------------------------------------------------

find_package(Boost REQUIRED COMPONENTS regex) 
message (STATUS "")
if(Boost_FOUND)
  message (STATUS " --------------------------")
  message (STATUS " Found Boost version ${Boost_VERSION}")
  message (STATUS "     * include dirs: ${Boost_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${Boost_LIBRARIES}")
else()
  message (FATAL_ERROR : "Boost NOT FOUND!")    
endif()

### ----------------------------------------------------------------------------
### OpenMP
### ----------------------------------------------------------------------------

find_package(OpenMP REQUIRED)
message (STATUS "")
if (OPENMP_FOUND)
  message (STATUS " ------------")
  message (STATUS " Found OpenMP")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
  message (FATAL_ERROR : "OpenMP NOT FOUND!")  
endif()

### ----------------------------------------------------------------------------
### OctoMap
### ----------------------------------------------------------------------------

find_package(octomap 1.8.0 REQUIRED)
message (STATUS "")
if (octomap_FOUND)
  message (STATUS " ---------------------------")
  message (STATUS " Found OctoMap version ${octomap_VERSION}")
  message (STATUS "     * include dirs: ${OCTOMAP_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${OCTOMAP_LIBRARIES}")
else()
  message (FATAL_ERROR : "OctoMap NOT FOUND!")  
endif()
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

### ----------------------------------------------------------------------------
### EDT3D
### ----------------------------------------------------------------------------

find_package(dynamicEDT3D REQUIRED)
message (STATUS "")
if (dynamicEDT3D_FOUND)
  message (STATUS " ---------------------------")
  message (STATUS " Found dynamicEDT3D version ${dynamicedt3d_VERSION}")
  message (STATUS "     * include dirs: ${DYNAMICEDT3D_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${DYNAMICEDT3D_LIBRARIES}")
else()
  message (FATAL_ERROR : "dynamicEDT3D NOT FOUND!")  
endif()
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

### ----------------------------------------------------------------------------
### Protobuffer
### ----------------------------------------------------------------------------

include(FindProtobuf)
find_package(Protobuf REQUIRED)
message (STATUS "")
if (PROTOBUF_FOUND)
  message (STATUS " ---------------------------")
  message (STATUS " Found Protobuf")
  message (STATUS "     * include dirs: ${PROTOBUF_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${PROTOBUF_LIBRARIES}")
else()
  message (FATAL_ERROR : "Protobuf NOT FOUND!")  
endif()

include_directories(${PROTOBUF_INCLUDE_DIRS})

### ----------------------------------------------------------------------------
### Project dependencies
### ----------------------------------------------------------------------------

set (includes ./utilities
              ./occupancy_map
              ./segmentation
              ./symmetry_segmentation)
include_directories (${includes})

### ----------------------------------------------------------------------------
### C++11 support
### ----------------------------------------------------------------------------

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

### ----------------------------------------------------------------------------
### Examples
### ----------------------------------------------------------------------------

add_subdirectory(examples)
add_subdirectory(proto)