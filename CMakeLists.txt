CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Ray_tracing)

MESSAGE("CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
SET(CMAKE_CXX_STANDARD 11)
include_directories(${PROJECT_SOURCE_DIR}/src)


list(APPEND CMAKE_CXX_FLAGS "-fopenmp -std=c++11  -fpermissive -march=native")

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  message("-- Eigen @ ${EIGEN3_INCLUDE_DIR}")
  message("-- Eigen version: ${EIGEN3_VERSION}")
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_FOUND)


SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(ARCHIVE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

FILE(GLOB source src/*.cpp)
FILE(GLOB header src/*.h)



ADD_LIBRARY(MC_Source ${source})


ADD_EXECUTABLE(test_aabb src/test/test_aabb.cc )
target_link_libraries(test_aabb MC_Source)

ADD_EXECUTABLE(test_model src/test/test_model.cc)
target_link_libraries(test_model MC_Source)

ADD_EXECUTABLE(test_kdtree src/test/test_kdtree.cc)
target_link_libraries(test_kdtree MC_Source)

ADD_EXECUTABLE(test_ray src/test/test_ray.cc)
target_link_libraries(test_ray MC_Source)

ADD_EXECUTABLE(test_radiance src/test/test_radiance.cc)
target_link_libraries(test_radiance MC_Source)

ADD_EXECUTABLE(MC src/main.cc)
target_link_libraries(MC MC_Source)
