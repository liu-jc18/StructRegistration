cmake_minimum_required( VERSION 2.8 )
project( struct_reg )

set(CMAKE_CXX_FLAGS "-std=c++11")
add_definitions(-std=c++11)

# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
    OPTION(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON)
ENDIF(OPENMP_FOUND)

IF(OPENMP_FOUND AND WITH_OPENMP)
    MESSAGE(STATUS "With OpenMP ")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=${PROCESSOR_COUNT} -DOPENMP_NUM_THREADS=${PROCESSOR_COUNT} ${OpenMP_CXX_FLAGS} -DOPENMP")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMAX_OPENMP_NUM_THREADS=${PROCESSOR_COUNT} -DOPENMP_NUM_THREADS=${PROCESSOR_COUNT} ${OpenMP_CXX_FLAGS} ${OpenMP_C_FLAGS} -DOPENMP")
ELSE(OPENMP_FOUND AND WITH_OPENMP)
    MESSAGE(STATUS "Without OpenMP")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
    SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
ENDIF(OPENMP_FOUND AND WITH_OPENMP)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)
# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/GaussComp.cpp
src/utils.cpp
src/Register.cpp
src/PlaneFitting.cpp
)

target_link_libraries(${PROJECT_NAME}
${EIGEN3_LIBS}
# ${PCL_LIBRARY_DIRS}
${PCL_LIBRARIES}
pthread
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

add_executable( structreg examples/demo.cpp )
target_link_libraries( structreg ${PROJECT_NAME} )
