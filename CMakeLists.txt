cmake_minimum_required(VERSION 3.3)
project(indoor_geometry_reocnstruction)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(dir  "../Mapping/Tools/indoor_geometry_reocnstruction/Debug")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir})

set (MAPPING_LIB_DIR "../Mapping/Library/lib64")
set (MAPPING_INCLUDE_DIR "../Mapping/Library")

# for boost library
set(Boost_INCLUDE_DIR ../Program\ Files/boost_gcc4.7/include/boost-1_64)
set(Boost_LIBRARY_DIR ../Program\ Files/boost_gcc4.7/lib)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../Mapping/Library/Photogrammetry/include)
include_directories(../Mapping/Library/Buildings/include)
include_directories(../Mapping/Library/LaserScan/include)
include_directories(../Mapping/Foreign/ANN/include/ANN)
include_directories(../Mapping/Foreign/ANN/src)
include_directories(../Mapping/Tools/visualization_tools)
#include_directories(../Mapping/Tools/building_modelling/pcm/include)
#link_directories(../Mapping/Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

set(SOURCE_FILES main.cpp indoorTopology.cpp setConsoleColor.cpp segment_refinement.cpp FilterSegmentsByTime.cpp
          Buffer.cpp Buffers.h GenerateWallPatches.cpp ReadAscii.cpp Laservoxel.cpp
        Laserpoint2ScanningPosition.cpp DepthMap.cpp WallAccuracy.cpp indoor_reconstruction.h MLS_preprocessing.cpp
        post_processing.cpp post_processing.h #../visualization_tools/VisualizePlane3D.cpp
        ../visualization_tools/MinimumRectangle3D.cpp LAtexTest.tex
        PlaneFittingResidual.h PlaneFittingResidual.cpp test_functions.cpp TrajectoryManipulation.cpp
        TrajectoryManipulation.h space_partitioning.cpp space_partitioning.h Directory_Processing.cpp
        Directory_Processing.h occlusion_test2.cpp )#MergeSurfaces.cpp)
add_executable(indoor_reconstruction ${SOURCE_FILES} indoor_reconstruction.h)
target_link_libraries(indoor_reconstruction ${Boost_LIBRARIES})
