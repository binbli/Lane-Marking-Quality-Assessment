cmake_minimum_required(VERSION 3.5.1)
project(Evalutionproject)

find_package(OpenCV 2.4.9.1 REQUIRED)
find_package(Boost COMPONENTS regex system filesystem REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
set(CMAKE_CXX_FLAGS_DEBUG "-g0")


set(ICNET_ROOT "/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/ICNet")
set(PUGIXML_ROOT "/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/pugixml-1.9")
set(LANE_ASSESS "/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/laneAssess")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)
set(ALGLIB_ROOT "/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/ALGLIB")

if (ALGLIB_INCLUDES)
    set (ALGLIB_FIND_QUIETLY TRUE)
endif (ALGLIB_INCLUDES)

find_path (ALGLIB_INCLUDES
        alglibinternal.h
        alglibmisc.h
        ap.h
        dataanalysis.h
        diffequations.h
        fasttransforms.h
        integration.h
        interpolation.h
        linalg.h
        optimization.h
        solvers.h
        specialfunctions.h
        statistics.h
        stdafx.h
        PATHS
        /usr/include/libalglib/
        )

find_library (ALGLIB_LIBRARIES NAMES alglib)
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (ALGLIB DEFAULT_MSG ALGLIB_LIBRARIES ALGLIB_INCLUDES)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# include directory
include_directories(
        /usr/include/python2.7
        /usr/lib/python2.7/dist-packages/numpy/core/include
        /usr/local/include
        /usr/include/hdf5/serial
        /usr/include/gsl/
        /home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/ALGLIB/src
        ${ICNET_ROOT}/PSPNet/.build_release/src
        ${ICNET_ROOT}/PSPNet/src
        ${ICNET_ROOT}/PSPNet/include
        ${PUGIXML_ROOT}/src
        #${LANE_ASSESS}/src
        ${LANE_ASSESS}/include
        ${LANE_ASSESS}/T_linkage
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

#inlcude libs
link_directories(
        ${ICNET_ROOT}/PSPNet/.build_release/lib/
        ${PUGIXML_ROOT}
        /usr/lib/x86_64-linux-gnu/)

# set complile flag
SET(LIBS "-lcaffe -lglog -lboost_system -lopencv_highgui -lopencv_imgproc -lopencv_core -lpugixml -lgsl -lgslcblas -lm")
SET(GCC_COVERAGE_COMPILE_FLAGS "-Wno-sign-compare -Wno-uninitialized -MMD -MP -pthread -fPIC -g -O2")

add_definitions(
        -DCAFFE_VERSION=1.0.0-rc3
        -DUSE_OPENCV
        -DUSE_LEVELDB
        -DUSE_LMDB
        -DCPU_ONLY
)

add_executable(
        lanevaluate
        src/main.cpp
        src/LaneDetector.cpp
        src/RoadSegment.cpp
        src/LMsfromCam.cpp
        src/KPercentExtractor.cpp
        src/VeloPtsProjectCam.cpp
        src/LMsintersection.cpp
        src/SurfaceDataMaker.cpp
        src/Utilities.cpp
        src/PointsVisualizer.cpp
        src/LaneQualityChecker.cpp
        3rdParty/TLinkage/TLinkage.cpp
        3rdParty/TLinkage/BSplineTLinkage.cpp
        3rdParty/TLinkage/KLargestOR.cpp
        3rdParty/TLinkage/MaxDiffOR.cpp
        3rdParty/TLinkage/UniformSampler.cpp
        3rdParty/TLinkage/DistBasedSampler.cpp
        ${ALGLIB_ROOT}/src/alglibinternal.cpp
        ${ALGLIB_ROOT}/src/alglibmisc.cpp
        ${ALGLIB_ROOT}/src/ap.cpp
        ${ALGLIB_ROOT}/src/dataanalysis.cpp
        ${ALGLIB_ROOT}/src/diffequations.cpp
        ${ALGLIB_ROOT}/src/fasttransforms.cpp
        ${ALGLIB_ROOT}/src/integration.cpp
        ${ALGLIB_ROOT}/src/interpolation.cpp
        ${ALGLIB_ROOT}/src/linalg.cpp
        ${ALGLIB_ROOT}/src/optimization.cpp
        ${ALGLIB_ROOT}/src/solvers.cpp
        ${ALGLIB_ROOT}/src/specialfunctions.cpp
        ${ALGLIB_ROOT}/src/statistics.cpp
        3rdParty/LSD/LSDLines.cpp 3rdParty/LSD/LSDLines.h )

target_link_libraries(lanevaluate
        ${OpenCV_LIBS}
        ${LIBS}
        /usr/lib/x86_64-linux-gnu/libgsl.a
        /usr/lib/x86_64-linux-gnu/libgslcblas.a
        )
