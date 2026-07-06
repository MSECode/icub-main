# Copyright (C) 2026 Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(legTrajectoryPlayer)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HDF5_PKG REQUIRED hdf5)

add_executable(${PROJECT_NAME} main.cpp)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE ${HDF5_PKG_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${HDF5_PKG_LDFLAGS})
target_compile_options(${PROJECT_NAME} PRIVATE ${HDF5_PKG_CFLAGS_OTHER})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
