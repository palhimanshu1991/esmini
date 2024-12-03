# This is a minimalistic example of how to create build files for an application
# making use of the esmini shared library.
#
# Make sure that the headerfile esminiLib.hpp and library esminiLib
# (exact filename depending on platform) is available, either in this folder or in
# the original location (relative this folder).

cmake_minimum_required(VERSION 3.7.1)

set(TARGET esmini-player)
set(LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Libraries/esminiLib)
link_directories(${LINK_DIRECTORIES})

project(${TARGET})

set_property(
    TARGET ${target}
    PROPERTY POSITION_INDEPENDENT_CODE
             ON)

add_executable(${TARGET} main.cpp)

target_link_libraries(${TARGET} PRIVATE esminiLib_static)
