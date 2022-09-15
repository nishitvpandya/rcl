cmake_minimum_required(VERSION 3.17)

project(difc_mechanism)

find_package(ament_cmake REQUIRED)
find_package(openabe REQUIRED)

set(DIFC_MECHANISM_SOURCES
    src/oabe_c_interface.c
)

add_library(${PROJECT_NAME} ${DIFC_MECHANISM_SOURCES})
ament_target_dependencies(${PROJECT_NAME} openabe)

ament_package()