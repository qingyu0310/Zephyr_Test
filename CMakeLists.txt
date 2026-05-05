# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(ACTIVE_PRJ "test" CACHE STRING "Active project")
set(PROJ_DIR projects)
set(CONFIG_SYM PRJ_TEST)

# Board root for finding custom boards in this project
set(BOARD_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/projects
  CACHE PATH ""
)

set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/boards/${BOARD}.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_project)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE src)
target_compile_options(app PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)

add_subdirectory(drivers)
add_subdirectory(algorithm)
add_subdirectory(modules)
add_subdirectory(topic)
add_subdirectory(cmd/shell)

if(CONFIG_${CONFIG_SYM})
  target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/apps/Init.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/apps/System_startup.cpp
  )
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/apps
  )
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/thread)
endif()

