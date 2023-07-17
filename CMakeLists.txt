#
# Copyright (c) 2023 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

cmake_minimum_required(VERSION 3.1)

set(CUSTOM_HEADER_DIR hpp/task-sequencing)
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_USE_TEMPLATE_CSS TRUE)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

set(PROJECT_NAME hpp-task-sequencing)
set(PROJECT_DESCRIPTION
    "Compute sequence of motions to achieve a set of tasks.")

include(cmake/hpp.cmake)
include(cmake/idl.cmake)
include(cmake/python.cmake)

option(CORBA "Implement CORBA server and client" ON)

set(PLUGIN_NAME task-sequencing)
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

list(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

add_project_dependency(hpp-core)

# Library
set(${PROJECT_NAME}_HEADERS
  include/hpp/task-sequencing/fwd.hh
  include/hpp/task-sequencing/multi-robot-solver.hh)
set(${PROJECT_NAME}_SOURCES
  src/multi-robot-solver.cc
  src/isodata/Cluster.h
  src/isodata/Cluster.cpp
  src/isodata/common.h
  src/isodata/error.h
  src/isodata/isodata.h
  src/isodata/isodata.cpp
  src/isodata/MyTime.h
  src/config-distances/distances.h
  src/config-distances/distances.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC hpp-constraints::hpp-constraints)
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

if (CORBA)
  add_project_dependency(hpp-corbaserver)
  add_subdirectory(corba)
endif(CORBA)
