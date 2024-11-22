################################################################################
#
# Copyright (c) 2024, UnaBiz SAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1 Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  2 Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

cmake_minimum_required(VERSION 3.15)
project(s2lp_rf_api)
include(ExternalProject)

#Check if Project configure by himself or by a parent project (like sigfox-ep-lib)
if ( ${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR} )
    if (NOT DEFINED  SIGFOX_EP_LIB_DIR)
        message(FATAL_ERROR "${PROJECT_NAME} cannot be configured without -DSIGFOX_EP_LIB_DIR=<sigfox-ep-lib directory>")
    endif()
    add_subdirectory(${SIGFOX_EP_LIB_DIR} "${CMAKE_CURRENT_BINARY_DIR}/sigfox-ep-lib")
 else()
    if (NOT DEFINED SIGFOX_EP_LIB_DIR)
        set(SIGFOX_EP_LIB_DIR ${CMAKE_SOURCE_DIR})
        message("${PROJECT_NAME} is Fetched by a parent project")
    endif()
endif()

set (S2LP_RF_API_SOURCES
    src/manuf/s2lp_rf_api.c
    src/board/S2LP_CORE_SPI.c
    src/board/s2lp_hw_api.c
)

set (S2LP_RF_API_HEADERS
    inc/manuf/s2lp_rf_api.h
    inc/board/s2lp_hw_api.h
    inc/board/S2LP_CORE_SPI.h
)

set( CHIP_RF_HEADERS_LIST "")

if (NOT TARGET s2lp_driver)
    list(APPEND CHIP_RF_HEADERS_LIST  ${CMAKE_CURRENT_BINARY_DIR}/drivers-src/s2lp/Drivers/BSP/Components/S2LP/S2LP_Library/inc/)
    list(APPEND CHIP_RF_HEADERS_LIST  ${CMAKE_CURRENT_SOURCE_DIR}/inc/board/)
    ExternalProject_Add(s2lp
        PREFIX drivers-build
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        UPDATE_COMMAND 
            COMMAND bash -c "git -C ${CMAKE_CURRENT_BINARY_DIR}/drivers-src/s2lp/ reset --hard"      
        SOURCE_DIR drivers-src/s2lp
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE
        GIT_REPOSITORY    https://github.com/STMicroelectronics/x-cube-subg2.git
        GIT_TAG           v5.0.0
    )
else ()
    list(APPEND CHIP_RF_HEADERS_LIST "$<BUILD_INTERFACE:$<TARGET_PROPERTY:s2lp_driver,INTERFACE_INCLUDE_DIRECTORIES>>")
endif()

#Add Cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

#Precompile module
find_program(UNIFDEF unifdef)
if(NOT UNIFDEF)
    message(STATUS "unifdef - not found (precompilation target will not be available)")   
else()
    include(precompile_s2lp_rf_api)
endif()

#Target to create object
add_library(${PROJECT_NAME}_obj OBJECT EXCLUDE_FROM_ALL ${S2LP_RF_API_SOURCES})
if (NOT TARGET s2lp_driver)
    add_dependencies(${PROJECT_NAME}_obj s2lp)
endif()
target_include_directories( ${PROJECT_NAME}_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CHIP_RF_HEADERS_LIST}
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_INCLUDE_DIRECTORIES>>
)

#Target to create library
add_library(${PROJECT_NAME} STATIC $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)
if (NOT TARGET s2lp_driver)
    add_dependencies(${PROJECT_NAME} s2lp)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CHIP_RF_HEADERS_LIST}
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_INCLUDE_DIRECTORIES>>
)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${LIB_LOCATION}
    LIBRARY_OUTPUT_DIRECTORY ${LIB_LOCATION}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${LIB_LOCATION}
    PUBLIC_HEADER DESTINATION ${API_LOCATION}  
)
