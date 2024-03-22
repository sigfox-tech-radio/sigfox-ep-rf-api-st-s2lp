cmake_minimum_required(VERSION 3.15)
project(s2lp_rf_api)
include(ExternalProject)

#Check if Project configure by himself or by a parent project (like sigfox-ep-lib)
if ( ${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR} )
    if (NOT DEFINED  SIGFOX_EP_LIB_DIR)
        message(FATAL_ERROR "${PROJECT_NAME} cannot be configured without -DSIGFOX_EP_LIB_DIR=<sigfox-ep-lib directory>")
    endif()
    add_subdirectory(${SIGFOX_EP_LIB_DIR} "${CMAKE_CURRENT_BINARY_DIR}/${SIGFOX_EP_LIB_DIR}")
 else()
    set(SIGFOX_EP_LIB_DIR ${CMAKE_SOURCE_DIR})
    message("${PROJECT_NAME} is Fetched by a parent project")
endif()

set (S2LP_RF_API_SOURCES
    src/manuf/s2lp_rf_api.c
    src/board/s2lp_hw_api.c
)

set (S2LP_RF_API_HEADERS
    inc/manuf/s2lp_rf_api.h
    inc/board/s2lp_hw_api.h
)

set( CHIP_RF_HEADERS_LIST "")

if (NOT TARGET s2lp_driver)
    list(APPEND CHIP_RF_HEADERS_LIST  ${CMAKE_CURRENT_BINARY_DIR}/drivers-src/s2lp/Drivers/BSP/Components/S2LP/S2LP_Library/inc/)
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
        GIT_REPOSITORY    https://github.com/STMicroelectronics/x-cube-sfxs2lp1.git
        GIT_TAG           v4.0.0
        PATCH_COMMAND git apply ${CMAKE_CURRENT_LIST_DIR}/cmake/s2lp.patch
    )
else ()
    list(APPEND CHIP_RF_HEADERS_LIST "$<BUILD_INTERFACE:$<TARGET_PROPERTY:s2lp_driver,INTERFACE_INCLUDE_DIRECTORIES>>")
endif()

#Add Cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

#Precompile module
include(precompile_s2lp_rf_api)

add_library(${PROJECT_NAME}_obj OBJECT EXCLUDE_FROM_ALL src/manuf/s2lp_rf_api.c)

if (NOT TARGET s2lp_driver)
add_dependencies(${PROJECT_NAME}_obj s2lp)
endif()
target_include_directories( ${PROJECT_NAME}_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${SIGFOX_EP_LIB_DIR}/inc
    ${CHIP_RF_HEADERS_LIST}
)
target_compile_definitions(${PROJECT_NAME}_obj PUBLIC ${DEF_FLAG_LIST})

add_library(${PROJECT_NAME} STATIC ${S2LP_RF_API_SOURCES})
add_dependencies(${PROJECT_NAME}  precompil_${PROJECT_NAME} s2lp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PRECOMPIL_DIR}/inc ${CHIP_RF_HEADERS_LIST})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF_FLAG_LIST})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${LIB_LOCATION}
    LIBRARY_OUTPUT_DIRECTORY ${LIB_LOCATION}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${LIB_LOCATION}
    PUBLIC_HEADER DESTINATION ${API_LOCATION}  
)
