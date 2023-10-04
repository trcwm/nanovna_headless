cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(nanovna VERSION 0.1.0)

########################################################################
# Check for CPM
########################################################################

set(CPM_DOWNLOAD_VERSION 0.38.2)

if(CPM_SOURCE_CACHE)
    set(CPM_DOWNLOAD_LOCATION "${CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
elseif(DEFINED ENV{CPM_SOURCE_CACHE})
    set(CPM_DOWNLOAD_LOCATION "$ENV{CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
else()
    set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
endif()

if(NOT (EXISTS ${CPM_DOWNLOAD_LOCATION}))
    message(STATUS "Downloading CPM.cmake to ${CPM_DOWNLOAD_LOCATION}")
    file(DOWNLOAD "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake" ${CPM_DOWNLOAD_LOCATION}
    )
endif()

message("${CPMstatus}")

include(${CPM_DOWNLOAD_LOCATION})

########################################################################
# INSTALL AND ADD libopencm3
########################################################################

CPMAddPackage("gh:libopencm3/libopencm3#master")

# create a target to build libopencm3 -- only for the target we need
add_custom_target(libopencm3 make CC=${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc AR=${TOOLCHAIN_PATH}/bin/arm-none-eabi-ar
    BYPRODUCTS ${libopencm3_SOURCE_DIR}/lib/libopencm3_stm32f0.a
    USES_TERMINAL
    WORKING_DIRECTORY ${libopencm3_SOURCE_DIR})

# Create a specific CPU target with the appropriate options etc
add_library(stm32f072 STATIC IMPORTED)
target_include_directories(stm32f072 INTERFACE ${libopencm3_SOURCE_DIR}/include)
set_property(TARGET stm32f072 PROPERTY IMPORTED_LOCATION ${libopencm3_SOURCE_DIR}/lib/libopencm3_stm32f0.a)
add_dependencies(stm32f072 libopencm3)

target_link_directories(stm32f072 INTERFACE ${libopencm3_SOURCE_DIR}/lib)

########################################################################
# MAIN APPLICATION
########################################################################

set(COMPILE_OPTIONS 
  --static
  -nostartfiles
  -fno-common
  -mcpu=cortex-m0
  -ffunction-sections 
  -fdata-sections
  -fno-rtti
  -ffreestanding
  -felide-constructors 
  -fno-exceptions
  -std=c++17
  -O2
)

set(LINKER_OPTIONS
    -Wl,--gc-sections
    -fno-rtti 
    -fno-exceptions
)

add_executable(nanovna 
    src/main.cpp
)

target_link_libraries(nanovna PUBLIC stm32f072)
target_compile_options(nanovna PUBLIC ${COMPILE_OPTIONS})
target_link_options(nanovna PUBLIC ${LINKER_OPTIONS})
target_compile_definitions(nanovna PUBLIC -DSTM32F0)

########################################################################
# Post processing
########################################################################

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${TOOLCHAIN_PATH}/bin/arm-none-eabi-size $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${TOOLCHAIN_PATH}/bin/arm-none-eabi-nm -S --demangle --size-sort -t d  $<TARGET_FILE:${CMAKE_PROJECT_NAME}> > $<TARGET_FILE:${CMAKE_PROJECT_NAME}>.nm
    COMMAND ${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy -O binary -R .eeprom $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE:${CMAKE_PROJECT_NAME}>.bin
    COMMAND ${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy -O srec -R .eeprom $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE:${CMAKE_PROJECT_NAME}>.srec
    COMMAND ${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy -O ihex -R .eeprom $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE:${CMAKE_PROJECT_NAME}>.hex
    COMMAND ${TOOLCHAIN_PATH}/bin/arm-none-eabi-objdump -x -d $<TARGET_FILE:${CMAKE_PROJECT_NAME}> >$<TARGET_FILE:${CMAKE_PROJECT_NAME}>.lst
)
