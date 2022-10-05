# change to out-of-source builds
cmake_minimum_required(VERSION 3.17)
set(PICO_SDK_FETCH_FROM_GIT on)


include(vendor/pico-examples/pico_sdk_import.cmake)

project(dehydrator)
pico_sdk_init() # this creates stuff in the top level. How to clear it out?
add_executable(dehydrator dehydrator.c)


# should only be set for GSL, probably via target_compile_options
add_definitions(-DHAVE_IEEE_COMPARISONS=1) # is this true?
set(GSL_DISABLE_TESTS ON)
set(NO_AMPL_BINDINGS ON)
add_subdirectory(vendor/gsl)
add_subdirectory(vendor/pico-examples)

add_library(ssd1306 INTERFACE)
target_sources(ssd1306 INTERFACE
        vendor/pico-ssd1306/ssd1306.c)
target_include_directories(ssd1306 INTERFACE vendor/pico-ssd1306)
target_link_libraries(ssd1306 INTERFACE hardware_i2c)

add_library(sht3x INTERFACE)
# add_definitions(-DUSE_SENSIRION_CLOCK_STRETCHING=1)
target_sources(sht3x INTERFACE
        sht3x_i2c.c
        vendor/embedded-sht/sht3x/sht3x.c
        vendor/embedded-sht/embedded-common/sensirion_common.c
        )

# ensure the embedded-common submodule was also checked out?
target_include_directories(sht3x INTERFACE
        vendor/embedded-sht/sht3x
        vendor/embedded-sht/embedded-common
        vendor/embedded-sht/sht-common)
target_link_libraries(sht3x INTERFACE hardware_i2c)


target_link_libraries(dehydrator
        gsl
        sht3x
        ssd1306
        nec_transmit_library
        pico_stdlib
        hardware_pio
        hardware_flash)
pico_add_extra_outputs(dehydrator)
pico_enable_stdio_usb(dehydrator 1)
