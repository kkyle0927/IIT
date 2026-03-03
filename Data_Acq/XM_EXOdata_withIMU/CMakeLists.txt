cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Extension_Module)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ===== User-defined source files =====

# Services - AGR_DOP (DOP + NMT)
set(AGR_DOP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_DOP/agr_dop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_DOP/agr_nmt.c
)

# Services - AGR_PNP
set(AGR_PNP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_PNP/agr_pnp.c
)

# Services - AGR_PnPMgr (Legacy - CM-XM용)
set(AGR_PNPMGR_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_PnPMgr/agr_pnpmgr.c
)

# Devices - AGR (Angel Robotics)
set(DEVICES_AGR_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Devices/AGR/Control_Module/cm_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Devices/AGR/IMU_Module/imu_hub_drv.c
)

# System - Links
set(SYSTEM_LINKS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/Control_Module/cm_xm_link.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/Control_Module/grf_xm_link.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/Control_Module/xsens_xm_link.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/IMU_Module/imu_hub_xm_link.c  # V1 (Deprecated)
    # ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/IMU_Module/imu_hub_xm_link_v2.c  # ✅ V3.0: Device Driver로 통합
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/PnP_Manager/pnp_manager.c
)

# System - Comm
set(SYSTEM_COMM_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Comm/CANFD/canfd_rx_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Comm/UART/uart_rx_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Comm/USB/cdc_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Comm/USB/data_logger.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Comm/USB/usb_mode_handler.c
)

# System - Core
set(SYSTEM_CORE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Core/system_startup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Core/core_process.c
)

# System - Debug
set(SYSTEM_DEBUG_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Debug/can_bus_monitor.c
)

# System - Board
set(SYSTEM_BOARD_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Board/Button/button_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Board/LED/led_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Board/GPIO/external_io.c
)

# Application Layer
set(APP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Apps/user_app.c
)

# IOIF Layer
set(IOIF_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_fs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Src/ioif_agrb_dwt.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Services
    ${AGR_DOP_SRC}
    ${AGR_PNP_SRC}
    ${AGR_PNPMGR_SRC}
    
    # Devices
    ${DEVICES_AGR_SRC}
    ${DEVICES_MARVELDEX_SRC}
    ${DEVICES_XSENS_SRC}
    
    # System - Links
    ${SYSTEM_LINKS_SRC}
    
    # System - Comm
    ${SYSTEM_COMM_SRC}
    
    # System - Core
    ${SYSTEM_CORE_SRC}
    
    # System - Debug
    ${SYSTEM_DEBUG_SRC}
    
    # System - Board
    ${SYSTEM_BOARD_SRC}
    
    # Application Layer
    ${APP_SRC}
    
    # IOIF Layer
    ${IOIF_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User-defined include paths
    # Services
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_DOP
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_PNP
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Services/AGR_PnPMgr
    
    # Devices
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Devices/AGR/Control_Module
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Devices/AGR/IMU_Module
    
    # System - Links
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/Control_Module
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/IMU_Module
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Links/PnP_Manager
    
    # System - Other
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Comm/CANFD
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Core
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Debug
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Board
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/System/Config
    
    # Application & IOIF
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/Apps
    ${CMAKE_CURRENT_SOURCE_DIR}/XM_FW/IOIF/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)