#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.26)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(basestation C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m33 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc Drivers/STM32H5xx_HAL_Driver/Inc Drivers/STM32H5xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32H5xx/Include Drivers/CMSIS/Include NetXDuo/App AZURE_RTOS/App Middlewares/ST/netxduo/addons/dhcp Middlewares/ST/threadx/common/inc Middlewares/ST/netxduo/common/inc Middlewares/ST/netxduo/ports/cortex_m33/gnu/inc Middlewares/ST/threadx/ports/cortex_m33/gnu/inc NetXDuo/Target Middlewares/ST/netxduo/common/drivers/ethernet Drivers/BSP/Components/lan8742 Libraries/fw-shared-modules/Inc Libraries/nRF24L01P-lib Libraries/proto-messages/proto_c Libraries/protobuf Core/Inc Drivers/STM32H5xx_HAL_Driver/Inc Drivers/STM32H5xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32H5xx/Include Drivers/CMSIS/Include NetXDuo/App AZURE_RTOS/App Middlewares/ST/netxduo/addons/dhcp Middlewares/ST/threadx/common/inc Middlewares/ST/netxduo/common/inc Middlewares/ST/netxduo/ports/cortex_m33/gnu/inc Middlewares/ST/threadx/ports/cortex_m33/gnu/inc NetXDuo/Target Middlewares/ST/netxduo/common/drivers/ethernet Drivers/BSP/Components/lan8742 Libraries/fw-shared-modules/Inc Libraries/nRF24L01P-lib Libraries/proto-messages/proto_c Libraries/protobuf)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H563xx -DNX_INCLUDE_USER_DEFINE_FILE -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1)

set(CORE_SRC
    Core/Src/app_threadx.c
    Core/Src/com.c
    Core/Src/handle_packet.c
    Core/Src/main.c
    Core/Src/stm32h5xx_hal_msp.c
    Core/Src/stm32h5xx_hal_timebase_tim.c
    Core/Src/stm32h5xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32h5xx.c
    Core/Src/tx_initialize_low_level.S
    Core/Startup/startup_stm32h563zitx.s
)
set(CORE_INC
    Core/Inc/app_threadx.h
    Core/Inc/com.h
    Core/Inc/handle_packet.h
    Core/Inc/main.h
    Core/Inc/stm32h5xx_hal_conf.h
    Core/Inc/stm32h5xx_it.h
    Core/Inc/tx_user.h
)
set(SHARED_MODULES_SRC
    Libraries/fw-shared-modules/Src/log.c)
set(SHARED_MODULES_INC
    Libraries/fw-shared-modules/Inc/log.h
    Libraries/fw-shared-modules/Inc/ringbuffer.h
)
set(PROTO_SRC
    Libraries/protobuf/protobuf-c/protobuf-c.c
    Libraries/proto-messages/proto_c/parsed_vision/parsed_vision.pb-c.c
    Libraries/proto-messages/proto_c/robot_action/robot_action.pb-c.c
)
set(PROTO_INC
    Libraries/protobuf/protobuf-c/protobuf-c.h
    Libraries/proto-messages/proto_c/parsed_vision/parsed_vision.pb-c.h
    Libraries/proto-messages/proto_c/robot_action/robot_action.pb-c.h
)
set(NRF_SRC 
    Libraries/nRF24L01P-lib/nrf24l01.c)
set(NRF_INC
    Libraries/nRF24L01P-lib/nrf24l01.h
    Libraries/nRF24L01P-lib/nrf_helper_defines.h)

file(GLOB_RECURSE STM32_SOURCES "Drivers/*.*" "Middlewares/*.*" "NetXDuo/*.*" "AZURE_RTOS/*.*")
file(GLOB NRF_SOURCES "Libraries/nRF24L01P-lib/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H563ZITX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m33 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${NRF_SOURCES} ${LINKER_SCRIPT} 
               ${CORE_SRC} ${CORE_INC}
               ${SHARED_MODULES_SRC} ${SHARED_MODULES_INC}
               ${PROTO_SRC} ${PROTO_INC}
               ${NRF_SRC} ${NRF_INC}
               ${STM32_SOURCES}
               ${DRIVER_SRC} ${DRIVER_INC})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

add_custom_target(flash DEPENDS ${BIN_FILE} ${PROJECT_NAME}.elf COMMAND STM32_Programmer_CLI -c port=SWD sn=004C00283232511639353236 ap=1 -w ${BIN_FILE} 0x08000000 -rst)

