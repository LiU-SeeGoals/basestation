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
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

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

file(GLOB_RECURSE SOURCES "AZURE_RTOS/*.*" "Core/*.*" "Drivers/*.*" "Libraries/fw-shared-modules/Src/*.*" "Libraries/proto-messages/proto_c/*.*" "Libraries/protobuf/*.*" "Middlewares/*.*" "NetXDuo/*.*")
file(GLOB NRF_SOURCES "Libraries/nRF24L01P-lib/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H563ZITX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m33 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${NRF_SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

add_custom_target(flash DEPENDS ${BIN_FILE} ${PROJECT_NAME}.elf COMMAND STM32_Programmer_CLI -c port=SWD ap=1 -w ${BIN_FILE} 0x08000000 -rst)

