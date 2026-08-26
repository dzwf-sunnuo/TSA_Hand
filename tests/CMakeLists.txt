cmake_minimum_required(VERSION 3.22)

# 使用宿主机编译器运行不依赖STM32硬件的协议单元测试
project(FR_Hand_s_ts_HostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(test_angle_codec
    test_angle_codec.c
    ../Core/Src/angle_codec.c
)

target_include_directories(test_angle_codec PRIVATE
    ../Core/Inc
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_angle_codec PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME angle_codec COMMAND test_angle_codec)
