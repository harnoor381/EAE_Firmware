cmake_minimum_required(VERSION 3.20)
project(EAE_Firmware VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(EAE_ENABLE_SANITIZERS "Check memory access and undefined behavior" OFF)

add_library(eae_cooling
    src/pid.cpp
    src/controller.cpp
    src/can.cpp
    src/simulation.cpp
    src/options.cpp
)
target_include_directories(eae_cooling PUBLIC include)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(eae_cooling PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror)
    if(EAE_ENABLE_SANITIZERS)
        target_compile_options(eae_cooling PUBLIC
            -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(eae_cooling PUBLIC -fsanitize=address,undefined)
    endif()
endif()

add_executable(eae_firmware src/main.cpp)
target_link_libraries(eae_firmware PRIVATE eae_cooling)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(eae_firmware PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
    # Download GoogleTest when configuring the build. Do not commit its source.
    include(FetchContent)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
        URL https://codeload.github.com/google/googletest/tar.gz/refs/tags/v1.17.0
        URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    )
    FetchContent_MakeAvailable(googletest)

    add_executable(eae_tests
        tests/test_controller.cpp
        tests/test_pid.cpp
        tests/test_can.cpp
        tests/test_simulation.cpp
        tests/test_options.cpp
    )
    target_link_libraries(eae_tests PRIVATE eae_cooling GTest::gtest_main)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(eae_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    include(GoogleTest)
    gtest_discover_tests(eae_tests)
endif()
