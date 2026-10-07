cmake_minimum_required(VERSION 3.21...3.28)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

project(libhcs-sdk VERSION 3 LANGUAGES C CXX)

if(NOT DEFINED libhcs_PROJECT_VERSION OR libhcs_PROJECT_VERSION STREQUAL "")
    execute_process(
        COMMAND ".scripts/generate_version"
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
        OUTPUT_VARIABLE libhcs_PROJECT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        COMMAND_ERROR_IS_FATAL ANY
    )
endif()
message(STATUS "libhcs project version: ${libhcs_PROJECT_VERSION}")

# Set C++ standard to C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set C standard to C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Disable GNU extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add compiler options based on compiler
if(MSVC)
    add_compile_options(/W4 /Zc:preprocessor)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
    # GCC/Clang
    add_compile_options(
        -g -Wall -Wextra -Wpedantic -fvisibility=hidden
        $<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>
    )
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

# Get project sources
file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

option(BUILD_STATIC_libhcs "Build static libhcs" ON)
if(BUILD_STATIC_libhcs)
    add_library(
        ${PROJECT_NAME} STATIC
        ${PROJECT_SOURCE}
    )
    target_compile_definitions(${PROJECT_NAME} PUBLIC STATIC_LINKING_libhcs)
    message(STATUS "Building static libhcs")
else()
    add_library(
        ${PROJECT_NAME} SHARED
        ${PROJECT_SOURCE}
    )
    message(STATUS "Building shared libhcs")
endif()

# How a project uses the SDK: FetchContent the release's libhcs-sdk-src-<version>.zip and link
# libhcs::sdk (README.md). The namespaced name fails at configure time when misspelled, where a
# plain target name would silently become a "-l" flag.
add_library(libhcs::sdk ALIAS ${PROJECT_NAME})
# The public headers need C++23 (concepts, requires-expressions); linking the target is enough
# for a consumer to get it. Without this a project on the compiler's default standard compiles
# the headers wrong instead of failing.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_compile_definitions(${PROJECT_NAME} PRIVATE libhcs_PROJECT_VERSION_STRING="${libhcs_PROJECT_VERSION}")
target_compile_definitions(${PROJECT_NAME} PRIVATE BUILDING_libhcs)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../core/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../host/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Optimize the SDK even in Debug builds -- deliberate, kept from upstream
# librmcs. An application built as Debug still runs the IO thread (record
# decoding, timestamp conversion, the transmit path) at release speed, so a
# Debug build of HCS behaves on the bench like the real thing and its timing
# tests stay meaningful. To step through the SDK itself, drop these lines in a
# local build.
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:/O2>)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:-O3>)
endif()

if(WIN32)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME} PUBLIC "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUSB_LIBRARIES})
elseif(UNIX)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUSB_LIBRARIES})
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
endif()

# LTO is the consumer's call, not the library's: a static archive of LTO objects only links when
# the final link does LTO too (clang's driver loads its LTO plugin only under -flto, so a plain
# link with the default bfd linker fails). Off by default, so the released source builds into
# ordinary objects any toolchain links; a project that links with LTO turns it on (HCS does).
option(LIBHCS_LTO "Build the SDK with link-time optimization; the final link must use LTO too" OFF)
if(LIBHCS_LTO)
    set_target_properties(${PROJECT_NAME} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# include(CTest)
# if(BUILD_TESTING)
#     include(FetchContent)
#     set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#     FetchContent_Declare(
#         googletest
#         URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
#         DOWNLOAD_EXTRACT_TIMESTAMP TRUE
#     )
#     FetchContent_MakeAvailable(googletest)

#     file(GLOB_RECURSE libhcs_TEST_SOURCES CONFIGURE_DEPENDS
#         ${PROJECT_SOURCE_DIR}/tests/*.cpp
#     )

#     add_executable(libhcs_tests
#         ${libhcs_TEST_SOURCES}
#     )
#     target_link_libraries(libhcs_tests PRIVATE gtest_main ${PROJECT_NAME})

#     add_test(NAME libhcs_tests COMMAND libhcs_tests)
# endif()

# Tests are part of the default build when this is the top-level project: a test
# nobody compiles is a test nobody notices breaking. Embedded in another project
# (add_subdirectory from HCS) they stay off here -- that project builds the same
# sources into its own test run.
option(LIBHCS_BUILD_TESTS "Build and register the host-side tests" ${PROJECT_IS_TOP_LEVEL})
if(LIBHCS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
