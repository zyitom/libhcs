# Host-side tests. They need no hardware: the wire protocol is exercised by
# playing both ends, the EP0 handshake against a simulated board.
#
# Built whenever libhcs is the top-level project (see ../CMakeLists.txt), so an
# API change that breaks one fails the ordinary build instead of going unnoticed.
# A project that embeds the SDK (HCS) compiles these same sources into its own
# test targets.

# Not REQUIRED, so a machine without GoogleTest can still build the SDK -- but
# loudly: the warning below is the only sign, which is why CI runs ctest with
# --no-tests=error and turns "no tests were built" into a failure.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    message(WARNING
        "GoogleTest not found: the libhcs host tests are NOT built. "
        "Install it (Debian/Ubuntu: libgtest-dev) or pass -DLIBHCS_BUILD_TESTS=OFF.")
    return()
endif()
include(GoogleTest)

function(libhcs_add_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
    # The repository root: tests reach the core's private headers (core/src/...).
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
    gtest_discover_tests(${name})
endfunction()

libhcs_add_test(wire_protocol_test wire_protocol_test.cpp)
libhcs_add_test(ep0_declaration_test ep0_declaration_test.cpp)
libhcs_add_test(sof_stamp_test sof_stamp_test.cpp)
libhcs_add_test(time_affine_test time_affine_test.cpp)
libhcs_add_test(sample_time_test sample_time_test.cpp)
libhcs_add_test(time_axis_test time_axis_test.cpp)
libhcs_add_test(board_ownership_test board_ownership_test.cpp)
libhcs_add_test(session_test session_test.cpp)
libhcs_add_test(sof_timebase_test sof_timebase_test.cpp)
