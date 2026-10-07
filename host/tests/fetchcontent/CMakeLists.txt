# A project outside this tree using the SDK the documented way: FetchContent the released
# source zip and link libhcs::sdk (README.md). CI packs the zip with
# .scripts/package_sdk_source and builds this against it; a broken zip, a missing file in it or
# a renamed target fails here, before a release ships it.
cmake_minimum_required(VERSION 3.24)
project(libhcs_fetchcontent_consumer LANGUAGES CXX)

set(LIBHCS_SDK_ZIP "" CACHE FILEPATH "libhcs-sdk-src-<version>.zip to build against")

include(FetchContent)
FetchContent_Declare(libhcs URL "${LIBHCS_SDK_ZIP}" DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(libhcs)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE libhcs::sdk)
