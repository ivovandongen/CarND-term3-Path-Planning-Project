cmake_minimum_required(VERSION 3.5)
project(PATH_PLANNER)

set(CMAKE_CXX_STANDARD 14)

# Update submodules as needed
option(GIT_SUBMODULE "Check submodules during build" ON)
option(ENABLE_LINTERS "Enalbe linters to check code during build" ON)
option(ENABLE_TESTS "Enable tests" ON)

# Hacky stuff from original cmake file
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Load modules
include(${CMAKE_MODULE_PATH}/git.cmake)
include(${CMAKE_MODULE_PATH}/include_guard.cmake)

if (USE_LINTERS)
    # Clang format

    file(GLOB_RECURSE SRC_FILES src/*.cpp src/*.hpp)
    file(GLOB_RECURSE TEST_FILES src/*.cpp src/*.hpp)
    set(ALL_SOURCES ${SRC_FILES} ${TEST_FILES})

    find_program(CLANG_FORMAT NAMES "clang-format" DOC "Path to clang-format executable")
    if (NOT CLANG_FORMAT)
        message(STATUS "clang-format not found.")
    else ()
        message(STATUS "clang-format found: ${CLANG_FORMAT}")
        add_custom_target(
                clang-format
                COMMAND ${CLANG_FORMAT}
                -style=file
                -i
                ${ALL_SOURCES}
        )
    endif ()

    # Clang tidy

    find_program(CLANG_TIDY NAMES "clang-tidy" DOC "Path to clang-tidy executable")
    if (NOT CLANG_TIDY)
        message(STATUS "clang-tidy not found.")
    else ()
        message(STATUS "clang-tidy found: ${CLANG_TIDY}")
        set(DO_CLANG_TIDY "${CLANG_TIDY}" "-fix;-fix-errors")
    endif ()

endif()

# Core sources

add_subdirectory(src)

# Main executable

add_executable(PathPlanner
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    )

git_include(uWebSockets)
git_include(json)

target_link_libraries(PathPlanner libuWS json core)


# Tests

if (ENABLE_TESTS)
    add_subdirectory(test)
endif()
