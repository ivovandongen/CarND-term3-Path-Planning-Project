include_guard_x()

set(FMT_DIR ${CMAKE_SOURCE_DIR}/deps/fmt)

add_library(fmt STATIC
   ${FMT_DIR}/src/format.cc
   ${FMT_DIR}/src/posix.cc
)

target_include_directories(fmt
    SYSTEM PRIVATE ${FMT_DIR}/include
)

target_include_directories(fmt
    SYSTEM INTERFACE ${FMT_DIR}/include
)