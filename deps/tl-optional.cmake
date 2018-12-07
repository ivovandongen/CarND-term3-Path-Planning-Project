include_guard_x()

add_library(tl-optional INTERFACE)
target_include_directories(tl-optional SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/deps/tl-optional)
