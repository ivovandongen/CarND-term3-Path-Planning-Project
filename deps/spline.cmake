include_guard_x()

add_library(spline INTERFACE)
target_include_directories(spline SYSTEM INTERFACE ${CMAKE_SOURCE_DIR}/deps/spline/src)
