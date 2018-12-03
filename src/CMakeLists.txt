# Submodules
git_include(Eigen)
git_include(spline)

add_library(core STATIC
        ${CMAKE_SOURCE_DIR}/src/constants.hpp
        ${CMAKE_SOURCE_DIR}/src/conversion.hpp
        ${CMAKE_SOURCE_DIR}/src/coordinates.hpp
        ${CMAKE_SOURCE_DIR}/src/coordinates.cpp
        ${CMAKE_SOURCE_DIR}/src/map.hpp
        ${CMAKE_SOURCE_DIR}/src/map.cpp
        ${CMAKE_SOURCE_DIR}/src/prediction.hpp
        ${CMAKE_SOURCE_DIR}/src/prediction.cpp
        ${CMAKE_SOURCE_DIR}/src/trajectory.hpp
        ${CMAKE_SOURCE_DIR}/src/trajectory.cpp
        ${CMAKE_SOURCE_DIR}/src/trigonometry.hpp
        ${CMAKE_SOURCE_DIR}/src/vehicle.hpp
        ${CMAKE_SOURCE_DIR}/src/vehicle.cpp
        )

target_include_directories(core
        PUBLIC ${CMAKE_SOURCE_DIR}/src
        )

target_link_libraries(core
        PUBLIC Eigen
        PRIVATE spline
        )

if (DO_CLANG_TIDY)
    set_target_properties(core PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
endif ()