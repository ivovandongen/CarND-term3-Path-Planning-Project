# Submodules
git_include(fmt)
git_include(Eigen)
git_include(tl-optional)
git_include(spline)

add_library(core STATIC
        ${CMAKE_SOURCE_DIR}/src/behaviour.hpp
        ${CMAKE_SOURCE_DIR}/src/behaviour.cpp
        ${CMAKE_SOURCE_DIR}/src/constants.hpp
        ${CMAKE_SOURCE_DIR}/src/coordinates.hpp
        ${CMAKE_SOURCE_DIR}/src/coordinates.cpp
        ${CMAKE_SOURCE_DIR}/src/cost.hpp
        ${CMAKE_SOURCE_DIR}/src/cost.cpp
        ${CMAKE_SOURCE_DIR}/src/map.hpp
        ${CMAKE_SOURCE_DIR}/src/map.cpp
        ${CMAKE_SOURCE_DIR}/src/prediction.hpp
        ${CMAKE_SOURCE_DIR}/src/prediction.cpp
        ${CMAKE_SOURCE_DIR}/src/trajectory.hpp
        ${CMAKE_SOURCE_DIR}/src/trajectory.cpp
        ${CMAKE_SOURCE_DIR}/src/trigonometry.hpp
        ${CMAKE_SOURCE_DIR}/src/vehicle.hpp
        ${CMAKE_SOURCE_DIR}/src/vehicle.cpp
        ${CMAKE_SOURCE_DIR}/src/fmt/behaviour.hpp
        ${CMAKE_SOURCE_DIR}/src/fmt/collections.hpp
        ${CMAKE_SOURCE_DIR}/src/fmt/coordinates.hpp
        ${CMAKE_SOURCE_DIR}/src/fmt/prediction.hpp
        ${CMAKE_SOURCE_DIR}/src/util/collections.hpp
        ${CMAKE_SOURCE_DIR}/src/util/conversion.hpp
        ${CMAKE_SOURCE_DIR}/src/util/time.hpp
        )

target_include_directories(core
        PUBLIC ${CMAKE_SOURCE_DIR}/src
        )

target_link_libraries(core
        PUBLIC Eigen
        PUBLIC fmt
        PUBLIC tl-optional
        PRIVATE spline
        )

if (DO_CLANG_TIDY)
    set_target_properties(core PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
endif ()