set(robottest_opencv_hint "")
if(MSVC AND CMAKE_CXX_COMPILER_ARCHITECTURE_ID STREQUAL "x64")
    set(robottest_opencv_hint "${PROJECT_SOURCE_DIR}/build/robottest-deps/opencv/build/x64/vc16/lib")
    list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/build/robottest-deps/vcpkg-master/installed/x64-windows")
endif()
find_package(Eigen3 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d HINTS ${robottest_opencv_hint})

add_executable(robottest_vision main.cpp camera_receiver.cpp
    ../tasks/auto_aim/armor.cpp
    ../tasks/auto_aim/solver.cpp
    ../tasks/auto_aim/tracker.cpp
    ../tasks/auto_aim/target.cpp
    ../tasks/auto_aim/aimer.cpp
    ../tasks/auto_aim/shooter.cpp
    ../tools/extended_kalman_filter.cpp
    ../tools/logger.cpp
    ../tools/math_tools.cpp
    ../tools/trajectory.cpp
)
add_custom_target(robottest_aim_config
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_BINARY_DIR}/bin/$<CONFIG>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/robottest_aim.yaml"
        "${PROJECT_BINARY_DIR}/bin/$<CONFIG>/robottest_aim.yaml"
    VERBATIM
)
add_dependencies(robottest_vision robottest_aim_config)
if(MSVC)
    target_compile_options(robottest_vision PRIVATE /utf-8)
endif()
target_compile_features(robottest_vision PRIVATE cxx_std_17)
set_target_properties(robottest_vision PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/$<CONFIG>")
target_compile_definitions(robottest_vision PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_include_directories(robottest_vision PRIVATE "${PROJECT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(robottest_vision PRIVATE ws2_32 ${OpenCV_LIBS}
    Eigen3::Eigen fmt::fmt spdlog::spdlog yaml-cpp::yaml-cpp)
