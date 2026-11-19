set(robottest_opencv_hint "")
if(MSVC AND CMAKE_CXX_COMPILER_ARCHITECTURE_ID STREQUAL "x64")
    set(robottest_opencv_hint "${PROJECT_SOURCE_DIR}/build/robottest-deps/opencv/build/x64/vc16/lib")
endif()
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d HINTS ${robottest_opencv_hint})

add_executable(robottest_vision main.cpp camera_receiver.cpp)
target_compile_features(robottest_vision PRIVATE cxx_std_17)
set_target_properties(robottest_vision PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/$<CONFIG>")
target_compile_definitions(robottest_vision PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_include_directories(robottest_vision PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(robottest_vision PRIVATE ws2_32 ${OpenCV_LIBS})
