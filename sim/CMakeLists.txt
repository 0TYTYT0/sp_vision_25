option(SP_VISION_SIM_DETECTOR "Build the RoboTest receiver with YOLOv5 inference" ON)

add_executable(robottest_vision main.cpp camera_receiver.cpp)
target_compile_features(robottest_vision PRIVATE cxx_std_17)
set(robottest_output_dir "${PROJECT_BINARY_DIR}/bin/$<CONFIG>")
set_target_properties(robottest_vision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${robottest_output_dir}")
target_compile_definitions(robottest_vision PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(robottest_vision PRIVATE ws2_32)

if(SP_VISION_SIM_DETECTOR)
    if(MSVC AND CMAKE_CXX_COMPILER_ARCHITECTURE_ID STREQUAL "x64")
        set(robottest_dependencies "${PROJECT_SOURCE_DIR}/build/robottest-deps")
        set(robottest_opencv_hint "${robottest_dependencies}/opencv/build/x64/vc16/lib")
        file(GLOB robottest_openvino_hints LIST_DIRECTORIES true
            "${robottest_dependencies}/w_openvino_toolkit_windows_*_x86_64/runtime/cmake")
        list(APPEND CMAKE_PREFIX_PATH "${robottest_dependencies}/vcpkg-master/installed/x64-windows")
    endif()

    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio dnn
        HINTS ${robottest_opencv_hint})
    find_package(OpenVINO REQUIRED COMPONENTS Runtime HINTS ${robottest_openvino_hints})
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(fmt CONFIG REQUIRED)
    find_package(spdlog CONFIG REQUIRED)
    find_package(yaml-cpp CONFIG REQUIRED)

    target_sources(robottest_vision PRIVATE
        ../tasks/auto_aim/armor.cpp
        ../tasks/auto_aim/classifier.cpp
        ../tasks/auto_aim/detector.cpp
        ../tasks/auto_aim/yolos/yolov5.cpp
        ../tools/img_tools.cpp
        ../tools/logger.cpp
    )
    target_include_directories(robottest_vision PRIVATE "${PROJECT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
    target_compile_definitions(robottest_vision PRIVATE SP_VISION_SIM_DETECTOR)
    target_link_libraries(robottest_vision PRIVATE
        ${OpenCV_LIBS} openvino::runtime Eigen3::Eigen fmt::fmt spdlog::spdlog yaml-cpp::yaml-cpp
    )
    if(MSVC)
        target_compile_options(robottest_vision PRIVATE /utf-8)
    endif()

    add_custom_target(robottest_vision_assets
        COMMAND ${CMAKE_COMMAND} -E make_directory "${robottest_output_dir}/assets"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/vision.yaml" "${robottest_output_dir}/vision.yaml"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PROJECT_SOURCE_DIR}/assets/yolov5.xml"
            "${PROJECT_SOURCE_DIR}/assets/yolov5.bin"
            "${PROJECT_SOURCE_DIR}/assets/tiny_resnet.onnx"
            "${robottest_output_dir}/assets"
        VERBATIM
    )
    add_dependencies(robottest_vision robottest_vision_assets)
endif()
