# RoboTest 模拟相机接收器（Windows x64）

`robottest_vision.exe` 监听本机 TCP，从 RoboTest 接收 RBTF v1 BGR8 帧。接收线程持续读流，检测线程只取最新的完整帧。Unity 切换机器人或重新连接时，接收器会再次接受连接。

## 构建

在已初始化 MSVC 的终端中使用 Visual Studio 随附的 CMake。下面以 Visual Studio 2026 为例。Windows 默认启用 `SP_VISION_ROBOTTEST`。完整构建需要 OpenCV（含 DNN、HighGUI、VideoIO）、OpenVINO Runtime、Eigen3、fmt、spdlog 和 yaml-cpp。

已经将 SDK 和 vcpkg 依赖放到仓库下的 build/robottest-deps 时，Visual Studio 的 x64 配置会自动搜索该目录。依赖放在其他位置时，可以用下面的显式配置。

使用独立 vcpkg 可安装四个通用依赖。将 OpenCV_DIR 指向 OpenCV SDK 的 build/x64/vc16/lib，将 OpenVINO_DIR 指向 OpenVINO SDK 的 runtime/cmake。OpenVINO 建议使用与上游一致的 2024.6。

```powershell
& "$env:VCPKG_ROOT/vcpkg.exe" install eigen3:x64-windows fmt:x64-windows spdlog:x64-windows yaml-cpp:x64-windows
cmake -S . -B build/robottest -G 'Visual Studio 18 2026' -A x64 "-DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" "-DOpenCV_DIR=$env:OpenCV_DIR" "-DOpenVINO_DIR=$env:OpenVINO_DIR"
cmake --build build/robottest --config Release --target robottest_vision
```

产物在 `build/robottest/bin/Release`，构建会复制 `vision.yaml` 和模型。运行时需能加载所用 SDK 的 DLL。可先在设置了 OpenVINO Runtime 环境并将 OpenCV bin 目录加入 PATH 的终端运行。分发时将所需 Release DLL、OpenVINO 的 CPU 插件及模型随 EXE 一同部署。

只验证 TCP 接收时，可构建不含视觉依赖的版本。

```powershell
cmake -S . -B build/robottest-receiver -G 'Visual Studio 18 2026' -A x64 -DSP_VISION_SIM_DETECTOR=OFF
cmake --build build/robottest-receiver --config Release --target robottest_vision
```

## 使用

先运行 EXE，再在 RoboTest 的 PlayerContext 开启 **Stream Simulation Camera**。默认监听 `127.0.0.1:5600`。检测版默认使用 CPU 推理，窗口显示接收到的画面和检测角点，控制台输出帧号、采集时间和检测结果。

```powershell
./build/robottest/bin/Release/robottest_vision.exe
./build/robottest/bin/Release/robottest_vision.exe --port 5600 --receive-only
./build/robottest/bin/Release/robottest_vision.exe --video assets/demo/demo.avi --no-preview
```

`--port` 须与 Unity 的设置相同。`--video` 支持离线视频文件，用实际文件路径替换示例。`--config` 可指定其他 YAML，YAML 中的相对模型路径以 EXE 目录为基准。`--no-preview` 适合只收集日志，`--max-frames N` 适合自动化检查。用 Ctrl+C 退出，有预览窗口时也可按 Esc。
