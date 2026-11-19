# RoboTest 装甲板角点解算（Windows x64）

`robottest_vision.exe` 从 RoboTest 接收灯条角点观测，并使用 OpenCV IPPE PnP 解算装甲板。收到的角点已经是归一化成像坐标，解算使用上游小装甲板模板，灯条间距为 135 mm，灯条长度为 56 mm。程序使用原框架的 Tracker 跟踪装甲板、由标准 Aimer 生成游戏内绝对云台目标角，并通过原 TCP 连接回传目标角和有效标记。原框架 Shooter 根据模拟炮口的实际朝向判断游戏内发射许可。连接和帧号可在控制台查看。

## 构建

Windows 模拟器目标需要 OpenCV 4 的 `core` 和 `calib3d`，以及 Eigen3、fmt、spdlog 和 yaml-cpp。仓库下 `build/robottest-deps/opencv` 中的 Windows x64 SDK 会被自动找到。若使用其他安装位置，令 `OpenCV_DIR` 指向包含 `OpenCVConfig.cmake` 的库目录，例如官方 SDK 的 `build/x64/vc16/lib`。

```powershell
cmake -S . -B build/robottest -G 'Visual Studio 18 2026' -A x64
cmake --build build/robottest --config Release --target robottest_vision
```

运行 `build/robottest/bin/Release/robottest_vision.exe`。同目录的 robottest_aim.yaml 提供仿真参数。运行时需能加载 OpenCV、fmt、spdlog 和 yaml-cpp 的 DLL，可将对应 DLL 放进 EXE 目录或将 SDK 的 bin 目录加入 PATH。

## 使用

先运行程序，再在 RoboTest 的比赛内设置开启 **外部自瞄程序**。默认监听 `127.0.0.1:5600`。使用 `--port PORT` 可指定其他端口，必须和 Unity 一致。HUD 显示解算数量、选中目标距离及连接状态。用 Ctrl+C 退出。协议说明见 RoboTest 仓库的 `Docs/SimulationCameraStream.md`。
