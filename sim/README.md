# RoboTest 装甲板角点解算（Windows x64）

`robottest_vision.exe` 从 RoboTest 接收灯条角点观测，并使用 OpenCV IPPE PnP 解算装甲板。收到的角点已经是归一化成像坐标，解算使用上游小装甲板模板，灯条间距为 135 mm，灯条长度为 56 mm。程序输出解算数量、选中目标的距离、Unity 世界位置和相机坐标下的旋转向量，并通过原 TCP 连接回传。连接和帧号可在控制台查看。

## 构建

Windows 模拟器目标需要 OpenCV 4 的 `core` 和 `calib3d`。仓库下 `build/robottest-deps/opencv` 中的 Windows x64 SDK 会被自动找到。若使用其他安装位置，令 `OpenCV_DIR` 指向包含 `OpenCVConfig.cmake` 的库目录，例如官方 SDK 的 `build/x64/vc16/lib`。

```powershell
cmake -S . -B build/robottest -G 'Visual Studio 18 2026' -A x64
cmake --build build/robottest --config Release --target robottest_vision
```

运行 `build/robottest/bin/Release/robottest_vision.exe`。运行时需能加载 OpenCV DLL，可将所用 Release DLL 放到 EXE 同一目录或把 OpenCV bin 目录加入 PATH。

## 使用

先运行程序，再在 RoboTest 的比赛内设置开启 **外部自瞄程序**。默认监听 `127.0.0.1:5600`。使用 `--port PORT` 可指定其他端口，必须和 Unity 一致。HUD 显示解算数量、选中目标距离及连接状态。用 Ctrl+C 退出。协议说明见 RoboTest 仓库的 `Docs/SimulationCameraStream.md`。
