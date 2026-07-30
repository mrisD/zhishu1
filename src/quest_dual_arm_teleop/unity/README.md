# Quest Unity 端

1. 新建 Android/Quest Unity 工程。
2. 在 Package Manager 安装 `XR Plug-in Management` 和 `OpenXR Plugin`。
3. `Edit -> Project Settings -> XR Plug-in Management -> Android`：启用 OpenXR。
4. 在 OpenXR 功能中启用 `Meta Quest Support`，Interaction Profiles 添加 `Oculus Touch Controller Profile`。
5. 场景中创建空 GameObject，挂载 `QuestControllerTcpSender.cs`。
6. `Player Settings -> Android`：目标架构选 ARM64，Internet Access 设为 Require。
7. 构建 APK 并安装：`adb install -r YourQuestTeleop.apk`。
8. USB 调试模式执行：`adb reverse tcp:5005 tcp:5005`，脚本 Host 保持 `127.0.0.1`。
9. 启动机器人端后，在 Quest 中启动应用。
10. 持续按住左手柄侧握键控制左臂，持续按住右手柄侧握键控制右臂；松开即停止对应机械臂。
