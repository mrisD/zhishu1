# 恢复 nine7 核心程序

目标是 `output/pallet_full_process_admittance_20261018/nine7`，该轮连续完成 `[0,1,2,6,7,8,5,4]`，最后一箱 `pallet_box_r1_c0` 在大箱工位对接失败。不是完整九箱通过。

依据该目录 `verification_profile.json`，恢复以下5个核心文件，逐项SHA-256与原记录完全一致：

- scripts/run_pallet_nine_continuous.py
- scripts/audit_pallet_process_admittance.py
- src/zhishu_robot_mtc_pick_place/src/pallet_bin_15kg_transfer.cpp
- src/zhishu_robot_mtc_pick_place/include/zhishu_robot_mtc_pick_place/process_admittance.hpp
- src/zhishu_robot_mtc_pick_place/include/zhishu_robot_mtc_pick_place/cartesian_microstep.hpp

其中4个文件从Git对象恢复；导纳头文件从本线程2026-10-18的原始写入及补丁记录重建，并以原始哈希校验，不按现有逻辑猜测重写。对应6项导纳单测也从同期历史记录恢复。

撤销本次新增的16轴组合SRDF组及其专用配置测试；其他不相关工作不重置。后续新增录像/监控工具保留在磁盘，但不是nine7原始运行文件，也不由恢复版runner自动启动。

恢复前备份见 `output/nine7_restoration_report.json` 中backup路径，归档包含当前控制包、MoveIt配置、脚本与恢复前二进制。原始nine7二进制未找回，因此需要从精确匹配的源码重新编译；不能声称重编译的可执行文件与旧二进制逐字节相同。

本次恢复不自动启动仿真、搬运或录像，已取消旧轮五分钟提醒。仍需在恢复版上重新验证才可产生新通过结论。
