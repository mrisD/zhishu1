# 第二排局部导纳验证

用户要求切换第二排验证。第三排 `third_row_diagnostic5` 的日志正式完成 `[6]`，在第2箱侧拉期间主动结束；另外两箱未验证完成，不能记为第三排全部通过。切换说明保存在该轮 `interruption_note.json`。

新运行 `output/pallet_full_process_admittance_20261018/second_row_diagnostic1` 仅保留第二排三个物理箱，初始移除第一、第三排。执行顺序 `[5,4,3]`，即 `pallet_box_r1_c2`、`pallet_box_r1_c1`、`pallet_box_r1_c0`；继续原有后方推出、双臂接近、抓取抬升、载物搬运、放置、支撑后松爪、退手/收臂与前两箱返位流程。末箱无下一取箱工位，不执行空载返回。

复用当前全流程导纳控制、回环通信配置与无数据停更监测，保留力/力矩、碰撞与真实抬升/放置验收。仅局部验收可记为 `diagnostic_passed`，`passed` 和 `process_admittance_passed` 均不得作为九箱通过。此模式禁止录制及通过后自动录制。

启动入口：

```bash
bash scripts/run_with_gz_control_safe_overlay.sh python3 scripts/run_pallet_nine_continuous.py \
  --output-dir output/pallet_full_process_admittance_20261018/second_row_diagnostic1 \
  --second-row-diagnostic --box-limit 3 --no-record --process-admittance
```

脚本语法、诊断互斥、三箱场景、初始位姿、结果门控和录像限制已通过静态检查。实时结果以本轮 `sequence_result.json`、日志和逐箱导纳审计为准，不复用旧箱子完成数。

用户进一步要求先完成第二排全部三箱，再验证第三排全部三箱。已启动 `scripts/continue_pallet_row_validation.py`，等待 second_row_diagnostic1 的实际完成顺序精确为 `[5,4,3]`、三份逐箱导纳审计通过且 `diagnostic_passed=true` 后，才自动启动全新的 `third_row_diagnostic6`，执行 `[6,7,8]` 三箱并切换 GUI。结果不全或有失败则拒绝启动下一排，不跳箱、不拼接旧结果、不录像。顺序门控的成功、缺箱、错序、审计失败及运行失败拒绝检查通过。

监督日志为 `output/pallet_full_process_admittance_20261018/second_to_third_sequence.log`。第三排的新结果仍仅代表本轮局部验证；两排独立通过不等于从九箱初始场景连续完成九箱。
