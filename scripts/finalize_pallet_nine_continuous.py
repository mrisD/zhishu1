#!/usr/bin/env python3
"""Gate and export an uninterrupted nine-box recording without editing its frames."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ORDER = [0, 1, 2, 6, 7, 8, 5, 4, 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--av-path', type=Path, default=Path('/tmp/avpkg'))
    args = parser.parse_args()
    source, out = args.source.resolve(), args.destination.resolve()
    sequence = json.loads((source / 'sequence_result.json').read_text())
    recording = json.loads((source / 'recording_summary.json').read_text())
    if not sequence['passed'] or sequence['completed'] != ORDER or sequence['order'] != ORDER:
        raise RuntimeError('Nine transfers did not complete in the requested order')
    if not sequence['single_continuous_scene'] or sequence['scene_restarts'] != 0 or sequence['recording_restarts'] != 0 or sequence['resumed'] or sequence['stitched']:
        raise RuntimeError('Recording is not a single uninterrupted execution')
    if not recording['passed'] or recording['initial_completed_transfers'] != 0 or recording['recorded_completed_transfers'] != 9 or recording['clean_video']:
        raise RuntimeError('Recording lacks nine newly completed transfers or live telemetry overlay')
    if (recording['video_width'], recording['video_height']) != (960,650):
        raise RuntimeError('Recording dimensions do not contain the requested telemetry overlay')
    if recording['fps'] != 30 or recording.get('sampling_time') != 'simulation':
        raise RuntimeError('Expected 30 fps output sampled by simulation timestamps')
    scene_log = (source / 'scene.log').read_text()
    recorder_log = (source / 'recorder.log').read_text()
    if len(re.findall(r'\[gazebo-\d+\]: process started with pid', scene_log)) != 1:
        raise RuntimeError('Scene log does not show exactly one Gazebo process start')
    if recorder_log.count('Recording actual rendered images; both video encoders ready') != 1:
        raise RuntimeError('Recorder log does not show exactly one recording start')
    world = ET.parse(source / 'sequence_source.sdf').getroot().find('world')
    initial = sorted(m.attrib['name'] for m in world.findall('model') if m.attrib['name'].startswith('pallet_box_'))
    expected = sorted(f'pallet_box_r{i//3}_c{i%3}' for i in range(9))
    if initial != expected:
        raise RuntimeError('Initial world does not contain all nine physical boxes')
    boxes = []
    for cycle, index in enumerate(ORDER):
        log = (source / f'box_{index}.log').read_text()
        for required in ('Complete transfer finished', 'Floor seating step 1:',
                         'Floor support verified BEFORE opening using wrist F/T estimate',
                         'Physical large-bin placement verified', 'return both arms home: execution completed'):
            if required not in log:
                raise RuntimeError(f'Box {index} lacks {required}')
        if '[FATAL]' in log or 'TOOL_BODY_GUARD stopped execution' in log:
            raise RuntimeError(f'Box {index} has an execution failure')
        returned = 'Empty robot returned to pallet; destination clearance now permitted' in log
        if cycle < 8 and not returned:
            raise RuntimeError(f'Box {index} lacks confirmed empty return')
        clearance = [float(value) for value in re.findall(r'TOOL_BODY_GUARD [^\n]*samples=\d+ minimum_clearance=([\d.]+) m', log)]
        if not clearance or min(clearance) < 0.003 - 1e-6:
            raise RuntimeError(f'Box {index} lacks accepted body-clearance evidence')
        floor = re.search(r'Floor support verified BEFORE opening: z=([\d.]+) tilt=([\d.]+) deg reaction=([\d.]+) N',log)
        admittance_steps = len(re.findall(r'Floor seating step \d+:', log))
        boxes.append({'index':index, 'target':f'pallet_box_r{index//3}_c{index%3}',
                      'wrist_force_admittance_verified':True, 'floor_admittance_steps':admittance_steps,
                      'empty_return_verified':returned, 'minimum_planned_tool_body_clearance_m':min(clearance),
                      'floor_before_open':dict(zip(('box_center_z_m','tilt_deg','reaction_n'),map(float,floor.groups())))})
    out.mkdir(parents=True, exist_ok=False)
    videos = []
    for source_name, dest_name, frames in (
        ('overview_8x.webm','nine_boxes_continuous_clean_8x.mp4',recording['quick_frames']),
        ('full_process.webm','nine_boxes_continuous_clean_original.mp4',recording['frames'])):
        destination = out / dest_name
        subprocess.run([sys.executable, str(ROOT/'scripts/convert_pallet_video_mp4.py'), str(source/source_name),
                        str(destination), '--av-path', str(args.av_path)], check=True)
        video = json.loads(destination.with_suffix('.json').read_text())
        if video['frames'] != frames or not video['all_source_frames_preserved'] or not video['all_frames_decoded']:
            raise RuntimeError(f'{dest_name}: incomplete output or changed source range')
        videos.append(video)
    result = {'passed':True, 'source':str(source), 'order':ORDER, 'row_order':['第一排','第三排','中间排'],
              'single_continuous_scene':True, 'resumed':False, 'stitched':False, 'caption_cards':False,
              'telemetry_overlay':True,
              'all_nine_initially_physical':True, 'newly_recorded_transfers':9, 'boxes':boxes, 'videos':videos,
              'receiving_bin_clearance':sequence['receiving_bin_clearance']}
    (out/'validation_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    rows = '\n'.join(f"| {b['target']} | {b['minimum_planned_tool_body_clearance_m']*1000:.1f} mm | {b['floor_before_open']['reaction_n']:.1f} N |" for b in boxes)
    readme = f'''# 九箱连续搬运与实时传感器数据录像

顺序：第一排 → 第三排 → 中间排。单个 Gazebo场景、单个录制器，从九箱初始状态完整运行；无续录、无源箱/机器人重建、无视频拼接。画面下方实时显示腰部俯仰角、升降台高度、左右腕部力模长及 XYZ 分量。

- [8倍速视频]({out}/nine_boxes_continuous_clean_8x.mp4)
- [原速视频]({out}/nine_boxes_continuous_clean_original.mp4)

两份视频均保留所有对应源帧并完成逐帧解码。尺寸960×540，H.264 baseline / yuv420p / faststart。

第一、三排先侧拉约120 mm，第三排使用延长至1.25 m的支撑板；中间排沿用对面推送与正面双臂取箱。所有未搬箱体持续保留。八次箱间空车返程、九次落底后松爪和收臂均通过。

夹爪与本体检查独立于任务碰撞豁免，覆盖IK、完整原始轨迹、重定时轨迹与执行中实测关节。要求3 mm间隙。下表为最终接受轨迹的最小规划间隙，不是实际关节测量的最小间隙。

| 箱体 | 最小规划夹爪—本体间隙 | 松爪前底部反力 |
|---|---:|---:|
{rows}

空车速度0.20 m/s、0.30 rad/s，负载速度0.12 m/s、0.18 rad/s；空载协调接近比例0.20/0.12，负载比例0.12/0.08，侧推TCP限速0.015 m/s。接触和低速落底保留独立速度控制。

接料端沿用既有仿真清箱：已放置箱仅在空车返回验证后从接料箱中移除；该操作不是实际自动卸料机构。腕部F/T数据不可用，落底依据Gazebo物理接触及箱底反力。自身腕部装配相邻几何和其他非夹爪历史豁免仍保留。

[逐箱验证与帧数]({out}/validation_result.json) · [源执行日志]({source}/sequence.log) · [控制修改记录]({ROOT}/docs/pallet_nine_continuous_clean_20261013.md)
'''
    (out/'README.md').write_text(readme)
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
