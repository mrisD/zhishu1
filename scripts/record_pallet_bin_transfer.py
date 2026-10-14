#!/usr/bin/env python3
"""Record real Gazebo sensor images with phase-dependent views, never replay poses.

Run after sourcing ROS Jazzy. The generated world only adds collision-free
observer cameras and changes visual contrast; all physics / tote geometry is
copied unchanged. WebM/VP8 uses the existing GStreamer encoder, without installing
system packages or capturing the user's desktop.
"""

import argparse
from collections import deque
import json
import math
import signal
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path


CAMERAS = {
    "rear": ((3.80, 2.50, 2.70), (1.45, 0.30, 0.90), 1.05),
    "pick": ((-1.35, 1.95, 2.10), (0.65, 0.05, 0.60), 1.05),
    "overview": ((-2.60, -3.80, 3.10), (0.66, -0.75, 0.63), 1.05),
    "place": ((1.95, -2.90, 2.10), (0.92, -1.55, 0.73), 1.05),
}
WIDTH, HEIGHT = 960, 540
FOOTER_HEIGHT = 110
STAGES = {
    0: "初始场景：360 mm 夹指，浅夹取约 50 mm",
    1: "1/6 真实接触预移位：侧拉或侧推，按实测箱位抓取",
    2: "2/6 重新定位，双臂下探闭合与抬升",
    3: "3/6 腰臂协调直腰，保持料箱水平",
    4: "4/6 绕过接料台，转运至 0.55 m 桌面",
    5: "5/6 双臂同步入箱，松爪并竖直撤离",
    6: "6/6 料箱稳定落底，机械臂回位",
    7: "验证完成：Gazebo 真实接触搬运",
    9: "本轮未完成：请查看验证日志",
    8: "5/6 低速落底：保持闭爪，确认箱底接触",
    10: "5/6 落底已确认，再松爪并竖直撤离",
    11: "返程：空车返回托盘，暂不清除已放置箱",
    12: "仿真场景操作：已返托盘，清除大箱内上一箱",
    13: "推箱位置已确认：撤爪后保持倾角收臂起身",
    14: "双爪已张开：绕托盘返回正面取箱",
}


def make_world(args):
    tree = ET.parse(args.source)
    world = tree.getroot().find("world")
    world.find("scene/background").text = "0.12 0.15 0.20 1"
    ground = world.find("model[@name='ground_plane']/link/visual/material")
    ground.find("ambient").text = "0.30 0.34 0.38 1"
    ground.find("diffuse").text = "0.42 0.46 0.50 1"
    for name, (eye, target, fov) in CAMERAS.items():
        dx, dy, dz = [target[i] - eye[i] for i in range(3)]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy))
        model = ET.SubElement(world, "model", name=f"video_{name}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, (*eye, 0, pitch, yaw)))
        link = ET.SubElement(model, "link", name="observer")
        sensor = ET.SubElement(link, "sensor", name=f"{name}_camera", type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = str(args.fps)
        ET.SubElement(sensor, "topic").text = f"/pallet_video/{name}/image"
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = str(fov)
        image = ET.SubElement(camera, "image")
        ET.SubElement(image, "width").text = str(WIDTH)
        ET.SubElement(image, "height").text = str(HEIGHT)
        ET.SubElement(image, "format").text = "R8G8B8"
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.04"
        ET.SubElement(clip, "far").text = "30"
    ET.indent(tree, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(f"Observer world generated: {args.output}", flush=True)


class CameraReceiver:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        self.frames = {}
        self.history = {name: deque(maxlen=30) for name in CAMERAS}
        self.lock = threading.Lock()
        self.node = Node()
        self.callbacks = []
        self.closed = False
        for name in CAMERAS:
            callback = lambda message, name=name: self.receive(name, message)
            self.callbacks.append(callback)
            if not self.node.subscribe(Image, f"/pallet_video/{name}/image", callback):
                raise RuntimeError(f"Cannot subscribe to {name} camera")

    def receive(self, name, message):
        if self.closed:
            return
        import numpy as np
        channels = {3: 3, 8: 3, 4: 4, 5: 4}.get(message.pixel_format_type)
        if channels is None:
            return
        stride = message.step or message.width * channels
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, stride)
        image = image[:, :message.width * channels].reshape(message.height, message.width, channels)
        image = image[:, :, :3]
        if message.pixel_format_type in (5, 8):
            image = image[:, :, ::-1]
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        with self.lock:
            frame = (image.copy(), time.monotonic(), stamp)
            self.frames[name] = frame
            self.history[name].append(frame)

    def get(self, name):
        with self.lock:
            return self.frames.get(name)

    def at_sim_time(self, name, stamp):
        with self.lock:
            frames = self.history[name]
            if not frames or frames[-1][2] + 1e-6 < stamp:
                return None
            if frames[0][2] > stamp + 2.0 / 30.0:
                raise RuntimeError(f"{name} camera buffer overrun at simulation time {stamp:.3f}")
            return min(frames, key=lambda frame: abs(frame[2] - stamp))

    def close(self):
        self.closed = True
        for name in CAMERAS:
            self.node.unsubscribe(f"/pallet_video/{name}/image")
        self.node = None
        self.callbacks.clear()


def preview(args):
    from PIL import Image
    args.output_dir.mkdir(parents=True, exist_ok=True)
    receiver = CameraReceiver()
    pending = set(CAMERAS)
    deadline = time.monotonic() + args.wait
    while pending and time.monotonic() < deadline:
        for name in list(pending):
            frame = receiver.get(name)
            if frame:
                Image.fromarray(frame[0]).save(args.output_dir / f"{name}.png")
                print(f"Saved actual Gazebo {name} image, sim_time={frame[2]:.3f}", flush=True)
                pending.remove(name)
        time.sleep(0.1)
    if pending:
        raise RuntimeError(f"No rendered images from {sorted(pending)}")


class WebMWriter:
    def __init__(self, path, fps, width=WIDTH, height=HEIGHT + FOOTER_HEIGHT):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        self.gst = Gst
        Gst.init(None)
        self.index = 0
        self.fps = fps
        self.pipeline = Gst.parse_launch(
            f'appsrc name=frames format=time block=true '
            f'caps="video/x-raw,format=RGB,width={width},height={height},framerate={fps}/1" '
            f'! queue max-size-buffers=4 ! videoconvert '
            f'! vp8enc deadline=1 cpu-used=6 target-bitrate=1500000 keyframe-max-dist={fps * 5} '
            f'! webmmux ! filesink location="{path}"'
        )
        self.source = self.pipeline.get_by_name("frames")
        self.pipeline.set_state(Gst.State.PLAYING)

    def write(self, image):
        data = image.tobytes()
        buffer = self.gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        buffer.pts = buffer.dts = int(self.index * self.gst.SECOND / self.fps)
        buffer.duration = int(self.gst.SECOND / self.fps)
        if self.source.emit("push-buffer", buffer) != self.gst.FlowReturn.OK:
            raise RuntimeError("GStreamer encoder did not accept frame")
        self.index += 1
        message = self.pipeline.get_bus().pop_filtered(self.gst.MessageType.ERROR)
        if message:
            raise RuntimeError(str(message.parse_error()))

    def close(self):
        self.source.emit("end-of-stream")
        message = self.pipeline.get_bus().timed_pop_filtered(
            20 * self.gst.SECOND, self.gst.MessageType.EOS | self.gst.MessageType.ERROR)
        self.pipeline.set_state(self.gst.State.NULL)
        if message is None:
            raise RuntimeError("Timed out finalizing WebM")
        if message.type == self.gst.MessageType.ERROR:
            raise RuntimeError(str(message.parse_error()))


def record(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from ros_gz_interfaces.msg import Contacts
    from PIL import Image, ImageDraw, ImageFont
    args.output_dir.mkdir(parents=True, exist_ok=True)
    receiver = CameraReceiver()
    rclpy.init()
    node = rclpy.create_node("pallet_video_recorder")
    data, data_lock = {}, threading.Lock()

    def joints(message):
        positions = dict(zip(message.name, message.position))
        with data_lock:
            data["waist_lift"] = positions.get("waist_up_down_joint")
            pitch = positions.get("waist_pitch_joint")
            data["waist_pitch_deg"] = None if pitch is None else math.degrees(pitch)
            data["left_gripper_q"] = positions.get("left_gripper_left_finger_joint")
            data["right_gripper_q"] = positions.get("gripper_left_finger_joint")

    current_target = [args.target_model]
    def contacts(message):
        matching = [c for c in message.contacts if
                    "large_bin" in (c.collision1.name + c.collision2.name) and
                    current_target[0] in (c.collision1.name + c.collision2.name) and
                    "bottom" in c.collision1.name and "bottom" in c.collision2.name]
        with data_lock:
            data["floor_contact"] = bool(matching)
            data["floor_reaction_n"] = sum(abs(w.body_1_wrench.force.z) for c in matching for w in c.wrenches)

    def box(message):
        p, q = message.pose.position, message.pose.orientation
        tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))
        with data_lock:
            data["box"] = [p.x, p.y, p.z]
            data["box_tilt_deg"] = tilt
            data["box_received"] = time.monotonic()

    def base(message):
        p = message.pose.position
        with data_lock:
            data["base"] = [p.x, p.y, p.z]

    node.create_subscription(JointState, "/joint_states", joints, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, "/pallet_bin/measured_box_pose", box, 10)
    node.create_subscription(PoseStamped, "/pallet_bin/measured_base_pose", base, 10)
    node.create_subscription(Contacts, "/pallet_bin/tote_bottom_contacts", contacts, 10)
    from rclpy.executors import SingleThreadedExecutor
    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(node)
    executor = threading.Thread(target=ros_executor.spin, daemon=True)
    executor.start()
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
    title_font = ImageFont.truetype(font_path, 21)
    small_font = ImageFont.truetype(font_path, 15)
    stage, offset, stop_at, passed = 0, 0, None, False
    completed_transfers = args.initial_completed_transfers
    cycle = completed_transfers + 1
    stop_requested = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_requested.set())
    writers = []
    telemetry = (args.output_dir / "telemetry.jsonl").open("w")
    started = None
    sim_started = None
    index, last_stage, final_data = 0, -1, {}
    deadline = time.monotonic() + args.max_seconds
    try:
        while time.monotonic() < deadline and not stop_requested.is_set():
            now = time.monotonic()
            if args.log.exists():
                with args.log.open(errors="replace") as handle:
                    handle.seek(offset)
                    lines = handle.readlines()
                    offset = handle.tell()
                for line in lines:
                    if "SEQUENCE_TARGET " in line:
                        current_target[0] = line.split("SEQUENCE_TARGET ", 1)[1].strip()
                        cycle = completed_transfers + 1
                        stage = 0
                        with data_lock:
                            data.pop("floor_verified_before_open", None)
                            data.pop("box", None)
                    elif "SEQUENCE_CLEAR_DESTINATION" in line:
                        stage = 12
                    elif "Returning empty robot to pallet" in line:
                        stage = 11
                    elif "Pre-extracting perimeter tote" in line:
                        stage = 1
                    elif "coordinated front pull approach: planning succeeded" in line or "physical side push without attachment: planning succeeded" in line:
                        stage = 1
                    elif "Starting complete pallet -> large-bin transfer" in line:
                        stage = 2
                    elif "Physical side push verified" in line:
                        stage = 13
                    elif "Both parked grippers physically open before dual grasp" in line:
                        stage = 14
                    elif "Physical lift verified" in line:
                        stage = 4
                    elif "large-bin insertion" in line and "planning succeeded" in line:
                        stage = 5
                    elif "Floor seating step" in line:
                        stage = 8
                    elif "Floor support verified BEFORE opening" in line:
                        stage = 10
                        with data_lock:
                            data["floor_verified_before_open"] = True
                    elif "Physical large-bin placement verified" in line:
                        stage = 6
                    elif "Complete transfer finished" in line:
                        completed_transfers += 1
                        if completed_transfers == args.expected_transfers:
                            stage, passed, stop_at = 7, True, now + 8
                    elif "[FATAL]" in line:
                        stage, passed, stop_at = 9, False, now + 8
            with data_lock:
                state = dict(data)
            if stage == 2 and state.get("box", [0, 0, 0])[2] > 0.55:
                stage = 3
            camera = "pick" if stage in (1, 2) else "overview"
            if stage in (0, 1) and args.rear_push:
                camera = "rear"
            if stage == 13:
                camera = "rear"
            if stage == 14:
                camera = "overview"
            if stage in (0, 1, 2, 3) and state.get("base", [0, 0, 0])[0] > 1.65:
                camera = "rear"
            at_bin = state.get("base", [0, 0, 0])[0] > 0.45 and state.get("base", [0, 0, 0])[1] < -1.4
            if stage in (5, 7, 8, 10, 12) or (stage == 4 and at_bin):
                camera = "place"
            frame = receiver.get(camera)
            if frame is None:
                time.sleep(0.05)
                continue
            if now - frame[1] > 15:
                raise RuntimeError(f"Selected {camera} camera stopped publishing; recording aborted")
            if started is None:
                started = now
                sim_started = frame[2]
                video_height = HEIGHT if args.clean_video else HEIGHT + FOOTER_HEIGHT
                writers = [WebMWriter(args.output_dir / "full_process.webm", args.fps, height=video_height),
                           WebMWriter(args.output_dir / f"overview_{args.speedup}x.webm", args.fps, height=video_height)]
                print("Recording actual rendered images; both video encoders ready", flush=True)
            elapsed = now - started
            if args.sampling_time == "simulation":
                target_sim_time = sim_started + index / args.fps
                frame = receiver.at_sim_time(camera, target_sim_time)
                if frame is None:
                    time.sleep(0.002)
                    continue
                elapsed = target_sim_time - sim_started
            canvas = Image.new("RGB", (WIDTH, HEIGHT + FOOTER_HEIGHT), (14, 22, 31))
            canvas.paste(Image.fromarray(frame[0]), (0, 48))
            draw = ImageDraw.Draw(canvas)
            title = STAGES[stage] if stage != 0 else (
                f"初始场景：{args.finger_length_mm} mm 手指，最大弯腰 {args.waist_limit_deg:g}°")
            if stage == 5 and args.floor_seated:
                title = "5/6 入箱：先低速落底，接触确认后才松爪"
            if (args.total_boxes or args.expected_transfers) > 1:
                title = f"箱 {cycle}/{args.total_boxes or args.expected_transfers}　" + title
            draw.text((12, 6), title, font=title_font, fill=(240, 246, 251))
            waist = state.get("waist_pitch_deg")
            angle_text = "实测弯腰：等待关节数据" if waist is None else f"实测弯腰：{waist:.2f}° | 允许下限 {args.waist_limit_deg:g}°"
            draw.text((12, HEIGHT + 80), angle_text, font=small_font, fill=(255, 226, 130))
            draw.text((12, HEIGHT + 52),
                      f"15 kg | 手指 {args.finger_length_mm} / TCP {380 if args.rear_push and stage in (0, 1, 13) else args.tcp_offset_mm} mm | " +
                      (f"箱心 {state.get('box', [0, 0, 0])[2]:.3f} m / 箱底反力 {state.get('floor_reaction_n', 0):.0f} N" if args.floor_seated else "浅夹取约 50 mm | 桌面 0.55 m"),
                      font=small_font, fill=(211, 224, 234))
            draw.text((WIDTH - 140, 12), f"{int(elapsed)//60:02d}:{int(elapsed)%60:02d} / 原速",
                      font=small_font, fill=(240, 220, 145))
            if now - frame[1] > 3:
                draw.text((12, 64), "图像更新暂停", font=title_font, fill=(255, 150, 80))
            if args.clean_video:
                canvas = Image.fromarray(frame[0])
            writers[0].write(canvas)
            if index % args.speedup == 0:
                if not args.clean_video:
                    draw.rectangle((WIDTH - 145, 5, WIDTH, 42), fill=(14, 22, 31))
                    draw.text((WIDTH - 140, 12), f"{int(elapsed)//60:02d}:{int(elapsed)%60:02d} / {args.speedup}倍速",
                              font=small_font, fill=(240, 220, 145))
                writers[1].write(canvas)
            if stage != last_stage:
                prefix = f"cycle_{cycle}_" if (args.total_boxes or args.expected_transfers) > 1 else ""
                canvas.save(args.output_dir / f"{prefix}stage_{stage}_{camera}.png")
                print(f"Stage {stage}, view={camera}, real elapsed={elapsed:.1f}s", flush=True)
                last_stage = stage
            if index % args.fps == 0:
                final_data = {"wall_time": time.time(), "elapsed": elapsed, "stage": stage,
                              "camera": camera, "image_sim_time": frame[2],
                              "image_age": now - frame[1], "cycle": cycle, "target": current_target[0], **state}
                telemetry.write(json.dumps(final_data) + "\n")
                telemetry.flush()
            index += 1
            if stop_at is not None and now >= stop_at:
                break
            # Simulation sampling advances only when timestamped camera frames
            # arrive; wall-time simulation slowdown does not duplicate footage.
            if args.sampling_time == "wall":
                target_tick = started + index / args.fps
                time.sleep(max(0, target_tick - time.monotonic()))
    finally:
        for writer in writers:
            writer.close()
        telemetry.close()
        ros_executor.shutdown(timeout_sec=2)
        executor.join(timeout=2)
        node.destroy_node()
        receiver.close()
        if rclpy.ok():
            rclpy.shutdown()
        summary = {"passed": passed, "frames": index, "fps": args.fps,
                   "completed_transfers": completed_transfers,
                   "initial_completed_transfers": args.initial_completed_transfers,
                   "recorded_completed_transfers": completed_transfers - args.initial_completed_transfers,
                   "full_duration": index / args.fps, "views": CAMERAS,
                   "sampling_time": args.sampling_time, "sim_start": sim_started,
                   "log": str(args.log), "final_telemetry": final_data,
                   "speedup": args.speedup,
                   "quick_frames": writers[1].index if writers else 0,
                   "finger_length_mm": args.finger_length_mm,
                   "tcp_offset_mm": args.tcp_offset_mm,
                   "waist_limit_deg": args.waist_limit_deg,
                   "floor_seated": args.floor_seated,
                   "rear_push": args.rear_push,
                   "target_model": args.target_model,
                   "clean_video": args.clean_video,
                   "video_width": WIDTH, "video_height": HEIGHT if args.clean_video else HEIGHT + FOOTER_HEIGHT,
                   "note": f"Real Gazebo rendered images, {args.sampling_time}-time sampling; uniform {args.speedup}x overview"}
        (args.output_dir / "recording_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if not passed:
        raise SystemExit("Recording ended without a complete-transfer success log")
    print("Full process passed; videos finalized", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    world = subparsers.add_parser("world")
    world.add_argument("--source", type=Path, required=True)
    world.add_argument("--output", type=Path, required=True)
    world.add_argument("--fps", type=int, default=30)
    world.set_defaults(action=make_world)
    pre = subparsers.add_parser("preview")
    pre.add_argument("--output-dir", type=Path, required=True)
    pre.add_argument("--wait", type=float, default=60)
    pre.set_defaults(action=preview)
    rec = subparsers.add_parser("record")
    rec.add_argument("--output-dir", type=Path, required=True)
    rec.add_argument("--log", type=Path, required=True)
    rec.add_argument("--fps", type=int, default=30)
    rec.add_argument("--sampling-time", choices=("simulation", "wall"), default="simulation")
    rec.add_argument("--max-seconds", type=float, default=2400)
    rec.add_argument("--speedup", type=int, choices=(2, 4, 8), default=4)
    rec.add_argument("--finger-length-mm", type=int, default=200)
    rec.add_argument("--tcp-offset-mm", type=int, default=140)
    rec.add_argument("--waist-limit-deg", type=float, default=-90)
    rec.add_argument("--floor-seated", action="store_true")
    rec.add_argument("--expected-transfers", type=int, default=1)
    rec.add_argument("--initial-completed-transfers", type=int, default=0)
    rec.add_argument("--total-boxes", type=int, default=0)
    rec.add_argument("--target-model", default="pallet_box_r0_c0")
    rec.add_argument("--rear-push", action="store_true")
    rec.add_argument("--clean-video", action="store_true", help="Record only camera pixels, without title cards, captions or black bars")
    rec.set_defaults(action=record)
    args = parser.parse_args()
    args.action(args)


if __name__ == "__main__":
    main()
