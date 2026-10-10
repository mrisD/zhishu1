#!/usr/bin/env python3
"""Transcode every frame to phone-compatible H.264 MP4, then decode-check."""
import argparse
from fractions import Fraction
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', type=Path)
parser.add_argument('destination', type=Path)
parser.add_argument('--av-path', type=Path)
parser.add_argument('--max-frames', type=int, help='Explicitly export a preview excerpt, not the full recording')
parser.add_argument('--start-frame', type=int, default=0, help='First source frame for an explicit excerpt')
args = parser.parse_args()
if args.av_path:
    sys.path.insert(0, str(args.av_path))
import av

if args.destination.exists():
    raise FileExistsError(args.destination)
args.destination.parent.mkdir(parents=True, exist_ok=True)
count = 0
with av.open(str(args.source)) as incoming:
    source = incoming.streams.video[0]
    rate = source.average_rate or Fraction(10, 1)
    with av.open(str(args.destination), 'w', options={'movflags': '+faststart'}) as outgoing:
        stream = outgoing.add_stream('libx264', rate=rate)
        stream.width, stream.height = source.width, source.height
        stream.pix_fmt = 'yuv420p'
        stream.options = {'crf': '21', 'preset': 'fast', 'profile': 'baseline', 'level': '3.1'}
        stream.codec_context.thread_count = 4
        for source_index, frame in enumerate(incoming.decode(video=0)):
            if source_index < args.start_frame:
                continue
            if args.max_frames is not None and count >= args.max_frames:
                break
            frame = frame.reformat(format='yuv420p')
            frame.pts = count
            frame.time_base = Fraction(rate.denominator, rate.numerator)
            for packet in stream.encode(frame):
                outgoing.mux(packet)
            count += 1
        for packet in stream.encode():
            outgoing.mux(packet)
with av.open(str(args.destination)) as check:
    decoded = sum(1 for _ in check.decode(video=0))
if not count or decoded != count:
    raise RuntimeError(f'Decode verification failed: {decoded}/{count}')
result = {'source': str(args.source), 'output': str(args.destination),
          'frames': count, 'fps': float(rate), 'seconds': count/float(rate),
          'all_frames_decoded': True, 'codec': 'H.264 baseline', 'pixel_format': 'yuv420p',
          'all_source_frames_preserved': args.max_frames is None and args.start_frame == 0,
          'preview_excerpt': args.max_frames is not None or args.start_frame > 0,
          'source_start_frame': args.start_frame, 'faststart': True}
args.destination.with_suffix('.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result), flush=True)
