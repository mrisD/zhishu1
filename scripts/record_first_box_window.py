#!/usr/bin/env python3
"""Capture the actual Gazebo window until the first transfer finishes."""
import argparse,json,os,re,signal,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--output-dir',type=Path,required=True)
p.add_argument('--log',type=Path,required=True)
a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
encoder=None;passed=False;started=time.monotonic()
try:
 for _ in range(90):
  windows=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
  match=re.search(r'(0x[0-9a-f]+) "Gazebo Sim": \("gz-sim-gui"',windows)
  if match:break
  time.sleep(1)
 else:raise RuntimeError('Gazebo window not available')
 window=match.group(1)
 encoder=subprocess.Popen(['ffmpeg','-y','-hide_banner','-loglevel','warning','-f','x11grab',
  '-framerate','15','-window_id',window,'-i',os.environ.get('DISPLAY',':1'),
  '-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-preset','ultrafast','-crf','23',
  '-pix_fmt','yuv420p',str(a.output_dir/'first_box_full.mp4')],stdin=subprocess.PIPE)
 print('Recording Gazebo window '+window,flush=True)
 while encoder.poll() is None:
  log=a.log.read_text(errors='replace') if a.log.exists() else ''
  if 'Complete transfer finished' in log:
   passed=True;time.sleep(5);break
  if '[FATAL]' in log:break
  time.sleep(1)
finally:
 if encoder is not None and encoder.poll() is None:
  encoder.communicate(b'q',timeout=30)
 (a.output_dir/'recording_summary.json').write_text(json.dumps({'passed':passed,'elapsed_wall_s':time.monotonic()-started,'target':'pallet_box_r0_c0','expected_transfers':1,'source':'actual Gazebo window'},indent=2))
if not passed:raise SystemExit('First-box transfer did not complete')
