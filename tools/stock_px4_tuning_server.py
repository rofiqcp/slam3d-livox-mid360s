#!/usr/bin/env python3
"""Local-only config staging and simulation preview. No Pixhawk access."""
import json
import pathlib
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import yaml
ROOT=pathlib.Path(__file__).resolve().parents[1]
RUNTIME=pathlib.Path.home()/'.local/share/slam3d-mid360s/tuning'
PAGE=ROOT/'foxglove/stock_px4_tuning.html'
STAGING=ROOT/'config/stock_px4_tuning.yaml'
PREVIOUS=RUNTIME/'stock_px4_tuning.previous.yaml'
SIM=ROOT/'config/stock_px4_simulation.yaml'
STATUS=pathlib.Path('/home/sirobo/.cache/mid360_pixhawk_ros/stock_px4_latest.json')
DEFAULT={'stop_distance_m':1.15,'slow_distance_m':3.0,'cruise_limit_mps':.75,
'avoid_limit_mps':.25,'deceleration_mps2':.8,'reaction_time_s':.45,
'safety_margin_m':.3,'left_clearance_m':1.2,'right_clearance_m':1.2,
'wall_kp':.6,'wall_ki':0.,'wall_kd':0.,'yaw_limit_radps':.25,
'wheel_track_m':0.0,'max_wheel_speed_mps':0.35,
'clear_confirm_frames':3,'advisory_timeout_ms':300,'mav_timeout_ms':1500,
'preferred_wall_side':'AUTO'}
BOUNDS={
'stop_distance_m':(.2,8),'slow_distance_m':(.4,20),
'cruise_limit_mps':(.05,3),'avoid_limit_mps':(.05,2),
'deceleration_mps2':(.1,4),'reaction_time_s':(.1,2),
'safety_margin_m':(0,2),'left_clearance_m':(.2,4),
'right_clearance_m':(.2,4),'wall_kp':(0,4),
'wall_ki':(0,2),'wall_kd':(0,2),
'yaw_limit_radps':(.05,2),'wheel_track_m':(0,4),
'max_wheel_speed_mps':(.05,3),
'clear_confirm_frames':(3,30),'advisory_timeout_ms':(100,5000),
'mav_timeout_ms':(300,10000)
}
INTEGER_FIELDS=('clear_confirm_frames','advisory_timeout_ms','mav_timeout_ms')
def check(v):
 import math
 if not isinstance(v,dict) or set(v)!=set(DEFAULT):
  raise ValueError('Incomplete or unexpected fields')
 for k,(lo,hi) in BOUNDS.items():
  x=v[k]
  if isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x):
   raise ValueError('Invalid number: '+k)
  if not lo<=x<=hi or (k in INTEGER_FIELDS and int(x)!=x):
   raise ValueError('Out of range: '+k)
 if v['preferred_wall_side'] not in ('AUTO','LEFT','RIGHT'):
  raise ValueError('Invalid preferred wall side')
 if v['slow_distance_m']<=v['stop_distance_m']:
  raise ValueError('Slow must exceed STOP')
 if v['avoid_limit_mps']>v['cruise_limit_mps']:
  raise ValueError('Avoid speed exceeds cruise')
 return v

def read(path):
 try:return check(yaml.safe_load(path.read_text()))
 except Exception:return DEFAULT.copy()
def write(path,v):
 path.parent.mkdir(parents=True,exist_ok=True)
 temp=path.with_suffix('.tmp')
 temp.write_text(yaml.safe_dump(v,sort_keys=True))
 temp.replace(path)
class Handler(BaseHTTPRequestHandler):
 def send(self,code,obj,ctype='application/json'):
  body=json.dumps(obj).encode() if ctype=='application/json' else obj
  self.send_response(code)
  self.send_header('Content-Type',ctype)
  self.send_header('Cache-Control','no-store')
  self.send_header('Content-Length',str(len(body)))
  self.end_headers();self.wfile.write(body)
 def do_GET(self):
  if self.path in ('/','/index.html'):
   return self.send(200,PAGE.read_bytes(),'text/html')
  if self.path=='/api/config':return self.send(200,{'config':read(STAGING)})
  if self.path=='/api/status':
   try:
    status=json.loads(STATUS.read_text())
    fresh=(time.time()*1000-status.get('observed_at_unix_ms',0)<2500)
    if not fresh:status={'state':'UNKNOWN','reason':'SUPERVISOR_HEARTBEAT_STALE'}
   except (OSError,ValueError):status={'state':'UNKNOWN','reason':'SUPERVISOR_NOT_RUNNING'};fresh=False
   return self.send(200,{'online':fresh,'supervisor':status})
  self.send(404,{'error':'Not found'})
 def do_POST(self):
  try:
   n=int(self.headers.get('Content-Length','0'))
   if n<0 or n>20000:raise ValueError('Request too large')
   data=json.loads(self.rfile.read(n) or b'{}')
   if self.path=='/api/save':
    cfg=check(data)
    write(PREVIOUS,read(STAGING))
    write(STAGING,cfg)
    result={'message':'Saved YAML staging only; Cube unchanged','config':cfg}
   elif self.path=='/api/apply-simulation':
    cfg=check(data);write(SIM,cfg)
    result={'message':'Saved simulation profile. Start simulator with this profile; no live runtime changes'}
   elif self.path=='/api/rollback':
    if not PREVIOUS.exists():raise ValueError('No previous profile')
    cfg=read(PREVIOUS);write(STAGING,cfg)
    result={'message':'Restored previous staging profile','config':cfg}
   elif self.path in ('/api/mcap/start','/api/mcap/stop'):
    raise ValueError('MCAP requires ROS2 bag CLI; not enabled in web server')
   else:return self.send(404,{'error':'Not found'})
   self.send(200,result)
  except (ValueError,TypeError) as e:self.send(400,{'error':str(e)})
if __name__=='__main__':
 server=ThreadingHTTPServer(('127.0.0.1',5002),Handler)
 print('STOCK PX4 SAFE TUNING LOCAL http://127.0.0.1:5002',flush=True)
 server.serve_forever()
