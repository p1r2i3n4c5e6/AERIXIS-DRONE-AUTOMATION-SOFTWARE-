import sys,time,threading
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import hybrid_tracker as h

def controller(target=None):
 c=h.UnifiedGimbalController.__new__(h.UnifiedGimbalController)
 c.running=True;c.state_lock=threading.Lock();c.mav_master=None;c.mav_active=False
 c._next_temp_check=float('inf');c.thermal_fault=False;c.observation_timeout=.4
 c.last_target=target;c.scan_active=False;c.prev_err_x=c.prev_err_y=1.
 c.smooth_yaw_rate=12.;c.smooth_pitch_rate=-7.
 c.target_yaw_deg=c.target_pitch_deg=0.;c.cmd_interval=.001;c.seq=0;c.actual_hz=0.
 return c
for target in (None, {'observed_t':time.monotonic()-1}):
 c=controller(target);sent=[]
 def tx(*args):
  sent.append(args);c.running=False
 c._transmit=tx;c._control_loop()
 assert sent[0][:2]==(0.,0.),sent
 assert c.prev_err_x==c.prev_err_y==0.
print('PASS: missing and stale target each emit zero on first control iteration, despite previous motion')
c=controller();c.preview_only=True
# A preview transmit must return without touching a socket or backend fields.
c._transmit(10,10,0,0)
c.center_gimbal();c.start_scan();c.toggle_scan()
assert not c.scan_active
print('PASS: preview has no command transmission; center and scan shortcuts cannot request motion')
class Thread:
 def join(self,timeout): pass
 def is_alive(self): return False
class Socket:
 closed=False
 def close(self): self.closed=True
c=controller();c.control_thread=Thread();c.udp_sock=Socket();sent=[]
c._transmit=lambda *args: sent.append(args)
c.stop()
assert len(sent)==3 and all(x[:2]==(0.,0.) for x in sent)
assert c.udp_sock.closed and not c.running
print('PASS: normal shutdown emits three zero requests then closes transport')
