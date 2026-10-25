import sys, logging
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import hybrid_tracker as h

class ClosingPipe:
 closed=False
 def send(self, value): raise BrokenPipeError('simulated Windows pipe closure')
 def close(self): self.closed=True
class Process:
 alive=True
 joins=0
 def is_alive(self): return self.alive
 def join(self, timeout): self.joins+=1
 def terminate(self): self.alive=False
worker=h.AsyncYOLOInferencer.__new__(h.AsyncYOLOInferencer)
worker.conn=ClosingPipe();worker.process=Process();worker.busy=False
worker.stop()
assert worker.conn.closed and worker.process.joins==2 and not worker.process.alive
print('PASS: closed child pipe cannot prevent worker join/termination')

events=[]
def broken():
 events.append('inference')
 raise BrokenPipeError('simulated shutdown race')
class Log:
 def exception(self,*args): pass
failures=h._cleanup_resources([
 ('gimbal',lambda:events.append('gimbal')),
 ('inference',broken),
 ('capture',lambda:events.append('capture')),
 ('telemetry',lambda:events.append('telemetry'))],Log())
assert failures==1 and events==['gimbal','inference','capture','telemetry']
print('PASS: gimbal stop runs first; inference failure does not skip remaining cleanup')

class FailedCapture:
 released=False
 def set(self,*args): pass
 def isOpened(self): return False
 def release(self): self.released=True
cap=FailedCapture()
grabber=h.LowLatencyA8Grabber.__new__(h.LowLatencyA8Grabber)
grabber.stream_urls=['rtsp://192.168.144.25:8554/main.264']
with patch.object(h.cv2,'VideoCapture',return_value=cap):
 assert not grabber._try_connect()
assert cap.released
print('PASS: failed RTSP capture handle is released')

root=Path(__file__).resolve().parents[1]
for name in ('run.sh','run_windows.bat','run_preview.bat','run_tracking.bat'):
 text=(root/name).read_text()
 assert 'rtsp://192.168.144.25:8554/main.264' in text and '/sub.264' not in text
print('PASS: all launchers use the main stream already observed working in the user log')
