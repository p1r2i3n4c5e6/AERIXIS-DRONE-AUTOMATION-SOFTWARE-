import sys
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import hybrid_tracker as h

def track(i,t,box=None):
 box=box or [100,60,200,300]
 return dict(id=i,box=box,observed_box=box,conf=.9,observed_t=t)

def selected():
 l=h.CenterLockStateMachine(640,360)
 with patch.object(h.time,'monotonic',return_value=100.):
  assert l.update([track(1,100.)])['id']==1
 return l

l=selected()
with patch.object(h.time,'monotonic',return_value=101.):
 candidate=track(2,101.,[110,60,210,300])
 for _ in range(50): assert l.update([candidate]) is None
 assert l.recovery_hits==1 and l.locked_id==1
for t in (101.15,101.3):
 with patch.object(h.time,'monotonic',return_value=t):
  result=l.update([track(2,t,[110,60,210,300])])
assert result['id']==2 and result['observed_t']==101.3
print('PASS: new ID recovers after three distinct observations; repeated display cannot trigger recovery')
for case in ('crowd','far','late','stale'):
 l=selected()
 for t in (101.,101.15,101.3):
  tracks=[track(2,t)]
  now=t
  if case=='crowd': tracks.append(track(3,t,[300,60,400,300]))
  if case=='far': tracks=[track(2,t,[450,60,550,300])]
  if case=='late': now=t+4
  if case=='stale': tracks=[track(2,t-.6)]
  with patch.object(h.time,'monotonic',return_value=now):
   assert l.update(tracks) is None
 assert l.locked_id==1
print('PASS: crowded, distant, long-gap and stale candidates cannot take over the lock')
# Simulate motion-model drift away from an actual returning detection.
t=h.OCSORTMultiTracker(min_hits=1)
a=t.update([[100,60,200,300,.9]],timestamp=100.)[0]
t.tracks[0].x[4]=1000.
b=t.update([[110,60,210,300,.9]],timestamp=100.5)[0]
assert b['id']==a['id'] and b['observed_t']==100.5
assert b['velocity']==(0.,0.)
print('PASS: unambiguous last-observation fallback preserves ID and resets drift')
# An arbitrarily old track must not survive merely because update matches first.
t=h.OCSORTMultiTracker(min_hits=1)
a=t.update([[100,60,200,300,.9]],timestamp=100.)[0]
b=t.update([[100,60,200,300,.9]],timestamp=105.)[0]
assert a['id']!=b['id']
print('PASS: expired tracks are removed before association')
