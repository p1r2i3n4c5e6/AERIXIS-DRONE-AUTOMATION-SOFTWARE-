"""Official OC-SORT plus OpenCV NanoTrack, with one persistent selection, for any detected class.

Nano is a visual measurement, never inserted into OC-SORT as a YOLO detection.
Detection/source-frame timestamps remain distinct from Nano timestamps.

Any class (2026-10-24): one OC-SORT per class, so a dog box never continues a person's track.
Track ids are ``class * CLASS_ID_STRIDE + OC-SORT id`` (class 0 keeps the plain OC-SORT id). The
selection only ever re-associates with detections of its own class; lock_classes limits which
classes may be selected (None = any).

Lock until the target is gone (2026-10-24, user request): the lock is kept while the target is in
the frame. When the detector is quiet (small target at altitude, motion blur) a confident
NanoTrack (score >= hold_score, box clear of the frame edge) keeps driving for up to nano_hold_s;
any detection elsewhere without one on the target still stops movement (ambiguous). The
selection is released only when the target leaves the frame (lost at the edge for
edge_release_s) or nothing has confirmed it for release_after_s; then the next target nearest
the centre is selected. Before, a selection could stay held forever with the gimbal stopped.
"""
import logging
import math
import os
import time
from pathlib import Path
import cv2
import numpy as np
from vendor.ocsort.ocsort import OCSort


CLASS_ID_STRIDE = 10000


def iou(a, b):
    if a is None or b is None:
        return 0.
    inter = max(0.,min(a[2],b[2])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[1],b[1]))
    area = max(1.,(a[2]-a[0])*(a[3]-a[1]))+max(1.,(b[2]-b[0])*(b[3]-b[1]))-inter
    return inter/area


def nano_factory(model_dir):
    """NanoTrack engine. Default: OpenCV's TrackerNano. A8_NANO_BACKEND=ncnn: the ncnn port
    (nano_ncnn.py, same boxes and scores, ~25 vs ~40 ms per update on the Pi 5) - not the default
    because the ncnn Python binding keeps the GIL during inference: in the tracker process the 50 Hz
    gimbal loop fell from 45 to 33 Hz for a gain of only ~0.5 fps on the saturated Pi (2026-10-26).
    Use it from its own process, or once the binding releases the GIL."""
    backend=os.environ.get('A8_NANO_BACKEND','opencv').lower()
    if backend=='ncnn':
        import nano_ncnn
        if not nano_ncnn.available(model_dir):
            raise RuntimeError('A8_NANO_BACKEND=ncnn but ncnn or the NanoTrack .ncnn files are missing')
        return nano_ncnn.NanoTrackNcnn(model_dir,threads=1)
    return opencv_nano(model_dir)


def opencv_nano(model_dir):
    if not (hasattr(cv2, 'TrackerNano_create') or
            (hasattr(cv2, 'TrackerNano') and hasattr(cv2.TrackerNano, 'create'))):
        raise RuntimeError('OpenCV TrackerNano is missing; install requirements-dual.txt in a virtual environment')
    params = cv2.TrackerNano_Params()
    params.backbone = str(Path(model_dir)/'nano/nanotrack_backbone_sim.onnx')
    params.neckhead = str(Path(model_dir)/'nano/nanotrack_head_sim.onnx')
    for path in (params.backbone, params.neckhead):
        if not Path(path).is_file():
            raise FileNotFoundError(f'NanoTrack model missing: {path}')
    params.backend = cv2.dnn.DNN_BACKEND_OPENCV
    params.target = cv2.dnn.DNN_TARGET_CPU
    if hasattr(cv2, 'TrackerNano_create'):
        return cv2.TrackerNano_create(params)
    return cv2.TrackerNano.create(params)


class DualTracker:
    """Full upstream OC-SORT for person detections plus NanoTrack between them.

    YOLO/OC-SORT chooses and checks identity. NanoTrack locates the selected
    person between detector results. Low confidence stops camera movement.
    """
    def __init__(self, model_dir, width=640, height=360, score_min=.45,
                 detector_grace=2., reconnect_window=5., factory=None,
                 clock=time.monotonic, names=None, lock_classes=None, nano_hold_s=4.,
                 hold_score=.6, release_after_s=5., edge_release_s=1., edge_margin=.04,
                 min_visible=.4, ambiguity_ratio=.5, doubt_release_s=1.5, camera_motion=True):
        self.names=dict(names or {0:'person'})
        self.lock_classes=None if lock_classes is None else {int(c) for c in lock_classes}
        self.model_dir=model_dir;self.factory=factory or (lambda:nano_factory(model_dir))
        self.nano=self.factory()  # Validate Nano models before actuator startup.
        self.width,self.height=width,height;self.score_min=score_min
        self.detector_grace=detector_grace;self.reconnect_window=reconnect_window
        self.nano_hold_s=max(detector_grace,nano_hold_s);self.hold_score=hold_score
        self.release_after_s=release_after_s;self.edge_release_s=edge_release_s
        self.edge_margin=edge_margin;self.min_visible=min_visible;self.ambiguity_ratio=ambiguity_ratio
        self.doubt_release_s=doubt_release_s
        self.camera_motion=camera_motion   # compensate gimbal turns before OC-SORT matching
        self.clock=clock
        self.log=logging.getLogger('dual_tracking');self.reset()
        self.log.info('NanoTrack engine: %s',type(self.nano).__name__)

    @staticmethod
    def _make_oc():
        return OSortNew(det_thresh=.45,max_age=30,min_hits=2,iou_threshold=.15,
                        delta_t=3,asso_func='iou',inertia=.2,use_byte=True)

    def reset(self):
        self.ocs={}
        self.oc_id=None;self.selected_id=None;self.selected_cls=None;self.oc_generation=0
        self.selected_generation=None;self.state='SEARCHING'
        self.reason='waiting for confirmed person';self.nano_ready=False
        self.nano_initialized=False;self.motion_block=None
        self.uncertain_since=None;self.good_after_failure=0
        self.seed_ms=0.;self.seed_count=0;self.replay_count=0
        self.last_status=None
        self.box=None;self.score=0.;self.nano_ms=0.;self.last_nano_frame_id=-1
        self.last_position_t=float('-inf');self.last_detection_t=float('-inf')
        self.last_detection_box=None;self.last_result_t=float('-inf')
        self.rows=[];self.recovery_id=None;self.recovery_hits=0
        self.last_processed_result=-1;self.visual_records=[];self.box_by_frame={}
        self.left_frame_t=None;self.releases=0;self.block_since=None;self.miss_with_others_t=float('-inf')
        self.cmc_prev=None;self.last_shift=(0.,0.)

    def _new_oc(self):
        self.ocs={}
        self.oc_id=None
        self.oc_generation+=1

    def _nano_step(self,frame,frame_id,frame_t):
        if not self.nano_initialized or frame_id<=self.last_nano_frame_id:return
        if not 0<=self.clock()-frame_t<=.4:
            self.nano_ready=self.nano_initialized=False
            self.state='COAST';self.reason='stale frame; stop requested';return
        if self.uncertain_since is not None and self.clock()-self.uncertain_since>.25:
            self.nano_ready=self.nano_initialized=False
            self.reason='Nano recovery expired; waiting for detector';return
        started=time.perf_counter();ok,xywh=self.nano.update(frame)
        self.nano_ms=(time.perf_counter()-started)*1000.
        try:self.score=float(self.nano.getTrackingScore())
        except Exception:self.score=0.
        x,y,w,h=map(float,xywh);candidate=[x,y,x+w,y+h];old=self.box or candidate
        at_edge=self._at_edge(old) or not self._inside(candidate)
        if ok and np.isfinite(candidate).all() and self._visible(candidate)<self.min_visible:
            # Most of the box is outside the picture: the target is leaving. NanoTrack would keep a
            # confident score on the image border (laptop pan test, 2026-10-24), so stop here.
            if self.left_frame_t is None:self.left_frame_t=self.clock()
            self.last_nano_frame_id=frame_id
            self.nano_ready=False;self.state='COAST';self.reason='target leaving the frame; movement stopped'
            return
        dx=abs((candidate[0]+candidate[2]-old[0]-old[2])/2)
        dy=abs((candidate[1]+candidate[3]-old[1]-old[3])/2)
        # Even an off-screen box is only a tracker estimate. A failed update
        # must never release the person's identity and select a bystander.
        valid=(ok and np.isfinite(candidate).all() and np.isfinite(self.score) and
               self.score>=self.score_min and w>=5 and h>=8 and
               x+w>0 and y+h>0 and x<self.width and y<self.height and
               # one frame (40 ms) never moves a target by more than ~3/4 of its size; a bigger
               # jump is NanoTrack hopping to a neighbour (crowd test 2026-10-24; was 1.25 / 50 px)
               dx<=max(20.,(old[2]-old[0])*.75) and dy<=max(20.,(old[3]-old[1])*.75) and
               .4<=w/max(1.,old[2]-old[0])<=2.5 and .4<=h/max(1.,old[3]-old[1])<=2.5 and
               0<=self.clock()-frame_t<=.4)
        self.last_nano_frame_id=frame_id
        if not valid:
            if at_edge and self.left_frame_t is None:self.left_frame_t=self.clock()
            self.nano_ready=False;self.state='COAST'
            if self.uncertain_since is None:self.uncertain_since=self.clock()
            self.good_after_failure=0
            self.reason=f'Nano uncertain (score {self.score:.2f}); stop requested';return
        self.box=candidate;self.last_position_t=frame_t;self.left_frame_t=None
        self.box_by_frame[frame_id]=list(candidate)
        if self.uncertain_since is not None:
            self.good_after_failure+=1
            if self.good_after_failure<2:
                self.reason='Nano recovering; waiting for second valid frame';return
            self.uncertain_since=None
        self.nano_ready=True;self.state='LOCKED'
        self.reason=f'Nano {self.score:.2f}; detector age {max(0,self.clock()-self.last_detection_t):.2f}s'

    def _visible(self,box):
        w,h=max(1.,box[2]-box[0]),max(1.,box[3]-box[1])
        vw=max(0.,min(box[2],self.width)-max(box[0],0.));vh=max(0.,min(box[3],self.height)-max(box[1],0.))
        return vw*vh/(w*h)

    def _inside(self,box):
        cx,cy=(box[0]+box[2])/2,(box[1]+box[3])/2
        return 0<=cx<self.width and 0<=cy<self.height

    def _at_edge(self,box):
        mx,my=self.edge_margin*self.width,self.edge_margin*self.height
        return box[0]<=mx or box[1]<=my or box[2]>=self.width-mx or box[3]>=self.height-my

    def _exiting(self):
        return self.last_detection_box is not None and self._at_edge(self.last_detection_box)

    def _release(self,why):
        self.log.info('Released %s ID %s: %s; selecting the next target',
                      self.names.get(self.selected_cls,self.selected_cls),self.selected_id,why)
        self.selected_id=self.oc_id=self.selected_cls=self.selected_generation=None
        self.nano_ready=self.nano_initialized=False;self.motion_block=None
        self.uncertain_since=None;self.recovery_id=None;self.recovery_hits=0
        self.left_frame_t=None;self.box=None;self.box_by_frame={};self.block_since=None
        self.state='SEARCHING';self.reason=why;self.releases+=1

    def _seed(self,frame,frame_id,frame_t,box,history):
        started=time.perf_counter()
        if not 0<=self.clock()-frame_t<=.4:return False
        h,w=frame.shape[:2];x1,y1,x2,y2=box
        x1=max(0,int(x1));y1=max(0,int(y1));x2=min(w,int(x2));y2=min(h,int(y2))
        if x2-x1<8 or y2-y1<12:return False
        # init() replaces the template; keep the already-loaded ONNX networks.
        self.nano_ready=self.nano_initialized=False
        self.nano.init(frame,(x1,y1,x2-x1,y2-y1))
        self.seed_count+=1;self.replay_count=0
        self.box=[float(x1),float(y1),float(x2),float(y2)]
        self.nano_ready=self.nano_initialized=True
        self.uncertain_since=None;self.good_after_failure=0;self.left_frame_t=None
        self.last_nano_frame_id=frame_id;self.last_position_t=frame_t
        self.box_by_frame={frame_id:list(self.box)}
        for fid,ft,f in history:
            if fid<=frame_id:continue
            # Bound catch-up work; never skip silently and drive on an old box.
            if self.replay_count>=8 or time.perf_counter()-started>.060:
                self.nano_ready=self.nano_initialized=False
                self.reason='Nano catch-up budget exceeded; waiting for fresh detector'
                break
            self._nano_step(f,fid,ft);self.replay_count+=1
            if not self.nano_initialized:break
        self.seed_ms=(time.perf_counter()-started)*1000.
        if not 0<=self.clock()-self.last_position_t<=.4:
            self.nano_ready=self.nano_initialized=False
        return self.nano_ready

    CMC_SIZE=(160,90)     # camera-motion estimate on a small grey copy (~1 ms on a Pi 5)

    def _camera_shift(self,frame):
        """Whole-image shift (px, full frame) since the previous detector frame. A gimbal turn moves
        every box sideways at once; OC-SORT predicts as if the camera were still, so small boxes
        lost their match and the target got a new ID (Fast gimbal profile, Pi 5, 2026-10-26)."""
        try:
            grey=frame if frame.ndim==2 else cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            small=cv2.resize(grey,self.CMC_SIZE,interpolation=cv2.INTER_AREA).astype(np.float32)
        except cv2.error:
            return 0.,0.
        prev,self.cmc_prev=self.cmc_prev,small
        if prev is None:return 0.,0.
        window=getattr(self,'cmc_window',None)
        if window is None:window=self.cmc_window=cv2.createHanningWindow(self.CMC_SIZE,cv2.CV_32F)
        (dx,dy),response=cv2.phaseCorrelate(prev,small,window)
        if response<.05 or not (math.isfinite(dx) and math.isfinite(dy)):return 0.,0.   # flat / unsure
        return dx*frame.shape[1]/self.CMC_SIZE[0],dy*frame.shape[0]/self.CMC_SIZE[1]

    def _shift_tracks(self,dx,dy):
        """Move every OC-SORT track (state, observations, frozen copy) by the camera shift, each
        array once (observations and history share arrays)."""
        if abs(dx)<1. and abs(dy)<1.:return
        seen=set()
        def corners(box):
            if box is None or id(box) in seen or np.asarray(box).sum()<0:return   # -1 placeholder
            seen.add(id(box));box[0]+=dx;box[2]+=dx;box[1]+=dy;box[3]+=dy
        def centre(z):
            if z is None or id(z) in seen:return
            seen.add(id(z));z[0]+=dx;z[1]+=dy
        for oc in self.ocs.values():
            for trk in oc.trackers:
                kf=trk.kf
                for z in (kf.x,getattr(kf,'x_prior',None),getattr(kf,'x_post',None)):centre(z)
                for z in getattr(kf,'history_obs',[]):centre(z)
                saved=getattr(kf,'attr_saved',None)
                if saved:
                    for key in ('x','x_prior','x_post'):centre(saved.get(key))
                    for z in saved.get('history_obs',[]):centre(z)
                corners(trk.last_observation)
                for box in trk.observations.values():corners(box)
                for box in trk.history_observations:corners(box)

    def _observe(self,dets,labels,frame_t,shape):
        """OC-SORT per class; confirmed rows matched back to their detection box."""
        labels=[0]*len(dets) if labels is None else [int(c) for c in labels]
        if len(labels)!=len(dets):raise ValueError('one class label per detection required')
        observed=[]
        for cls in sorted(set(labels)|set(self.ocs)):
            own=[d for d,c in zip(dets,labels) if c==cls]
            oc=self.ocs.get(cls)
            if oc is None:oc=self.ocs[cls]=self._make_oc()
            rows=oc.update(np.asarray(own,dtype=np.float64).reshape(-1,5),shape,shape)
            confirmed={int(trk.id)+1 for trk in oc.trackers if getattr(trk,'hits',0)>=1}
            for r in rows:
                if int(r[4]) not in confirmed:continue
                idx=int(np.argmax([iou(r[:4],d[:4]) for d in own])) if own else -1
                if idx<0 or iou(r[:4],own[idx][:4])<.15:continue
                observed.append({'id':cls*CLASS_ID_STRIDE+int(r[4]),'box':list(map(float,own[idx][:4])),
                                 'conf':float(own[idx][4]),'observed_t':frame_t,'cls':cls,
                                 'label':self.names.get(cls,str(cls))})
        return observed

    def _on_result(self,frame_id,frame_t,dets,source_frame,history,current_time,labels=None):
        if source_frame is None or frame_id<=self.last_processed_result:return
        if not 0<=current_time-frame_t<=.6:return
        if math.isfinite(self.last_result_t) and frame_t-self.last_result_t>2.:self._new_oc()
        self.last_processed_result=frame_id;self.last_result_t=frame_t
        h,w=source_frame.shape[:2]
        if self.camera_motion:
            self.last_shift=self._camera_shift(source_frame)
            self._shift_tracks(*self.last_shift)
        observed=self._observe(list(dets),labels,frame_t,(h,w))
        self.rows=observed
        if self.selected_id is None:
            lockable=[r for r in observed if self.lock_classes is None or r['cls'] in self.lock_classes]
            if not lockable:return
            target=min(lockable,key=lambda r:((r['box'][0]+r['box'][2]-w)/2)**2+
                                              ((r['box'][1]+r['box'][3]-h)/2)**2)
            if self._seed(source_frame,frame_id,frame_t,target['box'],history):
                self.selected_id=self.oc_id=target['id'];self.selected_generation=self.oc_generation
                self.selected_cls=target['cls']
                self.last_detection_t=frame_t
                self.last_detection_box=target['box'];self.state='LOCKED'
                self.reason='OC-SORT confirmed; Nano initialized'
                self.log.info('Selected %s, OC-SORT ID %d; NanoTrack initialized',
                              target['label'],self.selected_id)
            return
        # The selection is one object of one class: other classes never re-associate with it.
        observed=[r for r in observed if r['cls']==self.selected_cls]
        if not observed:self.recovery_id=None;self.recovery_hits=0;return
        reference=self.box_by_frame.get(frame_id)
        if reference is None:
            # Detection may be several frames old. Use the most recent Nano
            # observation at or before its source frame, never today's box.
            prior=[fid for fid in self.box_by_frame if fid<=frame_id]
            reference=self.box_by_frame[max(prior)] if prior else self.last_detection_box
        ordered=sorted(observed,key=lambda r:iou(reference,r['box']),reverse=True)
        best=ordered[0];overlap=iou(reference,best['box'])
        second=iou(reference,ordered[1]['box']) if len(ordered)>1 else 0.
        self.last_overlaps=(round(overlap,3),round(second,3))   # diagnostics (log / tests)
        # Ambiguous only when no detection clearly wins: a neighbour touching the box (crowd) no
        # longer stops the gimbal if the target's own detection overlaps at least twice as much.
        if overlap<.08 and not self._exiting():
            # The detector missed our target but saw others elsewhere: a miss, not a different
            # person (crowd test: this was every "ambiguous" stop). No confirmation is counted, so
            # NanoTrack may drive only within detector_grace (no long hold), and a detection that
            # overlaps the target later is still checked as usual.
            self.miss_with_others_t=current_time
            self.recovery_id=None;self.recovery_hits=0;return
        if overlap<.08 or (second>=.08 and second>=self.ambiguity_ratio*overlap):
            self.motion_block='ambiguous/disagreeing detector candidate; selection held, movement stopped'
            self.reason=self.motion_block
            self.recovery_id=None;self.recovery_hits=0;return
        if (self.oc_generation==self.selected_generation and
                best['id']==self.oc_id and overlap>=.10):
            self.last_detection_t=frame_t;self.last_detection_box=best['box']
            self.motion_block=None
            self.recovery_id=None;self.recovery_hits=0
            if not self.nano_ready or overlap<.5:
                self._seed(source_frame,frame_id,frame_t,best['box'],history)
            return
        if self._exiting():
            # Last confirmed at the frame edge: the target is leaving. A "new ID" here is almost
            # always the neighbour NanoTrack slid onto (laptop pan test, 2026-10-24): never re-attach.
            self.motion_block='target leaving the frame; not re-attaching to another detection'
            self.reason=self.motion_block;self.recovery_id=None;self.recovery_hits=0;return
        self.motion_block='OC-SORT identity changed; waiting for three matching detections'
        if current_time-self.last_detection_t>self.reconnect_window:return
        if best['id']!=self.recovery_id:self.recovery_id=best['id'];self.recovery_hits=0
        self.recovery_hits+=1
        if self.recovery_hits>=3 and self._seed(source_frame,frame_id,frame_t,best['box'],history):
            self.oc_id=best['id'];self.last_detection_t=frame_t;self.last_detection_box=best['box']
            self.selected_generation=self.oc_generation
            self.motion_block=None
            self.recovery_id=None;self.recovery_hits=0
            self.log.info('Re-associated selected person with OC-SORT ID %d',self.oc_id)

    def process(self,frame,frame_id,frame_t,result=None,frame_history=()):
        self.height,self.width=frame.shape[:2]
        if result is not None:
            rid,rt,dets,source_frame=result[:4]
            labels=result[4] if len(result)>4 else None
            self._on_result(rid,rt,dets,source_frame,frame_history,self.clock(),labels)
        self._nano_step(frame,frame_id,frame_t);now=self.clock()
        self.box_by_frame={fid:box for fid,box in self.box_by_frame.items()
                           if fid>=frame_id-48}
        detector_age=now-self.last_detection_t
        # Detector quiet: a confident NanoTrack well inside the frame keeps the lock moving.
        holding=(self.selected_id is not None and self.detector_grace<detector_age<=self.nano_hold_s and
                 self.nano_ready and self.score>=self.hold_score and self.motion_block is None and
                 self.miss_with_others_t<self.last_detection_t and
                 self.box is not None and not self._at_edge(self.box))
        if holding:
            self.reason=f'Nano holding lock {self.score:.2f}; detector quiet {detector_age:.1f}s'
        elif detector_age>self.detector_grace:
            self.nano_ready=self.nano_initialized=False
            if self.selected_id is not None:
                self.state='COAST';self.reason='detector confirmation expired; selection held, movement stopped'
        exiting=self.selected_id is not None and (self._exiting() or (self.box is not None and self._at_edge(self.box)))
        if exiting and detector_age>.3:
            holding=False   # at the edge only fresh detections may move the gimbal
        if self.selected_id is not None:
            seen=max(self.last_detection_t,self.last_position_t)
            if ((self.left_frame_t is not None and now-self.left_frame_t>=self.edge_release_s or exiting)
                    and detector_age>=self.edge_release_s):
                self._release('target left the frame')
            elif now-seen>self.release_after_s:
                self._release('target not seen for %.0f s' % self.release_after_s)
            elif self.motion_block is not None:
                # Identity in doubt (ambiguous / new ID) for too long: NanoTrack may have slid onto
                # someone else meanwhile (laptop crowd test, 2026-10-24), so never re-attach late.
                # Start again from the target nearest the centre (where the gimbal kept it).
                if self.block_since is None:self.block_since=now
                elif now-self.block_since>self.doubt_release_s:
                    self._release('identity in doubt for %.1f s' % self.doubt_release_s)
            else:
                self.block_since=None
        self.visual_records=[dict(r,box=list(map(int,r['box'])),observed_box=r['box'],
                                  state_time=r['observed_t'],velocity=(0.,0.),history=[],age=2,
                                  coast=False,lock_state='TRACK')
                             for r in self.rows if now-r['observed_t']<=.5]
        target=None
        if (self.selected_id is not None and self.nano_ready and self.motion_block is None and
                0<=now-self.last_position_t<=.4 and (0<=detector_age<=self.detector_grace or holding) and
                not (exiting and detector_age>.3)):
            target=dict(id=self.selected_id,box=list(map(int,self.box)),observed_box=list(self.box),
                        conf=self.score,observed_t=self.last_position_t,state_time=self.last_position_t,
                        velocity=(0.,0.),history=[],age=2,coast=False,lock_state='LOCKED',source='OC-SORT + NanoTrack',
                        cls=self.selected_cls,label=self.names.get(self.selected_cls,str(self.selected_cls)))
            self.visual_records=[r for r in self.visual_records if r['id']!=self.oc_id]
            self.visual_records.append(target)
        if self.selected_id is not None and target is None:
            self.state='COAST'
            if self.motion_block:self.reason=self.motion_block
        elif target is not None:self.state='LOCKED'
        status=(self.state,self.selected_id,self.motion_block,self.nano_initialized)
        if status!=self.last_status:
            self.log.info('Lock state=%s selected=%s oc_id=%s nano_score=%.3f '
                          'detector_age=%.3fs position_age=%.3fs reason=%s',
                          self.state,self.selected_id,self.oc_id,self.score,
                          now-self.last_detection_t,now-self.last_position_t,self.reason)
            self.last_status=status
        return self.visual_records,target,self.state


from vendor.ocsort.ocsort import OCSort as OSortNew
