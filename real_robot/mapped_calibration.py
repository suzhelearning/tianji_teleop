"""Operator calibration gate; never enables devices or fabricates ready."""
import time
from .mapped_events import EventFrame
from .safety import SafetyFault


class CalibrationGuard:
    def __init__(self,receiver):
        self.receiver=receiver
        self.expected_revision=None
        self.lock_pending=False
        self.lock_started=0
        self.lock_epoch=0
        self.lock_sequence=0

    def check(self,frame,now_ns,*,locked=False):
        if not isinstance(frame,EventFrame): raise SafetyFault('calibration event missing')
        if not 0<=now_ns-frame.timestamp_ns<=150_000_000: raise SafetyFault('calibration status stale')
        if self.expected_revision is None or frame.calibration_revision!=self.expected_revision:
            raise SafetyFault('C calibration required for this session')
        if frame.calibration_state not in ((3,) if locked else (2,3)):
            state={0:'required/invalidated',1:'sampling, hold forward 2 seconds',4:'failed; retry C'}.get(frame.calibration_state,'waiting for lock')
            raise SafetyFault('calibration '+state)
        if frame.tracking_epoch!=frame.calibration_epoch or frame.tracking_epoch<=0:
            raise SafetyFault('calibration epoch changed')

    def calibrate(self,gate):
        if gate.armed or self.lock_pending:
            print('C rejected: calibration is locked for enable/execution',flush=True)
            return
        frame=self.receiver.latest
        now=time.monotonic_ns()
        if frame is None or frame.tracking_epoch<=0 or not 0<=now-frame.timestamp_ns<=150_000_000:
            print('C rejected: fresh native/PICO identity required',flush=True)
            return
        if (self.expected_revision is not None and frame.calibration_revision<self.expected_revision) or frame.calibration_state==1:
            print('C ignored: calibration request/sampling already pending',flush=True)
            return
        self.receiver.calibration_command(1,frame)
        self.expected_revision=frame.calibration_revision+1
        print('C: sampling X/Z; hold both arms forward and horizontal for 2 seconds. Motors remain disabled.',flush=True)

    def request_lock(self,frame,now_ns):
        self.check(frame,now_ns)
        self.receiver.calibration_command(2,frame)
        self.lock_pending=True; self.lock_started=now_ns; self.lock_epoch=frame.tracking_epoch
        self.lock_sequence=frame.sequence

    def lock_acknowledged(self,frame,now_ns):
        if not self.lock_pending: return False
        if now_ns-self.lock_started>2_000_000_000: raise SafetyFault('calibration lock timed out')
        if frame is None or frame.tracking_epoch!=self.lock_epoch: raise SafetyFault('epoch changed during calibration lock')
        if frame.sequence<=self.lock_sequence or frame.timestamp_ns<self.lock_started: return False
        if frame.calibration_state!=3: return False
        self.check(frame,now_ns,locked=True)
        return True
