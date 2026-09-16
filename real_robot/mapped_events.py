"""Owned atomic native event/TJRC channel. No hardware SDK imports."""
from dataclasses import dataclass
import secrets
import socket
import struct
import math
from .protocol import CommandFrame, decode_packet, PACKET_SIZE, ARMS_READY
from .safety import SafetyFault

HEADER=struct.Struct('<4sBBH32sQQqQQ')
CALIBRATION=struct.Struct('<QQQ4d')
INPUT_VALID=struct.Struct('<q')
FAULTS={4:'input invalid/stale',5:'IK/output not accepted',6:'tracking epoch reset',
        7:'operator input event',8:'native resynchronization timeout'}


@dataclass(frozen=True)
class EventFrame(CommandFrame):
    event_state: int
    generation: int
    calibration_revision: int = 0
    calibration_epoch: int = 0
    calibration_state: int = 0
    calibration_offsets: tuple = (0.,0.,0.,0.)
    input_valid_ns: int = 0


def decode_event(data, token):
    if len(data) not in (HEADER.size+PACKET_SIZE,HEADER.size+CALIBRATION.size+PACKET_SIZE,
                         HEADER.size+CALIBRATION.size+INPUT_VALID.size+PACKET_SIZE):
        raise SafetyFault('invalid event size')
    magic,version,state,size,identity,seq,tick,stamp,epoch,generation=HEADER.unpack_from(data)
    if magic!=b'MRE1' or version not in (1,2,3) or size!=len(data) or state not in range(1,10 if version==3 else 9) or identity!=token.encode('ascii'):
        raise SafetyFault('invalid event header/identity')
    expected=HEADER.size+PACKET_SIZE+(CALIBRATION.size if version>=2 else 0)+(INPUT_VALID.size if version==3 else 0)
    if len(data)!=expected: raise SafetyFault('invalid versioned event size')
    extra={}
    offset=HEADER.size
    if version>=2:
        revision,cal_epoch,cal_state,*offsets=CALIBRATION.unpack_from(data,offset)
        if cal_state not in range(5) or not all(math.isfinite(v) for v in offsets):
            raise SafetyFault('invalid calibration status')
        extra=dict(calibration_revision=revision,calibration_epoch=cal_epoch,
                   calibration_state=cal_state,calibration_offsets=tuple(offsets))
        offset+=CALIBRATION.size
    if version==3:
        input_valid_ns,=INPUT_VALID.unpack_from(data,offset)
        if input_valid_ns<0 or input_valid_ns>stamp or (state in (1,3,9) and input_valid_ns==0):
            raise SafetyFault('invalid PICO input timestamp')
        extra['input_valid_ns']=input_valid_ns
        offset+=INPUT_VALID.size
    frame=decode_packet(data[offset:])
    if (seq,stamp,epoch)!=(frame.sequence,frame.timestamp_ns,frame.tracking_epoch) or tick!=seq:
        raise SafetyFault('event/command cycle mismatch')
    if (state in (1,3) and not frame.flags&ARMS_READY) or ((state==2 or state>=4) and frame.flags&ARMS_READY):
        raise SafetyFault('inconsistent event/ready flags')
    return EventFrame(**frame.__dict__,event_state=state,generation=generation,**extra)


class EventReceiver:
    def __init__(self,reserved,*,bounded=True,calibration=False,dropout=False):
        self.reserved=reserved
        self.reader,self.writer=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        self.reader.setblocking(False)
        self.token=secrets.token_hex(16)
        self.latest=None
        self.count=0
        self.bounded=bounded
        self.calibration=calibration
        self.dropout=dropout

    @property
    def port(self): return self.reserved.port

    def arguments(self):
        return ['--real-event-fd',str(self.writer.fileno()),'--real-event-token',self.token]+(
            ['--real-resync-bounded'] if self.bounded else [])+(
            ['--real-xz-calibration'] if self.calibration else [])+(
            ['--real-input-hold-300ms'] if self.dropout else [])

    def calibration_command(self,action,frame):
        payload=struct.pack('<4sB3xQQ',b'MRC1',action,frame.calibration_revision,frame.tracking_epoch)
        if self.reader.send(payload)!=len(payload): raise SafetyFault('calibration command incomplete')

    def drain(self,validate=None):
        for _ in range(512):
            try: data=self.reader.recv(4096)
            except BlockingIOError: break
            if not data: raise SafetyFault('native event channel closed')
            frame=decode_event(data,self.token)
            if self.dropout and data[4]!=3: raise SafetyFault('PICO dropout status missing')
            if self.calibration and data[4] not in (2,3): raise SafetyFault('calibration status missing')
            if frame.sequence!=self.count+1 or (self.latest and frame.timestamp_ns<=self.latest.timestamp_ns):
                raise SafetyFault('native event gap/replay')
            if self.latest and frame.generation<self.latest.generation:
                raise SafetyFault('native event generation rollback')
            if self.latest and frame.calibration_revision<self.latest.calibration_revision:
                raise SafetyFault('calibration revision rollback')
            if self.dropout and self.latest and frame.input_valid_ns<self.latest.input_valid_ns:
                raise SafetyFault('PICO input timestamp rollback')
            if validate: validate(frame)
            self.latest=frame; self.count+=1
        return self.latest

    def close(self):
        self.reader.close(); self.writer.close(); self.reserved.close()
