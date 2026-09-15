"""Read-only mapped-palm IK target overlay. Never a joint command receiver."""
import math
import socket
import struct
import time
import zlib

WIRE=struct.Struct('<4sBBHQQ14dI')


def decode(packet, now_ns):
    if len(packet)!=WIRE.size:
        raise ValueError('invalid target observation size')
    magic,version,valid,size,sequence,stamp,*values=WIRE.unpack(packet)
    crc=values.pop()
    if (magic!=b'MPT1' or version!=1 or valid not in (0,1) or size!=WIRE.size
            or sequence==0 or stamp==0 or not 0<=now_ns-stamp<=150_000_000
            or crc!=zlib.crc32(packet[:-4]) or not all(math.isfinite(x) for x in values)):
        raise ValueError('invalid or stale target observation')
    for offset in (3,10):
        if abs(sum(x*x for x in values[offset:offset+4])-1)>1e-5:
            raise ValueError('target orientation is not a unit quaternion')
    return sequence,stamp,bool(valid),values


class TargetOverlay:
    def __init__(self, simulation):
        self.simulation=simulation
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1',0)); self.socket.setblocking(False)
        self.port=self.socket.getsockname()[1]
        self.latest=None
        self.sequence=0
        self.last_poll_ns=0
        model=simulation.model
        self.bodies=[int(model.body(name).id) for name in ('target_L','target_R')]
        if any(model.body_mocapid[body]<0 for body in self.bodies):
            self.socket.close()
            raise ValueError('target observation requires dedicated mocap bodies')
        self.geoms=[int(model.geom(name).id) for name in ('target_geom_L','target_geom_R')]
        self.sites=[int(model.site(name).id) for name in ('target_site_L','target_site_R')]
        self.geom_rgba=[model.geom_rgba[i].copy() for i in self.geoms]
        self.site_rgba=[model.site_rgba[i].copy() for i in self.sites]
        self.update()

    def update(self):
        now=time.monotonic_ns()
        if now-self.last_poll_ns<15_000_000:
            return
        self.last_poll_ns=now
        for _ in range(64):
            try: data=self.socket.recv(141)
            except BlockingIOError: break
            try: row=decode(data,now)
            except ValueError: continue
            if row[0]>self.sequence:
                self.sequence=row[0]; self.latest=row
        visible=self.latest is not None and self.latest[2] and 0<=now-self.latest[1]<=150_000_000
        model,data=self.simulation.model,self.simulation.data
        for side,body in enumerate(self.bodies):
            model.geom_rgba[self.geoms[side]]=self.geom_rgba[side]
            model.site_rgba[self.sites[side]]=self.site_rgba[side]
            if not visible:
                model.geom_rgba[self.geoms[side],3]=0
                model.site_rgba[self.sites[side],3]=0
                continue
            p=self.latest[3][side*7:side*7+7]
            mocap=int(model.body_mocapid[body])
            data.mocap_pos[mocap]=p[:3]
            data.mocap_quat[mocap]=[p[6],p[3],p[4],p[5]]

    def close(self):
        self.socket.close()
