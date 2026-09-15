"""Owned simulation feedback only; never used by the physical executor."""
import os
import queue
import time
import numpy as np


class MappedRecovery:
    def __init__(self, simulation, controller):
        self.simulation=simulation
        self.controller=controller
        self.actions=queue.SimpleQueue()
        self.settled_since=None
        self.status='Waiting: focus this window; C calibrates, S starts'
        self.output_buffer=b''
        self.home=simulation.targets[:14].copy()
        self.home_phase=None
        self.phase_started=0
        self.home_settled=None
        self.accept_after=0
        self.ack_started=0
        ids=[simulation.model.joint(f'Joint{i}_{s}').id for s in ('L','R') for i in range(1,8)]
        self.velocity_indices=simulation.model.jnt_dofadr[ids]

    def key(self, key):
        if key in (67,99,83,115,80,112,82,114,72,104):
            self.actions.put(chr(key).lower())

    def accept_frame(self, frame):
        return self.home_phase is None and frame.timestamp_ns>self.accept_after

    def send(self, message):
        data=message.encode('ascii')
        if os.write(self.controller.stdin.fileno(),data)!=len(data):
            raise RuntimeError('incomplete simulation recovery request')

    def notice(self, text):
        self.status='MAPPED: '+text
        print(self.status,flush=True)

    def native_status(self, text):
        self.status=text
        if self.home_phase=='rearming' and text.startswith('MAPPED: R accepted'):
            self.home_phase='ready'
            self.notice('Home reached; rearmed. Press S to resume (no automatic start)')
        elif self.home_phase=='rearming' and text.startswith('MAPPED: R rejected'):
            self.home_phase='home_hold'
        elif self.home_phase=='starting':
            if text in ('MAPPED: resumed','MAPPED: simulation input started'):
                self.home_phase=None
                self.accept_after=time.monotonic_ns()
            elif text.startswith('MAPPED: S rejected'):
                self.home_phase='ready'

    def home_update(self, now, position, velocity):
        phase=self.home_phase
        if phase in ('rearming','starting') and now-self.ack_started>2_000_000_000:
            self.send('p')
            self.home_phase='home_hold'
            self.notice('Native acknowledgement timeout: holding; manual R required')
            return
        if phase not in ('settling','homing'): return
        if now-self.phase_started>60_000_000_000:
            self.home_phase='home_hold'
            self.notice('Home timeout: holding; inspect simulation, R remains manual')
            return
        if phase=='settling':
            if self.settled_since is None or now-self.settled_since<300_000_000: return
            self.home_start=self.simulation.targets[:14].copy()
            distance=float(max(abs(self.home-self.home_start)))
            # Quintic rest-to-rest; <=0.35 rad/s and <=0.5 rad/s^2.
            self.home_duration=max(2.,1.875*distance/.35,(5.774*distance/.5)**.5)
            self.home_motion_started=now
            self.home_phase='homing'
            self.notice('HOMING: S/C/R rejected; P aborts and holds')
        u=min(1.,max(0.,(now-self.home_motion_started)/1e9/self.home_duration))
        blend=u*u*u*(10+u*(-15+6*u))
        self.simulation.set_simulation_arm_targets(self.home_start+blend*(self.home-self.home_start))
        arrived=u==1. and np.isfinite(position).all() and np.isfinite(velocity).all() and max(abs(position-self.home))<=.01 and max(abs(velocity))<=.03
        if not arrived: self.home_settled=None
        elif self.home_settled is None: self.home_settled=now
        elif now-self.home_settled>=300_000_000:
            self.home_phase='rearming'
            self.ack_started=now
            self.send('R '+str(now)+' '+' '.join(format(float(v),'.17g') for v in (*position,*velocity))+'\n')
            self.notice('Home settled; waiting for native rearm acknowledgement')

    def update(self):
        stream=getattr(self.controller,'stdout',None)
        if stream is not None:
            for _ in range(4):
                try: chunk=os.read(stream.fileno(),4096)
                except BlockingIOError: break
                if not chunk: break
                self.output_buffer+=chunk
                while b'\n' in self.output_buffer:
                    line,self.output_buffer=self.output_buffer.split(b'\n',1)
                    text=line.decode('utf-8',errors='replace')
                    print(text,flush=True)
                    if text.startswith('MAPPED'): self.native_status(text)
                if len(self.output_buffer)>8192:
                    self.output_buffer=b''
        now=time.monotonic_ns()
        velocity=self.simulation.data.qvel[self.velocity_indices].copy()
        position=self.simulation.data.qpos[self.simulation.qpos_indices[:14]].copy()
        settled=np.isfinite(velocity).all() and np.isfinite(position).all() and max(abs(velocity))<=.03
        if not settled: self.settled_since=None
        elif self.settled_since is None: self.settled_since=now
        self.home_update(now,position,velocity)
        for _ in range(32):
            try: action=self.actions.get_nowait()
            except queue.Empty: break
            if action=='h':
                if self.home_phase in ('settling','homing','rearming','starting'):
                    self.notice('H ignored: transition in progress'); continue
                self.send('p')
                self.home_phase='settling'; self.phase_started=now
                self.home_settled=None; self.settled_since=None
                self.notice('H: following stopped; waiting for rest before smooth Home')
                continue
            if action=='p' and self.home_phase is not None:
                self.home_phase='home_hold'
                self.notice('Home aborted: holding; settle then R and S')
            elif self.home_phase in ('settling','homing','rearming','starting'):
                self.notice(action.upper()+' rejected: Home/rearm transition in progress')
                continue
            elif self.home_phase is not None and action=='s':
                if self.home_phase!='ready':
                    self.notice('S rejected: settle then R first'); continue
                self.home_phase='starting'
                self.ack_started=now
            if action=='r':
                if self.settled_since is None or now-self.settled_since<300_000_000:
                    self.status='MAPPED: R rejected: simulation must be stationary for 0.3 s'
                    print(self.status,flush=True)
                    continue
                message='R '+str(now)+' '+' '.join(format(float(v),'.17g') for v in (*position,*velocity))+'\n'
                if self.home_phase is not None:
                    self.home_phase='rearming'; self.ack_started=now
            else: message=action
            self.send(message)
