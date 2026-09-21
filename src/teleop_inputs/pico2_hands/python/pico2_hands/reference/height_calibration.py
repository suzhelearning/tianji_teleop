"""Read-only horizontal model reference and bounded human height sampling."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from .control_geometry import IK_TCP_TO_HAND_CONTROL_TRANSLATION_M


def horizontal_reference(urdf=None):
    """FK at q=0 (horizontal lateral extension), never a robot command."""
    path = Path(urdf) if urdf else Path(__file__).resolve().parents[1] / 'native/models/marvin_m6_s_ccs_696_v4.urdf'
    joints = ET.parse(path).getroot().findall('joint')
    by_child = {j.find('child').get('link'): j for j in joints}
    cache = {}
    def fk(link, visiting=()):
        if link in visiting:
            raise ValueError('cyclic URDF')
        if link not in cache:
            matrix = np.eye(4)
            if link in by_child:
                joint = by_child[link]
                origin = joint.find('origin')
                if origin is not None:
                    matrix[:3,:3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy','0 0 0').split()]).as_matrix()
                    matrix[:3,3] = [float(v) for v in origin.get('xyz','0 0 0').split()]
                matrix = fk(joint.find('parent').get('link'), (*visiting,link)) @ matrix
            cache[link] = matrix
        return cache[link]
    result = {}
    for side, suffix in [('left','L'), ('right','R')]:
        names = [f'Base_{suffix}', f'Link1_{suffix}', f'Link3_{suffix}', f'Link5_{suffix}', f'TCP_Link_{suffix}']
        if any(n not in by_child for n in names):
            raise ValueError('missing horizontal-reference URDF frames')
        heights = [fk(n)[2,3] for n in names[1:4]]
        if not np.isfinite(heights).all() or np.ptp(heights) > 1e-4:
            raise ValueError('q=0 is not a horizontal arm reference in this model')
        base = fk(names[0])
        tcp = fk(names[-1])
        control_world = tcp[:3,3] + tcp[:3,:3] @ IK_TCP_TO_HAND_CONTROL_TRANSLATION_M
        if abs(control_world[2]-heights[0]) > 1e-4:
            raise ValueError('q=0 control point is not horizontal')
        result[side] = {
            'control_position_base_m': base[:3,:3].T @ (control_world-base[:3,3]),
            'world_up_base': base[:3,:3].T @ [0,0,1],
        }
    return result


class HeightCalibration:
    """Collect distinct fresh frames; unsuccessful attempts retain old result."""
    duration_ns = 2_000_000_000
    gap_ns = 250_000_000

    def __init__(self, sides):
        self.sides = tuple(sides)
        self.state = 'uncalibrated'
        self.error = None
        self.means = None
        self.samples = {s: [] for s in self.sides}

    def begin(self, now):
        self.started = now
        self.state, self.error = 'collecting', None
        self.samples = {s: [] for s in self.sides}

    def fail(self, reason):
        self.state, self.error = 'failed', reason

    def add(self, observation, now):
        if self.state != 'collecting':
            return
        side = observation.side
        stamp = observation.received_timestamp_ns
        if side not in self.samples:
            self.fail('unexpected calibration side')
        elif not observation.valid or observation.pose is None:
            self.fail('wrist tracking lost during calibration')
        elif not 0 <= now-stamp <= self.gap_ns:
            self.fail('stale calibration input')
        elif stamp >= self.started:
            previous = self.samples[side]
            if stamp - (previous[-1][0] if previous else self.started) > self.gap_ns:
                self.fail('calibration input gap exceeds 0.25 s')
                return
            if previous and stamp <= previous[-1][0]:
                return  # Repeated receipt timestamp is not a fresh sample.
            previous.append((stamp, observation.pose[:3].copy()))

    def tick(self, now):
        if self.state != 'collecting':
            return None
        if any(now-(v[-1][0] if v else self.started) > self.gap_ns for v in self.samples.values()):
            self.fail('calibration input gap exceeds 0.25 s')
            return None
        if now-self.started < self.duration_ns:
            return None
        means = {}
        for side, values in self.samples.items():
            if len(values) < 30 or values[-1][0]-values[0][0] < 1_500_000_000:
                self.fail('insufficient distinct calibration frames')
                return None
            positions = np.array([v for _,v in values])
            if not np.isfinite(positions).all() or np.max(np.ptp(positions, axis=0)) > .06:
                self.fail('wrist moved too much; hold horizontal pose steadily')
                return None
            means[side] = float(np.mean(positions[:,2]))
        self.state = 'sampled'
        return means

    def accept(self, means):
        self.means, self.state, self.error = dict(means), 'calibrated', None

    def status(self):
        return {'state': self.state, 'error': self.error,
                'human_height_m': self.means,
                'samples': {s: len(v) for s,v in self.samples.items()}}
