"""Local-pipe adapter to the SAME DLS/Ruckig controller as VR simulation.

World-frame palm TCP targets, not the legacy V131 Base-frame flange contract.
Only this simulation owner may issue start/Home/hold. No hardware exports.
"""
import numpy as np
from .ik_worker import NativeIkWorker, ROOT


class DlsWorker(NativeIkWorker):
    ready_kind = "pico2_dls_ready"

    def _command(self):
        control = ROOT.parent / "control"
        return [str(control / "build/pico2_dls_worker"),
                str(control / "config/qp_ik_pico_shared_root_dls.yaml"),
                str(control / "models/marvin_m6_wuji2_shared_root_ceres.xml")]

    def command(self, operation, joints, now):
        return self._exchange(operation, joints, np.zeros((2, 7)), 0., now, now)
