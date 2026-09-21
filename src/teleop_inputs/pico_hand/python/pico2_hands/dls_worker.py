"""Local-pipe adapter to the SAME DLS/Ruckig controller as VR simulation.

World-frame palm TCP targets, not the legacy V131 Base-frame flange contract.
Only this simulation owner may issue start/Home/hold. No hardware exports.
"""
from pathlib import Path
import tempfile

import numpy as np
import yaml
from tianji_runtime.resources import controller_profile, native_executable

from .ik_worker import NativeIkWorker
from .resources import display_model_path


class DlsWorker(NativeIkWorker):
    ready_kind = "pico2_dls_ready"

    def __init__(self, *, timeout_s=1.0):
        source = controller_profile("qp_ik_pico_shared_root_dls.yaml")
        config = yaml.safe_load(source.read_text())
        key = "pico_ee_dls_kinematics_urdf_path"
        config["controller"][key] = str(display_model_path(Path(config["controller"][key]).name))
        for key in ("input_contract_artifact", "robot_geometry_artifact"):
            config["spark_shared_root"][key] = str((source.parent / config["spark_shared_root"][key]).resolve())
        # Keep the selected installed profile's parameters; only relocate paths.
        # The ready handshake confirms the native process has loaded the YAML.
        with tempfile.TemporaryDirectory(prefix="pico2-dls-") as directory:
            self._profile = Path(directory) / "runtime.yaml"
            self._profile.write_text(yaml.safe_dump(config, sort_keys=False))
            super().__init__(timeout_s=timeout_s)

    def _command(self):
        return [str(native_executable("pico2_dls_worker")), str(self._profile),
                str(display_model_path("marvin_m6_wuji2_shared_root_ceres.xml"))]

    def command(self, operation, joints, now):
        return self._exchange(operation, joints, np.zeros((2, 7)), 0., now, now)
