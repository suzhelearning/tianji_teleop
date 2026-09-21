"""Recording shortcuts layered over the shared operator keyboard.

The raw terminal handling lives in :class:`tianji_runtime.OperatorKeyboard`; this
class only adds the recording meaning of R/S/D and the executor's fault type.
"""
from tianji_runtime import OperatorKeyboard

from tianji_controller.safety import SafetyFault


class CollectionKeyboard:
    def __init__(self, on_key, on_calibrate=None):
        self._on_key = on_key
        self._keyboard = OperatorKeyboard(self._dispatch, on_calibrate)

    def _dispatch(self, key):
        try:
            self._on_key(key)
        except Exception as error:  # noqa: BLE001 - never kill the control loop
            print(f'DATASET KEY FAILED (robot control unchanged): {error}', flush=True)

    def poll_enter(self):
        try:
            return self._keyboard.poll_enter()
        except EOFError as error:
            raise SafetyFault(str(error)) from error

    def close(self):
        self._keyboard.close()
