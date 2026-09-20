"""Read-only runtime checks with fake ELF bytes; no SDK main or devices."""
import io
from pathlib import Path
import subprocess
from unittest.mock import patch

import pytest

from manus.start_hand_teleop import check_collector


COLLECTOR = Path('/fixture/manus_raw')
SDK = Path('/fixture/libManusSDK_Integrated.so')
GOOD = f'libManusSDK_Integrated.so => {SDK} (0x1000)\n'


def test_no_execute_permission():
    with patch('manus.start_hand_teleop.os.access', return_value=False), \
         patch('manus.start_hand_teleop.subprocess.run') as run:
        with pytest.raises(ValueError, match='not executable'):
            check_collector(COLLECTOR, SDK)
        run.assert_not_called()


def test_lfs_pointer_rejected_without_loader():
    with patch('manus.start_hand_teleop.os.access', return_value=True), \
         patch.object(Path, 'open', side_effect=[io.BytesIO(b'\x7fELF'), io.BytesIO(b'version https://git-lfs')]), \
         patch('manus.start_hand_teleop.subprocess.run') as run:
        with pytest.raises(ValueError, match='not an ELF'):
            check_collector(COLLECTOR, SDK)
        run.assert_not_called()


@pytest.mark.parametrize('output,code,error', [
    (GOOD, 0, None),
    ('libusb.so => not found', 0, 'dependencies unavailable'),
    ('version GLIBC_999 not found', 1, 'dependencies unavailable'),
    ('libManusSDK_Integrated.so => /other/sdk.so (0x1000)', 0, 'bundled Manus SDK'),
    ('', 0, 'bundled Manus SDK'),
])
def test_loader_results(output, code, error):
    with patch('manus.start_hand_teleop.os.access', return_value=True), \
         patch.object(Path, 'open', side_effect=lambda *_: io.BytesIO(b'\x7fELF')), \
         patch('manus.start_hand_teleop.subprocess.run',
               return_value=subprocess.CompletedProcess([], code, output, '')) as run:
        if error:
            with pytest.raises(RuntimeError, match=error):
                check_collector(COLLECTOR, SDK)
        else:
            check_collector(COLLECTOR, SDK)
        assert run.call_args.args[0] == ['ldd', str(COLLECTOR)]
        assert run.call_args.kwargs['timeout'] == 5


def test_loader_timeout():
    with patch('manus.start_hand_teleop.os.access', return_value=True), \
         patch.object(Path, 'open', side_effect=lambda *_: io.BytesIO(b'\x7fELF')), \
         patch('manus.start_hand_teleop.subprocess.run', side_effect=subprocess.TimeoutExpired('ldd', 5)):
        with pytest.raises(RuntimeError, match='timed out'):
            check_collector(COLLECTOR, SDK)
