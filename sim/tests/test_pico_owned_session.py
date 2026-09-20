import signal
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from sim.pico_owned_session import run_with_owned_pico, terminate_group, release_input


class OwnedPicoTests(unittest.TestCase):
    def test_real_unresponsive_child_is_reaped(self):
        child = subprocess.Popen([sys.executable, '-c',
            'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
            'print("ready", flush=True); time.sleep(60)'],
            start_new_session=True, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'ready')
            self.assertEqual(terminate_group(child), -signal.SIGKILL)
            self.assertEqual(child.poll(), -signal.SIGKILL)
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=5)
            child.stdout.close()

    def execute(self, codes, cleanup_code=0):
        children = [Mock(pid=8000+i, wait=Mock(return_value=code), poll=Mock(return_value=code))
                    for i, code in enumerate(codes)]
        with patch('sim.pico_owned_session.subprocess.Popen', side_effect=children) as start, \
             patch('sim.pico_owned_session.release_input', return_value=cleanup_code) as stop:
            result = run_with_owned_pico(['viewer'], 'person', Path('/project'))
        token = start.call_args_list[0].kwargs['env']['TIANJI_PICO_SIM_OWNER']
        self.assertEqual(stop.call_args.args[0][-2:], ['--release-owner', token])
        return result, start

    def test_normal_exit_releases_token_after_viewer(self):
        result, start = self.execute([0, 0])
        self.assertEqual(result, 0)
        self.assertEqual(start.call_args_list[1].args[0], ['viewer'])

    def test_startup_failure_still_releases_only_token(self):
        result, start = self.execute([2])
        self.assertEqual(result, 2)
        self.assertEqual(start.call_count, 1)

    def test_viewer_error_keeps_failure_status(self):
        self.assertEqual(self.execute([0, 7])[0], 7)

    def test_cleanup_failure_is_reported(self):
        self.assertEqual(self.execute([0, 0], 2)[0], 2)

    def test_sigint_waits_for_child_then_cleans(self):
        handlers = {}
        child = Mock(pid=8001, poll=Mock(return_value=None))
        def install(sig, handler):
            handlers[sig] = handler
            return signal.SIG_DFL
        def wait(**kwargs):
            handlers[signal.SIGINT](signal.SIGINT, None)
            child.poll.return_value = -signal.SIGINT
            return -signal.SIGINT
        child.wait.side_effect = wait
        with patch('sim.pico_owned_session.signal.signal', side_effect=install), \
             patch('sim.pico_owned_session.os.killpg') as kill, \
             patch('sim.pico_owned_session.subprocess.Popen', return_value=child) as start, \
             patch('sim.pico_owned_session.release_input', return_value=0) as stop:
            self.assertEqual(run_with_owned_pico(['viewer'], 'person', Path('/project')), 130)
            kill.assert_called_once_with(8001, signal.SIGINT)
            self.assertEqual(start.call_count, 1)
            stop.assert_called_once()

    def test_group_escalation_is_bounded(self):
        child = Mock(pid=8001)
        child.wait.side_effect = [subprocess.TimeoutExpired('child', 3), -signal.SIGKILL]
        with patch('sim.pico_owned_session.os.killpg') as kill:
            self.assertEqual(terminate_group(child), -signal.SIGKILL)
        self.assertEqual([call.args for call in kill.call_args_list],
                         [(8001, signal.SIGTERM), (8001, signal.SIGKILL)])
        self.assertEqual([call.kwargs for call in child.wait.call_args_list],
                         [{'timeout': 3}, {'timeout': 2}])

    def test_cleanup_timeout_terminates_helper_group(self):
        child = Mock(pid=8001)
        child.wait.side_effect = subprocess.TimeoutExpired('cleanup', 20)
        with patch('sim.pico_owned_session.subprocess.Popen', return_value=child), \
             patch('sim.pico_owned_session.terminate_group') as terminate:
            with self.assertRaises(subprocess.TimeoutExpired):
                release_input(['cleanup'], Path('/project'))
            terminate.assert_called_once_with(child)

    def test_ignored_interrupt_reaches_escalation_and_cleanup(self):
        handlers = {}
        child = Mock(pid=8001, poll=Mock(return_value=None))
        def install(sig, handler):
            handlers[sig] = handler
            return signal.SIG_DFL
        def wait(**kwargs):
            handlers[signal.SIGTERM](signal.SIGTERM, None)
            raise subprocess.TimeoutExpired('child', .2)
        child.wait.side_effect = wait
        def terminate(process):
            process.poll.return_value = -signal.SIGKILL
            return -signal.SIGKILL
        with patch('sim.pico_owned_session.signal.signal', side_effect=install), \
             patch('sim.pico_owned_session.time.monotonic', side_effect=[0, 6]), \
             patch('sim.pico_owned_session.os.killpg'), \
             patch('sim.pico_owned_session.subprocess.Popen', return_value=child), \
             patch('sim.pico_owned_session.terminate_group', side_effect=terminate) as escalation, \
             patch('sim.pico_owned_session.release_input', return_value=0) as cleanup:
            self.assertEqual(run_with_owned_pico(['viewer'], 'person', Path('/project')), 143)
            escalation.assert_called_once_with(child)
            cleanup.assert_called_once()
