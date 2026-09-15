"""R/S/D operate on real HDF5 episodes with fake acquisition, never robot control."""
import os
from pathlib import Path
import pty
import sys
import termios
import threading
import time

import h5py
import numpy as np
import pytest

from data_collection.dataset import EpisodeWriter
from data_collection.integration import CollectionSession
from data_collection.keyboard import CollectionKeyboard

ROOT = Path(__file__).resolve().parents[2]


class Acquisition:
    def __init__(self, hardware, cameras, writer, **kwargs):
        self.writer = writer
        self.cameras = cameras
        self.recording = False
        self.error = None
        self.episode_error = None
    def start(self):
        pass
    def begin_episode(self, now_ns):
        self.episode_error = None
        self.writer.start(now_ns)
        self.recording = True
    def end_episode(self):
        if self.recording:
            self.episode_error = self.error
        self.writer.stop()
        self.recording = False
    def request_stop(self):
        pass
    def close(self):
        self.end_episode()
    def check(self):
        if self.error:
            raise RuntimeError(self.error)
        self.writer.check()
    def check_episode(self):
        if self.episode_error:
            raise RuntimeError(self.episode_error)
    def publish(self):
        now = time.monotonic_ns()
        self.writer.append_arms(now, (.1,) * 14)
        self.writer.append_hands(now + 1, (.2,) * 40)
        for name in self.cameras:
            self.writer.append_rgb(name, now + 2, np.full((720, 1280, 3), 80, dtype=np.uint8))


def wait_state(session, state):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if session.state == state:
            return
        time.sleep(.002)
    raise AssertionError(f'expected {state}, got {session.state}')


@pytest.fixture
def session(tmp_path, monkeypatch):
    monkeypatch.setattr('data_collection.runtime.validate_camera_profiles', lambda cameras: {})
    monkeypatch.setattr('data_collection.runtime.CollectionRuntime', Acquisition)
    instance = CollectionSession(tmp_path, 'pick', ROOT / 'collection_config.json',
        ROOT / 'control/models/marvin_m6_wuji2.xml')
    instance.start({})
    yield instance
    instance.finish()


def test_record_save_discard_and_exit_are_independent_segments(session):
    session.command('r', 'READY')
    assert session.state == 'IDLE'
    assert not session.runtime.recording
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    session.command('s', 'TELEOP')
    wait_state(session, 'IDLE')
    saved = session.saved_paths[0]
    with h5py.File(saved) as episode:
        assert set(episode) == {'observations', 'images'}
        assert 'action_type' not in episode.attrs
        assert bool(episode.attrs['success'])
        assert episode['observations/arms/qpos'].shape == (1, 14)
        assert episode['observations/hands/qpos'].shape == (1, 40)
        assert set(episode['images']) == {'top', 'left_wrist', 'right_wrist'}
        assert episode['images/right_wrist/rgb'].shape == (1, 720, 1280, 3)
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    session.command('d', 'TELEOP')
    wait_state(session, 'IDLE')
    assert saved.exists()
    assert not list(session.dataset_dir.rglob('*.partial.h5'))
    session.command('d', 'TELEOP')  # no active segment must not erase the saved episode
    assert saved.exists()
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    session.finish()  # no S: keep partial, never silently mark success
    assert saved.exists()
    partials = list(session.dataset_dir.rglob('*.partial.h5'))
    assert len(partials) == 1
    with h5py.File(partials[0]) as episode:
        assert 'success' not in episode.attrs


def test_save_key_returns_while_the_background_writer_is_blocked(session, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = EpisodeWriter.finish
    def delayed_finish(writer, success):
        entered.set()
        if not release.wait(5):
            raise RuntimeError('test did not release writer')
        return original(writer, success)
    monkeypatch.setattr(EpisodeWriter, 'finish', delayed_finish)
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    caller = threading.Thread(target=session.command, args=('s', 'TELEOP'))
    try:
        caller.start()
        caller.join(.5)
        assert not caller.is_alive(), 'S blocked the control/keyboard caller'
        assert entered.wait(2)
        assert session.state == 'SAVING'
        session.check()  # control loop can keep checking while HDF5 finalizes
    finally:
        release.set()
        caller.join(2)
    wait_state(session, 'IDLE')
    assert len(session.saved_paths) == 1


def test_capture_fault_preserves_partial_without_raising_to_teleop(session):
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    session.runtime.error = 'camera disconnected'
    session.check()
    wait_state(session, 'IDLE')
    assert session.saved_paths == []
    assert len(list(session.dataset_dir.rglob('*.partial.h5'))) == 1


@pytest.mark.parametrize('fault_before_stop', [True, False])
def test_save_uses_fault_at_recording_cutoff(session, monkeypatch, fault_before_stop):
    entered, release = threading.Event(), threading.Event()
    original = session._finish_recording

    def delayed_save(operation):
        entered.set()
        if not release.wait(5):
            raise RuntimeError('save was not released')
        return original(operation)

    monkeypatch.setattr(session, '_finish_recording', delayed_save)
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    if fault_before_stop:
        session.runtime.error = 'camera disconnected'
    session.command('s', 'TELEOP')
    try:
        assert entered.wait(2)
        session.runtime.error = 'camera disconnected'
    finally:
        release.set()
    wait_state(session, 'IDLE')
    if fault_before_stop:
        assert session.saved_paths == []
        partial, = session.dataset_dir.rglob('*.partial.h5')
        with h5py.File(partial) as episode:
            assert 'success' not in episode.attrs
    else:
        saved, = session.saved_paths
        with h5py.File(saved) as episode:
            assert bool(episode.attrs['success'])
            assert episode['observations/arms/qpos'].shape == (1, 14)


def test_abort_timeout_retains_writer_until_shutdown(session, monkeypatch):
    from data_collection import dataset

    entered, release = threading.Event(), threading.Event()
    original = EpisodeWriter._handle
    active = []

    def blocked(writer, item):
        active.append(writer)
        entered.set()
        if not release.wait(5):
            raise RuntimeError('writer was not released')
        return original(writer, item)

    monkeypatch.setattr(dataset, '_CLOSE_TIMEOUT_S', .05)
    monkeypatch.setattr(EpisodeWriter, '_handle', blocked)
    session.command('r', 'TELEOP')
    wait_state(session, 'RECORDING')
    session.runtime.publish()
    try:
        assert entered.wait(2)
        session.end_episode()
        wait_state(session, 'FAILED')
        session.command('r', 'TELEOP')
        assert session.state == 'FAILED'
        assert session.saved_paths == []
    finally:
        release.set()
    session.finish()
    assert session.state == 'CLOSED'
    assert not active[0]._thread.is_alive()
    partial, = session.dataset_dir.rglob('*.partial.h5')
    with h5py.File(partial) as episode:
        assert 'success' not in episode.attrs
        assert episode['observations/arms/qpos'].shape == (1, 14)


def test_single_keys_do_not_require_enter_or_generate_robot_enter(monkeypatch):
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    keys = []
    with os.fdopen(os.dup(slave), 'r') as terminal:
        monkeypatch.setattr(sys, 'stdin', terminal)
        keyboard = CollectionKeyboard(keys.append)
        try:
            os.write(master, b'rsd')
            assert keyboard.poll_enter() is False
            assert keys == ['r', 's', 'd']
            os.write(master, b'\n')
            assert keyboard.poll_enter() is True
        finally:
            keyboard.close()
            assert termios.tcgetattr(slave) == original
            os.close(master)
            os.close(slave)
