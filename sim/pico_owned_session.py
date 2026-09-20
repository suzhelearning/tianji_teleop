"""Supervise a viewer and release only PICO created with this run's token."""
import os
import signal
import subprocess
import sys
import uuid
import time


def terminate_group(child):
    """Bounded escalation, limited to our start_new_session process group."""
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=3)
    except subprocess.TimeoutExpired:
        pass
    finally:
        # A leader may exit before its descendants. Reap the whole owned group.
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return child.wait(timeout=2)


def release_input(argv, root):
    child = subprocess.Popen(argv, cwd=root, start_new_session=True)
    try:
        return child.wait(timeout=20)
    except subprocess.TimeoutExpired:
        terminate_group(child)
        raise


def run_with_owned_pico(command, user, root):
    token = uuid.uuid4().hex
    active = None
    interrupted = 0
    interrupted_at = None
    previous = {}

    def forward(signum, _frame):
        nonlocal interrupted, interrupted_at
        if interrupted_at is None:
            interrupted_at = time.monotonic()
        interrupted = signum
        if active is not None and active.poll() is None:
            try:
                os.killpg(active.pid, signum)
            except ProcessLookupError:
                pass

    def run(argv, env=None):
        nonlocal active
        if interrupted:
            return 128 + interrupted
        active = subprocess.Popen(argv, cwd=root, env=env, start_new_session=True)
        # A signal between the pre-launch check and Popen must also reach child.
        if interrupted:
            forward(interrupted, None)
        try:
            while True:
                if interrupted_at is not None and time.monotonic() - interrupted_at >= 5:
                    print('Simulation child did not stop within 5 seconds; terminating its owned process group. Recording may be incomplete.',
                          file=sys.stderr, flush=True)
                    return terminate_group(active)
                try:
                    return active.wait(timeout=.2)
                except subprocess.TimeoutExpired:
                    continue
        finally:
            if active.poll() is not None:
                active = None

    prefix = ['pixi', 'run', '--locked', '-e', 'tracking', 'ensure-pico-user']
    result = 2
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, forward)
        environment = {**os.environ, 'TIANJI_PICO_SIM_OWNER': token}
        result = run(prefix + ['--user', user], environment)
        if result == 0 and not interrupted:
            print('PICO created by this simulation will stop on exit; reused PICO and Manus remain running.', flush=True)
            result = run(command)
        elif not interrupted:
            print('PICO input startup failed; robot simulation was not started.', file=sys.stderr, flush=True)
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f'Simulation launch failed: {error}', file=sys.stderr, flush=True)
        result = 2
    finally:
        # Viewer/startup has exited before releasing its input. Never use broad --stop.
        try:
            if active is not None:
                raise RuntimeError('Child exit could not be confirmed; PICO retained for manual inspection')
            cleanup = release_input(prefix + ['--release-owner', token], root)
            if cleanup:
                print('Owned PICO cleanup failed; inspect session before stopping manually.', file=sys.stderr)
                if result == 0:
                    result = 2
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            print(f'Owned PICO cleanup failed: {error}', file=sys.stderr)
            if result == 0:
                result = 2
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 128 + interrupted if interrupted else (result if result >= 0 else 128-result)
