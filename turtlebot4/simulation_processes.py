"""Find Gazebo servers and reap only servers marked by this launch session."""
from pathlib import Path
import os
import select
import signal


SESSION_ENV = 'TURTLEBOT4_SIM_SESSION'


def gazebo_servers(proc=Path('/proc'), session=None):
    result = []
    for path in proc.glob('[0-9]*'):
        try:
            args = path.joinpath('cmdline').read_bytes().replace(b'\0', b' ').decode().strip()
            if args != 'ign gazebo server' and args != 'gz sim server':
                continue
            if session is not None:
                environment = path.joinpath('environ').read_bytes().split(b'\0')
                if f'{SESSION_ENV}={session}'.encode() not in environment:
                    continue
            result.append(int(path.name))
        except (OSError, UnicodeError):
            continue
    return result


def stop_owned_servers(session):
    # pidfds retain identity if a process exits and its PID is reused. Polling
    # waits for actual exit rather than sleeping or killing by a process name.
    descriptors = {}
    for pid in gazebo_servers(session=session):
        try:
            descriptors[os.pidfd_open(pid)] = pid
        except ProcessLookupError:
            continue
    try:
        for sig, timeout in ((signal.SIGINT, 1.), (signal.SIGTERM, 1.), (signal.SIGKILL, .5)):
            for descriptor in descriptors:
                try:
                    signal.pidfd_send_signal(descriptor, sig)
                except ProcessLookupError:
                    pass
            poller = select.poll()
            for descriptor in descriptors:
                poller.register(descriptor, select.POLLIN)
            for descriptor, _ in poller.poll(int(timeout*1000)) if descriptors else []:
                os.close(descriptor)
                descriptors.pop(descriptor, None)
            if not descriptors:
                break
    finally:
        for descriptor in descriptors:
            os.close(descriptor)
