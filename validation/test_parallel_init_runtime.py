"""Exercise the USART3 initialization CLI without a physical serial device."""
import fcntl
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time
import tty
import unittest


ROOT = Path(__file__).resolve().parents[1]
INIT_FRAME = bytes((0x53, 22, 0x45))


def status_frame(mode='z', *, detail=0x80, fault=0, corrupt=False):
    payload = bytes((0xA5, 0x5A, ord(mode), fault, 0, detail))
    checksum = 0
    for value in payload:
        checksum ^= value
        for _ in range(8):
            checksum = ((checksum << 1) ^ (0x07 if checksum & 0x80 else 0)) & 0xFF
    return payload + bytes((checksum ^ int(corrupt), 0x0D))


class ParallelInitRuntimeTests(unittest.TestCase):
    def exercise(self, initial, *, completion=None, lose_feedback=False, lock_port=False):
        master, slave = pty.openpty()
        try:
            tty.setraw(slave)
            os.set_blocking(master, False)
            if lock_port:
                fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with tempfile.TemporaryDirectory(prefix='letsgo_parallel_init_') as directory:
                config = Path(directory) / 'serial.yaml'
                config.write_text(
                    'motor_serial:\n  ros__parameters:\n'
                    f'    port: {os.ttyname(slave)}\n'
                    '    status_timeout: 0.4\n    initialization_timeout: 0.85\n',
                    encoding='utf-8',
                )
                process = subprocess.Popen(
                    ['bash', 'run/parallel_init.sh', '--params-file', str(config)],
                    cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding='utf-8', start_new_session=True,
                )
                received = bytearray()
                request_at = None
                completion_sent = False

                def drain_commands():
                    try:
                        received.extend(os.read(master, 4096))
                    except BlockingIOError:
                        pass

                try:
                    deadline = time.monotonic() + 15.0
                    while process.poll() is None and time.monotonic() < deadline:
                        now = time.monotonic()
                        drain_commands()
                        if request_at is None and len(received) >= len(INIT_FRAME):
                            request_at = now
                        feedback = initial
                        if request_at is not None:
                            if lose_feedback:
                                feedback = None
                            elif completion is not None and now - request_at >= 0.18:
                                feedback = completion
                                completion_sent = True
                        if feedback is not None:
                            os.write(master, feedback)
                        time.sleep(0.02)
                    self.assertIsNotNone(process.poll(), 'Initialization CLI did not exit')
                    drain_commands()
                    output, _ = process.communicate()
                    return process.returncode, bytes(received), output, completion_sent
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.communicate(timeout=5.0)
        finally:
            os.close(master)
            os.close(slave)

    def test_invalid_feedback_never_sends_initialize(self):
        cases = [
            (None, '未收到有效电控回传'),
            (status_frame(corrupt=True), '未收到有效电控回传'),
            (status_frame(detail=0), '未声明支持'),
            (status_frame(detail=0, fault=1), '电控故障 1'),
        ]
        for frame, expected_error in cases:
            with self.subTest(expected_error=expected_error, frame=frame):
                result, received, output, _ = self.exercise(frame)
                self.assertNotEqual(result, 0, output)
                self.assertEqual(received, b'', output)
                self.assertIn(expected_error, output)

    def test_writes_once_and_waits_for_running_feedback(self):
        result, received, output, completion_sent = self.exercise(
            status_frame(), completion=status_frame('a'),
        )
        self.assertEqual(result, 0, output)
        self.assertEqual(received, INIT_FRAME, output)
        self.assertTrue(completion_sent, 'CLI succeeded before running feedback was sent')
        self.assertIn('初始化完成', output)

    def test_already_running_never_sends_initialize(self):
        result, received, output, _ = self.exercise(status_frame('a'))
        self.assertEqual(result, 0, output)
        self.assertEqual(received, b'', output)
        self.assertIn('已处于运行态', output)

    def test_feedback_can_fail_after_request_without_retransmission(self):
        cases = [
            ({}, '未在限时内完成'),
            ({'lose_feedback': True}, '回传中断'),
            ({'completion': status_frame('a', fault=2)}, '电控故障 2'),
        ]
        for conditions, expected_error in cases:
            with self.subTest(expected_error=expected_error):
                result, received, output, _ = self.exercise(status_frame(), **conditions)
                self.assertNotEqual(result, 0, output)
                self.assertEqual(received, INIT_FRAME, output)
                self.assertIn(expected_error, output)

    def test_occupied_port_never_sends_initialize(self):
        result, received, output, _ = self.exercise(status_frame(), lock_port=True)
        self.assertNotEqual(result, 0, output)
        self.assertEqual(received, b'', output)
        self.assertIn('exclusively lock', output)


if __name__ == '__main__':
    unittest.main()

