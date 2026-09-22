"""Offline tests: fake socket only; never connect to a robot."""
import importlib.util
import json
from pathlib import Path
import struct
import unittest
from unittest import mock

import numpy as np

_PATH = Path(__file__).resolve().parents[1] / "src/remote_controller/trackj_streamer.py"
_SPEC = importlib.util.spec_from_file_location("trackc_diagnostics_test_module", _PATH)
streamers = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(streamers)


class TrackCDiagnosticsTest(unittest.TestCase):
    def make_streamer(self):
        sock = mock.Mock()
        with mock.patch.object(streamers.socket, "socket", return_value=sock):
            streamer = streamers.TrackCStreamer(None, "unused.invalid", 12345)
        return streamer, sock

    def test_success_records_packet_and_snapshot_is_detached(self):
        streamer, sock = self.make_streamer()
        initial = streamer.get_diagnostics()
        self.assertIsNone(initial["last_send_seq"])
        self.assertEqual(initial["successful_send_count"], 0)
        target = np.eye(4)
        target[0, 3] = 0.003
        with mock.patch.object(streamers.time, "monotonic", return_value=123.5):
            streamer._send_packet(target)
        packet, addr = sock.sendto.call_args.args
        values = struct.unpack(streamers.TRACKC_PACKET_FORMAT, packet)
        self.assertEqual(values[0], 1)
        np.testing.assert_array_equal(np.array(values[1:]).reshape(4, 4), target)
        diag = streamer.get_diagnostics()
        self.assertEqual(diag["last_send_monotonic_s"], 123.5)
        self.assertEqual(diag["successful_send_count"], 1)
        self.assertEqual(addr, ("unused.invalid", 12345))
        diag["last_sent_target_T"][0][3] = 99
        target[0, 3] = 88
        self.assertEqual(streamer.get_diagnostics()["last_sent_target_T"][0][3], 0.003)
        json.dumps(diag)

    def test_hold_keeps_sending_after_completed(self):
        streamer, sock = self.make_streamer()
        target = np.eye(4)
        target[1, 3] = 0.004
        streamer.manager.initialize_hold(target)
        streamer.running.set()
        def stop_after_third(packet, addr):
            if sock.sendto.call_count == 3:
                streamer.running.clear()
            return len(packet)
        sock.sendto.side_effect = stop_after_third
        with mock.patch.object(streamers.time, "sleep"):
            streamer._send_loop()
        diag = streamer.get_diagnostics()
        self.assertEqual(diag["last_send_seq"], 3)
        self.assertEqual(diag["successful_send_count"], 3)
        self.assertTrue(diag["manager"]["completed"])
        self.assertEqual(diag["manager"]["index"], 0)
        self.assertEqual(diag["manager"]["length"], 1)
        for call in sock.sendto.call_args_list:
            vals = struct.unpack(streamers.TRACKC_PACKET_FORMAT, call.args[0])
            np.testing.assert_array_equal(np.array(vals[1:]).reshape(4, 4), target)

    def test_failure_retains_last_success_and_propagates(self):
        streamer, sock = self.make_streamer()
        old = np.eye(4)
        streamer._send_packet(old)
        last_success_time = streamer.get_diagnostics()["last_send_monotonic_s"]
        scheduled = old.copy()
        scheduled[0, 3] = 0.009
        streamer.manager.initialize_hold(scheduled)
        streamer.running.set()
        sock.sendto.side_effect = OSError("simulated send failure")
        with self.assertRaisesRegex(OSError, "simulated send failure"):
            streamer._send_loop()
        diag = streamer.get_diagnostics()
        self.assertEqual(diag["last_send_seq"], 1)
        self.assertEqual(diag["successful_send_count"], 1)
        self.assertEqual(diag["last_send_monotonic_s"], last_success_time)
        self.assertEqual(diag["send_error"]["type"], "OSError")
        self.assertEqual(diag["send_error"]["message"], "simulated send failure")
        self.assertGreaterEqual(diag["send_error"]["monotonic_s"], last_success_time)
        self.assertEqual(diag["last_sent_target_T"][0][3], 0)
        self.assertEqual(diag["manager"]["scheduled_target_T"][0][3], 0.009)
        self.assertTrue(diag["manager"]["completed"])


if __name__ == "__main__":
    unittest.main()
