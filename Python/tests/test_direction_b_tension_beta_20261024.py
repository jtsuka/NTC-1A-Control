"""Offline regression for 2026-10-24 Direction-B beta revisions.

Run from repository root:
    python3 -m unittest discover -s Python/tests -p 'test_direction_b_tension_beta_20261024.py' -v

No pyserial, hardware, or Raspberry Pi required. No port is opened.
"""
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parents[2]

BETAS = (
    "Python/NTC_1A_serial_comm_v1_3_5_beta.py",
    "Python/pi_logger_evidence_v2_1_beta.py",
    "Python/pi_logger_evidence_v3_1_beta.py",
    "Python/pi_logger_synced_v2_1_beta.py",
    "ESP32/TCBridge_EvidenceV2/pi_logger_evidence_v2_1_beta.py",
)

ORIGINALS = (
    "Python/NTC_1A_serial_comm_v1_3_4.py",
    "Python/pi_logger_evidence_v2.py",
    "Python/pi_logger_evidence_v3.py",
    "Python/pi_logger_synced_v2.py",
    "ESP32/TCBridge_EvidenceV2/pi_logger_evidence_v2.py",
)


def load(path, name):
    serial = types.ModuleType("serial")
    serial.Serial = lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError("Hardware access forbidden in offline tests")
    )
    utils = types.ModuleType("NTC_1A_utils")
    utils.out = lambda message: None
    with patch.dict(sys.modules, {"serial": serial, "NTC_1A_utils": utils}):
        spec = importlib.util.spec_from_file_location(name, ROOT / path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def frame(tension, length=0):
    return bytes((
        length % 100, length // 100 % 100, length // 10000 % 100,
        tension % 10, tension // 10, 0x7F
    ))


class DirectionBBetaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.betas = [load(path, "beta_%d" % i) for i, path in enumerate(BETAS)]
        cls.originals = [load(path, "original_%d" % i) for i, path in enumerate(ORIGINALS)]

    def accepts(self, module, packet, pi):
        return (module._is_direction_b_telemetry(packet) if pi
                else module.is_telemetry(packet))

    def test_all_1024_tension_values(self):
        for i, module in enumerate(self.betas):
            for tension in range(1024):
                with self.subTest(module=BETAS[i], tension=tension):
                    self.assertTrue(self.accepts(module, frame(tension), i == 0))

    def test_length_groups_0_to_99(self):
        for i, module in enumerate(self.betas):
            self.assertTrue(self.accepts(module, bytes((99, 99, 99, 9, 99, 0x7F)), i == 0))
            for position in range(3):
                pkt = bytearray(frame(17))
                pkt[position] = 100
                self.assertFalse(self.accepts(module, bytes(pkt), i == 0))

    def test_invalid_tension_encoding(self):
        for i, module in enumerate(self.betas):
            for bad in (
                bytes((0, 0, 0, 10, 0, 0x7F)),
                bytes((0, 0, 0, 4, 102, 0x7F)),  # 1024 > 1023
                bytes((0, 0, 0, 0, 103, 0x7F)),
                bytes((0, 0, 0, 0, 0, 0x7E)),
                bytes((0, 0, 0, 0, 0)),
                bytes((0, 0, 0, 0, 0, 0x7F, 0)),
            ):
                with self.subTest(module=BETAS[i], bad=bad):
                    self.assertFalse(self.accepts(module, bad, i == 0))

    def test_real_tc106_1023_example(self):
        pkt = bytes.fromhex("00 00 00 03 66 7F")
        for i, module in enumerate(self.betas):
            self.assertTrue(self.accepts(module, pkt, i == 0))
            self.assertFalse(self.accepts(self.originals[i], pkt, i == 0))

    def test_boot_magic_rejected_by_field_validator(self):
        pkt = bytes.fromhex("DE AD BE EF 01 7F")
        for i, module in enumerate(self.betas):
            self.assertFalse(self.accepts(module, pkt, i == 0))

    def test_pi_decodes_full_tension_range(self):
        pi = self.betas[0]
        for tension in range(1024):
            pi._publish_telemetry(frame(tension, 1506))
            self.assertEqual(pi.get_telemetry()["actual_tension"], tension)
            self.assertEqual(pi.get_telemetry()["wind_length"], 1506)


if __name__ == "__main__":
    unittest.main()
