#!/usr/bin/env python3
import struct
import unittest

from t8_sensor_sequence_check import (
    DEFAULT_SEQUENCE,
    MAX_LAND_TARGET_M,
    MAX_TAKEOFF_TARGET_M,
    pack_step,
    validate_sequence,
)


class PackStepTests(unittest.TestCase):
    def test_pack_step_length_is_29_bytes(self):
        packed = pack_step("TAKEOFF_SENSOR", [0.3, 0.5, 3.0])
        self.assertEqual(len(packed), 29)

    def test_pack_step_type_byte(self):
        packed = pack_step("DELAY", [1.0])
        self.assertEqual(packed[0], 1)

    def test_pack_step_pads_unused_params_with_zero(self):
        packed = pack_step("DELAY", [1.0])
        floats = struct.unpack("<7f", packed[1:])
        self.assertEqual(floats, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_pack_step_land_sensor_params(self):
        packed = pack_step("LAND_SENSOR", [0.0, 2.0])
        floats = struct.unpack("<7f", packed[1:])
        self.assertEqual(floats[:2], (0.0, 2.0))

    def test_pack_step_unknown_type_raises(self):
        with self.assertRaises(ValueError):
            pack_step("SPIN", [1.0])

    def test_pack_step_too_many_params_raises(self):
        with self.assertRaises(ValueError):
            pack_step("DELAY", [1.0] * 8)


class ValidateSequenceTests(unittest.TestCase):
    def test_default_sequence_is_valid(self):
        self.assertEqual(validate_sequence(DEFAULT_SEQUENCE), [])

    def test_empty_sequence_returns_error(self):
        errors = validate_sequence([])
        self.assertTrue(any("序列为空" in e for e in errors))

    def test_last_step_not_land_sensor_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("DELAY", 1.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("最后一步必须是 LAND_SENSOR" in e for e in errors))

    def test_takeoff_target_out_of_bounds_returns_error(self):
        seq = [("TAKEOFF_SENSOR", MAX_TAKEOFF_TARGET_M + 1.0, 0.5, 3.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("target=" in e for e in errors))

    def test_takeoff_timeout_not_greater_than_stable_hold_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 3.0, 3.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("必须大于 stable_hold" in e for e in errors))

    def test_delay_nonpositive_hold_time_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("DELAY", 0.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("DELAY hold_time=" in e for e in errors))

    def test_land_target_out_of_bounds_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("LAND_SENSOR", MAX_LAND_TARGET_M + 1.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("LAND_SENSOR target=" in e for e in errors))

    def test_land_nonpositive_duration_returns_error(self):
        seq = [("TAKEOFF_SENSOR", 0.3, 0.5, 3.0), ("LAND_SENSOR", 0.0, 0.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("LAND_SENSOR duration=" in e for e in errors))

    def test_unknown_step_name_returns_error(self):
        seq = [("SPIN", 1.0), ("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq)
        self.assertTrue(any("命令名未知" in e for e in errors))

    def test_too_many_steps_returns_error(self):
        seq = [("DELAY", 0.1)] * 20 + [("LAND_SENSOR", 0.0, 2.0)]
        errors = validate_sequence(seq, max_steps=16)
        self.assertTrue(any("超过固件缓冲区上限" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
