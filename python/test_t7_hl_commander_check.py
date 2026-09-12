#!/usr/bin/env python3
import unittest

from t7_hl_commander_check import PARAM_SET_TIMEOUT_S, set_and_verify_param


class FakeParam:
    """Stands in for cflib's cf.param. respond=False simulates a callback
    that never fires (timeout). response=None means "echo back whatever
    set_value was called with" (simulates a successful readback)."""

    def __init__(self, response=None, respond=True):
        self._response = response
        self._respond = respond
        self.set_calls = []
        self._cb = None

    def add_update_callback(self, group, name, cb):
        self._cb = cb

    def set_value(self, full_name, value_str):
        self.set_calls.append((full_name, value_str))
        if self._respond:
            response = value_str if self._response is None else self._response
            self._cb(full_name, response)


class FakeCf:
    def __init__(self, response=None, respond=True):
        self.param = FakeParam(response=response, respond=respond)


class SetAndVerifyParamTests(unittest.TestCase):
    def test_success_when_readback_matches(self):
        cf = FakeCf()
        ok = set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=0.05)
        self.assertTrue(ok)
        self.assertEqual(cf.param.set_calls, [("commander.enHighLevel", "1")])

    def test_timeout_when_no_callback_fires(self):
        cf = FakeCf(respond=False)
        ok = set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=0.05)
        self.assertFalse(ok)

    def test_failure_when_readback_value_mismatches(self):
        cf = FakeCf(response="0")
        ok = set_and_verify_param(cf, "commander", "enHighLevel", 1, timeout_s=0.05)
        self.assertFalse(ok)

    def test_default_timeout_constant_is_positive(self):
        self.assertGreater(PARAM_SET_TIMEOUT_S, 0)


if __name__ == "__main__":
    unittest.main()
