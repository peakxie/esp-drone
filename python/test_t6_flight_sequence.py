#!/usr/bin/env python3
import unittest

from t6_flight_sequence import FLIGHT_PLAN, MAX_HEIGHT_M, MAX_XY_OFFSET_M, validate_flight_plan


class ValidateFlightPlanTests(unittest.TestCase):
    def test_default_flight_plan_is_valid(self):
        self.assertEqual(validate_flight_plan(FLIGHT_PLAN), [])

    def test_minimal_valid_plan_returns_no_errors(self):
        plan = [("takeoff", 0.3), ("land",)]
        self.assertEqual(validate_flight_plan(plan), [])

    def test_empty_plan_returns_error(self):
        errors = validate_flight_plan([])
        self.assertEqual(len(errors), 1)
        self.assertIn("命令列表为空", errors[0])

    def test_first_command_not_takeoff_returns_error(self):
        plan = [("hover", 1.0), ("takeoff", 0.3), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("第一条命令必须是 takeoff" in e for e in errors))

    def test_unknown_command_name_returns_error(self):
        plan = [("takeoff", 0.3), ("spin", 1.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("命令名未知" in e for e in errors))

    def test_wrong_arg_count_returns_error(self):
        plan = [("takeoff", 0.3, 1.0, 99.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("参数个数为" in e for e in errors))

    def test_goto_xy_offset_out_of_bounds_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", MAX_XY_OFFSET_M + 0.5, 0.0, 0.3, 2.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("超出 ±" in e for e in errors))

    def test_goto_height_out_of_bounds_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", 0.0, 0.0, MAX_HEIGHT_M + 0.5, 2.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("高度 h=" in e for e in errors))

    def test_goto_nonpositive_duration_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", 0.1, 0.0, 0.3, 0.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("goto duration_s=" in e for e in errors))

    def test_takeoff_height_out_of_bounds_returns_error(self):
        plan = [("takeoff", MAX_HEIGHT_M + 1.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("takeoff 高度" in e for e in errors))

    def test_hover_nonpositive_duration_returns_error(self):
        plan = [("takeoff", 0.3), ("hover", 0.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("hover duration_s=" in e for e in errors))

    def test_land_optional_duration_is_valid(self):
        plan = [("takeoff", 0.3), ("land", 1.5)]
        self.assertEqual(validate_flight_plan(plan), [])

    def test_land_nonpositive_duration_returns_error(self):
        plan = [("takeoff", 0.3), ("land", 0.0)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("land duration_s=" in e for e in errors))

    def test_multiple_takeoff_returns_error(self):
        plan = [("takeoff", 0.3), ("goto", 0.1, 0.0, 0.3, 1.0), ("takeoff", 0.4), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("出现了 2 次 takeoff" in e for e in errors))

    def test_land_not_terminal_returns_error(self):
        plan = [("takeoff", 0.3), ("land",), ("hover", 1.0)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("land 只能是最后一条" in e for e in errors))

    def test_multiple_land_returns_error(self):
        plan = [("takeoff", 0.3), ("land",), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("出现了 2 次 land" in e for e in errors))

    def test_total_duration_exceeds_max_flight_time_returns_error(self):
        plan = [("takeoff", 0.3), ("hover", 40.0), ("land",)]
        errors = validate_flight_plan(plan)
        self.assertTrue(any("超过" in e and "max_flight_time_s" in e for e in errors))

    def test_total_duration_within_budget_including_auto_appended_land(self):
        # 没写 land，预算里要把 main() 自动补的一次 land（含最坏情况触地等待）算进去
        plan = [("takeoff", 0.3), ("hover", 1.0)]
        errors = validate_flight_plan(plan)
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
