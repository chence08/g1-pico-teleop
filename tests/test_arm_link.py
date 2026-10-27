import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "outputs"))
from g1_arm_link import validate_target, ARM_LOWER


def packet():
    return {"seq": 1, "challenge": "fresh", "input_age_s": 0.01, "q": [0.] * 14, "follow": False}


class GuardTests(unittest.TestCase):
    def test_accepts_fresh_bounded_target(self):
        self.assertEqual(validate_target(packet(), [0.] * 14, [("fresh", 10.)], 10.1, 0), (1, [0.] * 14, False))

    def test_rejects_queued_duplicate_and_stale_input(self):
        for now, seq, age in [(10.3, 0, .01), (10.1, 1, .01), (10.1, 0, .3)]:
            row = packet()
            row["input_age_s"] = age
            with self.assertRaises(ValueError):
                validate_target(row, [0.] * 14, [("fresh", 10.)], now, seq)

    def test_input_age_includes_conservative_command_transit_bound(self):
        row = packet(); row['input_age_s'] = .12
        with self.assertRaises(ValueError):
            validate_target(row,[0.]*14,[('fresh',10.)],10.06,0)
        row['input_age_s'] = .01
        validate_target(row,[0.]*14,[('fresh',10.)],10.06,0)

    def test_rejects_invalid_and_excessive_targets(self):
        for q in ([0.] * 13, [math.nan] * 14, [True] * 14, [.151] * 14):
            row = packet()
            row["q"] = q
            with self.assertRaises(ValueError):
                validate_target(row, [0.] * 14, [("fresh", 10.)], 10.1, 0)
        with self.assertRaises(ValueError):
            validate_target([], [0.] * 14, [("fresh", 10.)], 10.1, 0)

    def test_joint_limits_apply_even_inside_relative_envelope(self):
        row = packet()
        initial = ARM_LOWER.copy()
        row["q"] = initial.copy()
        row["q"][0] -= .01
        with self.assertRaises(ValueError):
            validate_target(row, initial, [("fresh", 10.)], 10.1, 0)

    def test_explicit_larger_range_preserves_bounds_and_freshness(self):
        row = packet()
        row['q'] = [.49] * 14
        validate_target(row, [0.] * 14, [('fresh', 10.)], 10.1, 0, .5)
        for radius in (.15, 1.501, float('nan')):
            with self.assertRaises(ValueError):
                validate_target(row, [0.] * 14, [('fresh', 10.)], 10.1, 0, radius)
        row['q'] = [.501] * 14
        with self.assertRaises(ValueError):
            validate_target(row, [0.] * 14, [('fresh', 10.)], 10.1, 0, .5)
        row['q'] = [.49] * 14
        with self.assertRaises(ValueError):
            validate_target(row, [0.] * 14, [('fresh', 10.)], 10.3, 0, .5)

    def test_responsive_envelope_and_absolute_limit_mode(self):
        row=packet();row['q']=[1.1]*14
        validate_target(row,[0.]*14,[('fresh',10.)],10.1,0,1.2)
        validate_target(row,[0.]*14,[('fresh',10.)],10.1,0,0)
        row['q'][0]=3.
        with self.assertRaises(ValueError):
            validate_target(row,[0.]*14,[('fresh',10.)],10.1,0,0)
        row=packet();row['pc_sent_s']=float('nan')
        with self.assertRaises(ValueError):
            validate_target(row,[0.]*14,[('fresh',10.)],10.1,0,1.2)


if __name__ == "__main__":
    unittest.main()
