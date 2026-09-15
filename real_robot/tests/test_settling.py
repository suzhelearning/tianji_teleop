import unittest
from real_robot.settling import FeedbackRest


class FeedbackRestTests(unittest.TestCase):
    def test_duplicates_cannot_complete_rest(self):
        rest=FeedbackRest(.03,200_000_000,300_000_000)
        self.assertFalse(rest.observe([0.],1_000_000_000,True))
        for _ in range(100):
            self.assertFalse(rest.observe([0.],1_000_000_000,True))
        self.assertFalse(rest.observe([0.],1_010_000_000,True))
        self.assertTrue(rest.observe([0.],1_210_000_000,True))

    def test_motion_within_position_tolerance_prevents_rest(self):
        rest=FeedbackRest(.03,200_000_000,300_000_000)
        for tick in range(100):
            self.assertFalse(rest.observe([.01 if tick%2 else 0.],1_000_000_000+tick*5_000_000,True))

    def test_gap_rollback_invalid_and_out_of_position_reset(self):
        rest=FeedbackRest(.03,200_000_000,300_000_000)
        for stamp in (1_000_000_000,1_010_000_000,1_210_000_000): rest.observe([0.],stamp,True)
        self.assertTrue(rest.resting)
        self.assertFalse(rest.observe([0.],1_600_000_000,True))
        self.assertFalse(rest.observe([0.],1_500_000_000,True))
        self.assertFalse(rest.observe([float('nan')],1_700_000_000,True))
        self.assertFalse(rest.observe([0.],1_800_000_000,False))
