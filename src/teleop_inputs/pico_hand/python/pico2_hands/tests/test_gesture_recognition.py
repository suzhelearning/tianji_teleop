import unittest
import numpy as np
from pico2_hands.reference import gesture_recognition as gestures


def hand_points(curled=False):
    points = np.zeros((21, 3))
    for finger in range(5):
        base = 1 + finger * 4
        x = (finger - 2) * .025
        for joint in range(4):
            points[base + joint] = [x, .05 + joint * .025, 0]
        if curled:
            points[base+2] = [x, .085, .02]
            points[base+3] = [x, .055, .02]
    return points


class GestureRecognitionTest(unittest.TestCase):
    def test_open_fist_pinch_and_rigid_scale_invariance(self):
        examples = {'open': hand_points(), 'fist': hand_points(True), 'pinch': hand_points()}
        examples['pinch'][4] = examples['pinch'][8] + [.001, 0, 0]
        rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        for expected, points in examples.items():
            for transformed in (points, 3 * points @ rotation.T + [1, 2, 3]):
                observation = gestures.classify_hand(transformed, valid=True)
                self.assertTrue(observation.available)
                self.assertEqual(observation.gesture, expected)

    def test_invalid_or_degenerate_geometry_is_unavailable_not_open(self):
        for points, valid in ((hand_points(), False), (np.zeros((21, 3)), True),
                              (np.full((21, 3), np.nan), True)):
            result = gestures.classify_hand(points, valid=valid)
            self.assertFalse(result.available)
            self.assertEqual(result.gesture, 'unknown')

    def test_pinch_hysteresis_and_invalid_previous_label(self):
        points = hand_points()
        scale = np.linalg.norm(points[9] - points[0])
        points[4] = points[8] + [scale * .28, 0, 0]
        self.assertEqual(gestures.classify_hand(points, valid=True, previous='pinch').gesture, 'pinch')
        self.assertNotEqual(gestures.classify_hand(points, valid=True).gesture, 'pinch')
        with self.assertRaises(ValueError):
            gestures.classify_hand(points, valid=True, previous='start_request')
