import sys
from pathlib import Path
import struct
import unittest
import zlib

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_synthetic_body_mapping import transform_points, synthesize, compare, parse_output


class SyntheticBodyTest(unittest.TestCase):
    def setUp(self):
        self.points = np.array([[[0, .16, 1.121], [.1, .3, 1.2], [.3, .3, 1.2], [.35, .3, 1.2]],
                                [[0, -.16, 1.121], [.1, -.3, 1.2], [.3, -.3, 1.2], [.35, -.3, 1.2]]])

    def test_identity(self):
        np.testing.assert_allclose(transform_points(self.points, 1), self.points, atol=1e-15)

    def test_uniform(self):
        root = np.array([0, 0, 1.121])
        np.testing.assert_allclose(transform_points(self.points, 1.2), root+(self.points-root)*1.2)

    def test_independent_lengths(self):
        out = transform_points(self.points, 1, .85, 1.1, .9, 1)
        np.testing.assert_allclose(np.diff(out, axis=1), np.diff(self.points, axis=1)*np.array([1.1,.9,1])[None,:,None])
        np.testing.assert_allclose(out[:,0].mean(axis=0), [0,0,1.121])

    def test_bad_ratios(self):
        for ratio in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                transform_points(self.points, ratio)

    def test_wire_preservation_and_crc(self):
        packet = bytearray(656)
        struct.pack_into('<4sHH', packet, 0, b'TJVR', 4, 656)
        struct.pack_into('<24d', packet, 204, *self.points.ravel())
        data = struct.pack('<4sHHQq', b'TJVT', 1, 656, 1, 42) + packet
        output = synthesize(data, height_ratio=1.1)
        self.assertEqual(output[:228], data[:228])
        self.assertEqual(output[420:676], data[420:676])
        self.assertEqual(struct.unpack_from('<I', output, 676)[0], zlib.crc32(output[24:676]))

    def test_comparison(self):
        a = np.zeros(48)
        b = a.copy(); b[12] = .01
        result = compare({(1,2): a, (2,3): None}, {(1,2): b, (2,3): a})
        self.assertEqual(result['valid_mask_changed_frames'], 1)
        self.assertEqual(result['paired_valid_frames'], 1)
        self.assertAlmostEqual(result['palm_target_difference_max_m'], .01)
        with self.assertRaises(ValueError):
            compare({(1,2): a}, {})

    def test_parser(self):
        frames, _, _ = parse_output('mapping_frame 1 2 0\n')
        self.assertIsNone(frames[(1,2)])
        with self.assertRaises(ValueError):
            parse_output('mapping_frame 1 2 1 0\n')


if __name__ == '__main__':
    unittest.main()
