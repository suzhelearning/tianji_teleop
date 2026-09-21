import struct
import time
import unittest
import zlib
from simulation.mapped_overlay import WIRE, decode


def packet(stamp=1_000_000_000,magic=b'MPT1',valid=1):
    poses=(.6,.2,1.,0.,0.,0.,1.,.6,-.2,1.,0.,0.,0.,1.)
    data=bytearray(WIRE.pack(magic,1,valid,WIRE.size,1,stamp,*poses,0))
    struct.pack_into('<I',data,len(data)-4,zlib.crc32(data[:-4]))
    return bytes(data)


class OverlayTests(unittest.TestCase):
    def test_explicit_observation_schema(self):
        self.assertEqual(WIRE.size,140)
        self.assertTrue(decode(packet(),1_000_000_001)[2])
        self.assertFalse(decode(packet(valid=0),1_000_000_001)[2])

    def test_stale_future_corrupt_and_joint_commands_rejected(self):
        for data,now in ((packet(),1_150_000_001),(packet(),999_999_999),
                         (packet(magic=b'TJRC'),1_000_000_000),(packet()[:-1]+b'\x00',1_000_000_000)):
            with self.subTest(now=now),self.assertRaises(ValueError):
                decode(data,now)


if __name__=='__main__':unittest.main()
