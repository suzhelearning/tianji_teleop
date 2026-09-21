"""Test the offline clock adapter without starting sockets or native workers."""
import struct
import unittest
import zlib

from tianji_controller.mapped_palm_trace import shifted


class TraceCompareTests(unittest.TestCase):
    def test_clock_translation_preserves_payload_sequence_epoch_and_latency(self):
        packet=bytearray(656)
        struct.pack_into('<4sHHQQqq',packet,0,b'TJVR',4,656,27,9,10_000_000,10_000_250)
        packet[44:652]=bytes(i%256 for i in range(608))
        struct.pack_into('<I',packet,652,zlib.crc32(packet[:-4]))
        result=shifted(bytes(packet),1_000_000_000)
        self.assertEqual(result[:24],packet[:24])
        self.assertEqual(result[40:652],packet[40:652])
        source,bridge=struct.unpack_from('<qq',result,24)
        self.assertEqual(source,1_010_000_000)
        self.assertEqual(bridge-source,250)
        self.assertEqual(struct.unpack_from('<I',result,652)[0],zlib.crc32(result[:-4]))

    def test_translation_overflow_is_not_silently_wrapped(self):
        with self.assertRaises(struct.error): shifted(bytes(656),2**63)

    def test_xr_and_bridge_have_independent_clock_origins(self):
        packet=bytearray(656)
        struct.pack_into('<qq',packet,24,1_000_000,9_000_000_000)
        result=shifted(packet,10_000_000-1_000_000,10_000_000-9_000_000_000)
        self.assertEqual(struct.unpack_from('<qq',result,24),(10_000_000,10_000_000))


if __name__=='__main__': unittest.main()
