"""Exercise the discovery CLI against a C ABI double with no motor-control API."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "read_hand_serials.py"
SDK_HEADER = ROOT / "vendor/wuji-sdk/include"

FAKE_SDK = r'''
#include "wuji_sdk.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
extern "C" {
WujiStatus wuji_init(const WujiInitOptions*) { return WUJI_STATUS_OK; }
void wuji_shutdown() {}
const char* wuji_version() { return "fixture-sdk"; }
const char* wuji_last_error() { return "fixture read failed"; }
WujiStatus wuji_scan(WujiDiscovered** out, size_t* count) {
  if (std::getenv("TEST_EMPTY_SCAN")) { *out=nullptr; *count=0; return WUJI_STATUS_OK; }
  *count=3; *out=new WujiDiscovered[3]{};
  const char* serials[]={"HAND-B", "GLOVE", "HAND-A"};
  for(size_t i=0;i<3;++i) {
    std::snprintf((*out)[i].serial_number,64,"%s",serials[i]);
    std::snprintf((*out)[i].model,32,"%s",i==1?"WujiGlove":"WujiHand2");
    (*out)[i].device_id=i==1?WUJI_DEVICE_TYPE_WUJI_GLOVE:WUJI_DEVICE_TYPE_WUJI_HAND_2;
    (*out)[i].transport=WUJI_TRANSPORT_TYPE_UDP;
    std::snprintf((*out)[i].address,64,"192.0.2.%zu:2333",10+i);
  }
  return WUJI_STATUS_OK;
}
void wuji_discovered_free(WujiDiscovered* arr,size_t) { delete[] arr; }
WujiConnectOptions wuji_connect_options_default() { return {}; }
WujiStatus wuji_connect(const WujiConnectTarget* target,const char*,const WujiConnectOptions*,WujiDevice** out) {
  if(!std::getenv("TEST_ALLOW_QUERY")) std::abort();
  *out=reinterpret_cast<WujiDevice*>(std::strcmp(target->value,"HAND-B")==0?1:2);
  return WUJI_STATUS_OK;
}
WujiStatus wuji_hand_2_get_handedness(WujiDevice* dev,WujiHandedness* out) {
  if(dev==reinterpret_cast<WujiDevice*>(2) && std::getenv("TEST_FAIL_SIDE"))
    return WUJI_STATUS_ERR_NOT_FOUND;
  *out=dev==reinterpret_cast<WujiDevice*>(1)?WUJI_HANDEDNESS_LEFT:WUJI_HANDEDNESS_RIGHT;
  return WUJI_STATUS_OK;
}
WujiStatus wuji_dev_disconnect(WujiDevice*) { return WUJI_STATUS_OK; }
void wuji_dev_release(WujiDevice*) {}
}
'''


class ReadHandSerialsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="hand-discovery-test-")
        folder = Path(cls.temp.name)
        source = folder / "sdk.cpp"
        source.write_text(FAKE_SDK)
        cls.library = folder / "sdk.so"
        subprocess.run(["/usr/bin/g++", "-shared", "-fPIC", "-I", str(SDK_HEADER),
                        str(source), "-o", str(cls.library)], check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_cli(self, *args, extra_env=None):
        env = os.environ.copy()
        env.update(extra_env or {})
        return subprocess.run([sys.executable, str(SCRIPT), "--sdk-library", str(self.library),
                               "--json", *args], env=env, text=True, capture_output=True, timeout=10)

    def test_scan_does_not_connect_or_guess_sides(self):
        result = self.run_cli()
        self.assertEqual(result.returncode, 0, result.stderr)
        devices = json.loads(result.stdout)["devices"]
        self.assertEqual([d["serial_number"] for d in devices], ["HAND-A", "HAND-B"])
        self.assertTrue(all(d["side"] == "unknown" for d in devices))
        self.assertTrue(all(d["query_error"] is None for d in devices))

    def test_explicit_query_uses_device_handedness_not_scan_order(self):
        result = self.run_cli("--read-handedness", extra_env={"TEST_ALLOW_QUERY": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        sides = {d["serial_number"]: d["side"] for d in json.loads(result.stdout)["devices"]}
        self.assertEqual(sides, {"HAND-A": "right", "HAND-B": "left"})

    def test_failed_side_query_retains_serial_and_does_not_guess(self):
        result = self.run_cli("--read-handedness", extra_env={"TEST_ALLOW_QUERY": "1", "TEST_FAIL_SIDE": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        devices = {d["serial_number"]: d for d in json.loads(result.stdout)["devices"]}
        self.assertEqual(devices["HAND-A"]["side"], "unknown")
        self.assertIn("failed", devices["HAND-A"]["query_error"])
        self.assertEqual(devices["HAND-B"]["side"], "left")

    def test_empty_scan_returns_empty_device_list(self):
        result = self.run_cli(extra_env={"TEST_EMPTY_SCAN": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["devices"], [])

    def test_output_does_not_overwrite_existing_device_file(self):
        target = Path(self.temp.name) / "devices.json"
        target.write_text("keep operator configuration")
        result = self.run_cli("--output", str(target))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(target.read_text(), "keep operator configuration")


if __name__ == "__main__":
    unittest.main()
