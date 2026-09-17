import hashlib
import json
from pathlib import Path
import unittest


class SourceManifestTest(unittest.TestCase):
    def test_migrated_reference_fingerprints(self):
        root = Path(__file__).resolve().parents[2]
        for name in ("source_manifest.json", "native_source_manifest.json"):
            manifest = json.loads((root / "pico2_hands" / name).read_text())
            for item in manifest["files"]:
                path = root / item["destination"]
                with self.subTest(file=item["destination"]):
                    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                                     item["destination_sha256"])
