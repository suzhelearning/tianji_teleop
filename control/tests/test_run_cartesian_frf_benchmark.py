import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_cartesian_frf_benchmark.py"
SPEC = importlib.util.spec_from_file_location("run_cartesian_frf_benchmark", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MatrixTests(unittest.TestCase):
    def test_full_matrix_has_336_unique_cases(self):
        cases = MODULE.expand_cases(MODULE.ALGORITHMS, MODULE.ARMS,
                                    MODULE.WORKING_POINTS, MODULE.CHANNELS)
        self.assertEqual(len(cases), 336)
        self.assertEqual(len({case.slug for case in cases}), 336)

    def test_filters_form_cartesian_product(self):
        cases = MODULE.expand_cases(MODULE.ALGORITHMS[:2], ["left"],
                                    ["center"], ["x", "rz"])
        self.assertEqual(len(cases), 4)

    def test_resume_requires_nonempty_csv_with_header(self):
        case = MODULE.expand_cases(MODULE.ALGORITHMS[:1], ["left"],
                                   ["center"], ["x"])[0]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / f"{case.slug}.csv"
            self.assertFalse(MODULE.case_complete(path))
            path.write_text("broken")
            self.assertFalse(MODULE.case_complete(path))
            path.write_text("# dt=0.005\nsample,time_s,input,output,accepted\n0,0,0,0,1\n")
            self.assertTrue(MODULE.case_complete(path))


if __name__ == "__main__":
    unittest.main()
