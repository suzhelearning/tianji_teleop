"""Portable checkout and dataset locations shared by command entry points."""
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent.parent
DATASET = Path(os.environ.get("TIANJI_DATASET", str(ROOT / "dataset"))).expanduser().resolve()
COMPRESSED_DATASET = DATASET.with_name(DATASET.name + "_jpeg50")
