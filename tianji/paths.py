"""Portable checkout and dataset locations shared by command entry points."""
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent.parent
DATASET = Path(os.environ.get("TIANJI_DATASET", "/data/TianjiData/raw")).expanduser().resolve()
COMPRESSED_DATASET = DATASET.parent / "compressed"
