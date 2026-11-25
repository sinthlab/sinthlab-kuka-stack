"""Post-processing upload (experiment_ctrl_gui/postprocess.py): a cancelled or failed copy leaves no .partial.

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

pp = pytest.importorskip("postprocess")


def test_a_verified_copy(tmp_path):
    src = tmp_path / "a.nwb"; src.write_bytes(b"x" * 100_000)
    dest = tmp_path / "server"; dest.mkdir()
    got = pp._copy_verified(str(src), str(dest / "a.nwb"), lambda m, f: None)
    assert (dest / "a.nwb").read_bytes() == src.read_bytes() and got["bytes"] == 100_000
    assert not list(dest.glob("*.partial"))


def test_a_cancelled_copy_leaves_nothing(tmp_path):
    src = tmp_path / "a.nwb"; src.write_bytes(b"x" * 100_000)
    dest = tmp_path / "server"; dest.mkdir()

    def say(msg, frac):
        if frac > 0.2:
            raise pp.Cancelled()
    with pytest.raises(pp.Cancelled):
        pp._copy_verified(str(src), str(dest / "a.nwb"), say)
    assert list(dest.iterdir()) == []
