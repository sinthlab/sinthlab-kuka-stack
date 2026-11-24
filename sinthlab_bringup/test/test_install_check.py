"""experiment_ctrl_gui/install_check.py: the dashboard refuses to start when the installed sinthlab_bringup
is not this checkout's code (a stale build would run old behaviour with new settings).

    python3 -m pytest sinthlab_bringup/test
"""
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))

import install_check as ic  # noqa: E402


def make(tmp_path, mode):
    """A fake checkout (ws/src/repo) and an install prefix: linked (symlink-install), copied, or none."""
    repo = tmp_path / "ws" / "src" / "repo"
    pkg = repo / "sinthlab_bringup"
    for rel, text in {"orchestrators/free_move_orchestrator.py": "a", "launch/x.launch.py": "b",
                      "sinthlab_bringup/actions/visual_cue.py": "c", "sinthlab_bringup/__init__.py": ""}.items():
        (pkg / rel).parent.mkdir(parents=True, exist_ok=True)
        (pkg / rel).write_text(text)
    prefix = tmp_path / "ws" / "install" / "sinthlab_bringup"
    (prefix / "share" / "sinthlab_bringup").mkdir(parents=True)
    targets = {pkg / "orchestrators/free_move_orchestrator.py": prefix / "lib/sinthlab_bringup/free_move_orchestrator.py",
               pkg / "launch/x.launch.py": prefix / "share/sinthlab_bringup/launch/x.launch.py",
               pkg / "sinthlab_bringup": prefix / "lib/python3.12/site-packages/sinthlab_bringup"}
    for src, dst in targets.items():
        dst.parent.mkdir(parents=True, exist_ok=True)
        if mode == "link":
            dst.symlink_to(src)
        elif src.is_dir():
            shutil.copytree(src, dst)
        else:
            shutil.copy(src, dst)
    return repo, prefix


def test_a_symlink_install_or_an_identical_copy_passes(tmp_path, monkeypatch):
    for mode in ("link", "copy"):
        repo, prefix = make(tmp_path / mode, mode)
        monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
        assert ic.problem(repo) is None, mode


def test_a_stale_build_is_refused_with_the_rebuild_command(tmp_path, monkeypatch):
    repo, prefix = make(tmp_path, "copy")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    (repo / "sinthlab_bringup/sinthlab_bringup/actions/visual_cue.py").write_text("c, newer")   # pulled, not rebuilt
    (repo / "sinthlab_bringup/sinthlab_bringup/actions/free_reach.py").write_text("new file")
    msg = ic.problem(repo)
    assert "2 file(s)" in msg and "actions/visual_cue.py" in msg and "actions/free_reach.py" in msg
    assert f"cd {tmp_path / 'ws'} && colcon build --symlink-install --packages-select sinthlab_bringup" in msg


def test_nothing_installed(tmp_path, monkeypatch):
    repo, _ = make(tmp_path, "copy")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path / "elsewhere"))
    assert "not in the sourced workspace" in ic.problem(repo)
