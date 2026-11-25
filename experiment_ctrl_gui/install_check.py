"""Is the code `ros2 launch` will run the code in this checkout?

The dashboard reads its settings (the task YAMLs) from the checkout it runs from, but `ros2 launch
sinthlab_bringup ...` runs whatever sinthlab_bringup the shell sourced (run_gui.sh sources
<workspace>/install/setup.bash). If that install was built from older code -- not rebuilt after a pull, or
another workspace first on AMENT_PREFIX_PATH -- the arm runs old behaviour with new settings, silently. So
before a run the dashboard compares the installed orchestrators, package modules and launch files with the
checkout's, and refuses to start if they differ. A --symlink-install build links straight to the checkout, so
it always matches.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional, Tuple

PACKAGE = "sinthlab_bringup"


def installed_prefix() -> Optional[Path]:
    """The install prefix `ros2 launch` resolves sinthlab_bringup from: the first on AMENT_PREFIX_PATH."""
    for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if p and (Path(p) / "share" / PACKAGE).is_dir():
            return Path(p)
    return None


def _site_package(prefix: Path) -> Optional[Path]:
    for pattern in (f"lib/python3*/site-packages/{PACKAGE}", f"local/lib/python3*/dist-packages/{PACKAGE}",
                    f"lib/python3*/dist-packages/{PACKAGE}"):
        found = sorted(prefix.glob(pattern))
        if found:
            return found[0]
    return None


def _pairs(repo: Path, prefix: Path) -> List[Tuple[str, Path, Optional[Path]]]:
    """(name, the checkout's file, the installed file or None) for every file the launch runs."""
    src = repo / PACKAGE
    out = [(f"orchestrators/{f.name}", f, prefix / "lib" / PACKAGE / f.name)
           for f in sorted((src / "orchestrators").glob("*.py"))]
    out += [(f"launch/{f.name}", f, prefix / "share" / PACKAGE / "launch" / f.name)
            for f in sorted((src / "launch").glob("*.py"))]
    site = _site_package(prefix)
    for f in sorted((src / PACKAGE).rglob("*.py")):
        rel = f.relative_to(src / PACKAGE)
        out.append((f"{PACKAGE}/{rel}", f, None if site is None else site / rel))
    return out


def differing(repo: Path, prefix: Path) -> List[str]:
    """The files whose installed copy is missing or differs from the checkout's."""
    bad = []
    for name, mine, inst in _pairs(repo, prefix):
        if inst is None or not inst.exists():
            bad.append(name)
        elif inst.resolve() != mine.resolve() and inst.read_bytes() != mine.read_bytes():
            bad.append(name)
    return bad


def problem(repo: Path) -> Optional[str]:
    """None if the installed sinthlab_bringup is this checkout's code; else why not, and what to do."""
    prefix = installed_prefix()
    ws = repo.parent.parent
    rebuild = f"cd {ws} && colcon build --symlink-install --packages-select {PACKAGE}, then restart the dashboard"
    if prefix is None:
        return f"{PACKAGE} is not in the sourced workspace (AMENT_PREFIX_PATH). Build it: {rebuild}."
    bad = differing(repo, prefix)
    if not bad:
        return None
    shown = ", ".join(bad[:4]) + (f" and {len(bad) - 4} more" if len(bad) > 4 else "")
    return (f"the arm would run the code installed in {prefix}, which differs from this checkout ({repo}) in "
            f"{len(bad)} file(s): {shown}. Rebuild: {rebuild}.")
