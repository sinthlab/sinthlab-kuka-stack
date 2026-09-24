#!/usr/bin/env python3
"""Check that every parameter and block in the sinthlab YAMLs has a one-line description.

    python3 experiment_ctrl_gui/check_param_docs.py

The description is the `# comment` on the key's own line: what it is, with units. Longer reasoning
goes in comment lines directly above the key. The dashboard shows the description under each
parameter and both in its hover popup, so a key without one is a parameter nobody can look up.
Exit code 1 lists the undocumented keys.
"""
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from params import yaml_docs  # noqa: E402

CONFIG = HERE.parent / "sinthlab_bringup" / "config"


def key_paths(d: dict, prefix: str = ""):
    """Every mapping key as a dotted name, blocks included -- the same naming yaml_docs uses."""
    for k, v in d.items():
        if k == "ros__parameters" or k == "/**":
            yield from key_paths(v, prefix) if isinstance(v, dict) else ()
            continue
        name = f"{prefix}{k}"
        yield name
        if isinstance(v, dict):
            yield from key_paths(v, name + ".")


def main() -> int:
    bad = 0
    for path in sorted(CONFIG.glob("*.yaml")):
        text = path.read_text()
        desc, _ = yaml_docs(text)
        doc = yaml.safe_load(text) or {}
        # Node keys such as `/**/controller_manager` are the file's structure, not parameters.
        is_node_key = lambda n: n.startswith("/") and "." not in n
        missing = [n for n in key_paths(doc) if n not in desc and not is_node_key(n)]
        print(f"{'OK ' if not missing else 'BAD'} {path.name}" + (f": {len(missing)} undocumented" if missing else ""))
        for n in missing:
            print(f"      {n}")
        bad += len(missing)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
