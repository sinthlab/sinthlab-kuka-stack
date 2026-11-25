"""The early-touch cue (helpers/early_touch_flash.py) and the quick park after an early touch.

    python3 -m pytest sinthlab_bringup/test
"""
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sinthlab_bringup"))

from sinthlab_bringup.helpers import early_touch_flash as etf  # noqa: E402
from sinthlab_bringup.helpers import live_params as lp  # noqa: E402

CONFIGS = sorted(p for p in (ROOT / "sinthlab_bringup" / "config").glob("*.yaml")
                 if "move_to_park:" in p.read_text())


def params(path):
    doc = yaml.safe_load(path.read_text())
    return next(iter(doc.values()))["ros__parameters"]


def test_two_flashes_end_dark():
    assert etf.schedule(2, 0.15) == [(0.0, True), (0.15, False), (0.3, True), (0.45, False)]
    assert etf.schedule(0, 0.15) == []
    assert etf.schedule(1, 0.0) == [(0.0, True), (0.02, False)]       # never zero-length


def test_colour_and_the_board_request():
    assert etf.colour([255, 200, 0]) == (255, 200, 0, 0)
    assert etf.colour([300, -5, 0, 0]) == (255, 0, 0, 0)
    assert etf.colour([0, 0, 0, 0]) is None and etf.colour(None) is None and etf.colour(["x"]) is None
    assert etf.ring_path((255, 200, 0, 0)) == "/segments?factor=4&colors=" + ".".join(["255,200,0,0"] * 4)


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.name)
def test_every_task_has_the_cue_and_the_quick_retreat(path):
    p = params(path)
    q, park = p["quiet_gate"], p["move_to_park"]
    rgbw = etf.colour(q["flash_colour"])
    assert rgbw is not None and max(rgbw) == 255          # full value: visual_cue.brightness alone dims it
    assert q["flashes"] >= 1 and lp.check_value("quiet_gate.flashes", q["flashes"]) is None
    assert lp.check_value("quiet_gate.flash_sec", q["flash_sec"]) is None
    assert lp.is_live(path_key(path), "quiet_gate.flash_colour")
    assert lp.is_live(path_key(path), "move_to_park.early_touch_speed_scale")
    assert lp.check_value("move_to_park.early_touch_speed_scale", park["early_touch_speed_scale"]) is None
    assert park["early_touch_speed_scale"] >= park["speed_scale"]
    # The park move itself is quicker than the start move's limits (the arm leaves the hand promptly).
    assert park["move_to_pos_a_max"] >= 4.0 and park["move_to_pos_j_max"] >= 20.0


def test_the_flash_colour_is_not_another_cue_colour():
    """Yellow means 'too early' only: no other cue in any task uses that colour."""
    for path in CONFIGS:
        p = params(path)
        flash = tuple(p["quiet_gate"]["flash_colour"])
        for block in ("ring_cue", "visual_cue"):
            for name, c in ((p.get(block) or {}).get("colours") or {}).items():
                assert tuple(c) != flash, (path.name, block, name)



def path_key(path):
    """The experiment a config belongs to (its key in helpers/live_params.LIVE_PARAMS)."""
    sys.path.insert(0, str(ROOT / "experiment_ctrl_gui"))
    ex = pytest.importorskip("experiments")
    return next(e.key for e in ex.EXPERIMENTS if Path(e.params_yaml).name == path.name)


# ---------------------------------------------------------------- the ROS side, with stand-ins for ROS
def _ros_stand_ins():
    import importlib
    import types

    def stub(name, **attrs):
        if name in sys.modules:                  # another test's stand-in: add what this one needs
            for k, v in attrs.items():
                if not hasattr(sys.modules[name], k):
                    setattr(sys.modules[name], k, v)
            return
        try:
            importlib.import_module(name)
            return
        except ImportError:
            pass
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod

    stub("rclpy")
    stub("rclpy.node", Node=object)
    stub("optas")
    stub("ruckig", InputParameter=object, OutputParameter=object, Ruckig=object)
    stub("lbr_fri_idl")
    stub("lbr_fri_idl.msg", LBRState=object, LBRJointPositionCommand=object)


class _Param:
    def __init__(self, value):
        self.value = value


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = error = info


class _Node:
    def __init__(self, **params):
        self.params, self.log = params, _Log()

    def has_parameter(self, n):
        return n in self.params

    def get_parameter(self, n):
        return _Param(self.params[n])

    def get_logger(self):
        return self.log


class _Sender:
    def __init__(self):
        self.sent = []

    def show(self, path, label):
        self.sent.append((path, label))


def test_go_cue_flashes_the_ring_yellow_twice_then_dark(monkeypatch):
    _ros_stand_ins()
    from sinthlab_bringup.actions import go_cue as gc

    node = _Node(**{"visual_cue.enabled": True, "quiet_gate.flash_colour": [255, 200, 0, 0],
                    "quiet_gate.flashes": 2, "quiet_gate.flash_sec": 0.15})
    sender = _Sender()
    monkeypatch.setattr(gc.VisualCue, "sender", staticmethod(lambda n: sender))
    clock = [100.0]
    monkeypatch.setattr(gc.time, "monotonic", lambda: clock[0])
    cue = gc.GoCue.__new__(gc.GoCue)                 # without __init__'s timers and audio warm-up
    cue._node, cue._flash, cue._flash_path = node, [], ""
    cue._audio = type("A", (), {"enabled": lambda self: False})()
    cue.warn_early_touch()
    yellow = etf.ring_path((255, 200, 0, 0))
    assert sender.sent == [(yellow, "early_touch_on")]            # the first flash at once
    for t in (0.1, 0.16, 0.31, 0.46):
        clock[0] = 100.0 + t
        cue._step_flash()
    assert [s[0] for s in sender.sent] == [yellow, "/off", yellow, "/off"]   # ends dark
    node.params["visual_cue.enabled"] = False
    sender.sent.clear()
    cue.warn_early_touch()
    assert sender.sent == []                                       # ring off: no flashes
    node.params.update({"visual_cue.enabled": True, "quiet_gate.flashes": 0})
    cue.warn_early_touch()
    assert sender.sent == []


def test_park_after_an_early_touch_is_quicker_for_that_move_only(monkeypatch):
    _ros_stand_ins()
    from sinthlab_bringup.actions import park_move as pm

    scales = []
    monkeypatch.setattr(pm.MoveToPositionJointSpace, "start", lambda self: scales.append(self._speed_scale))
    park = pm.ParkMove.__new__(pm.ParkMove)             # without __init__'s robot model and pose checks
    park._node = _Node(**{"move_to_park.early_touch_speed_scale": 1.25})
    park._speed_scale, park._penalty, park._retreat, park._scale_after = 1.0, 0.0, False, None
    park._on_first, park._after = None, lambda: None
    park.start()
    park.penalize(5.0)                                   # an early touch
    park.start()
    park.set_speed_scale(0.8)                            # changed on the dashboard during the retreat
    park._done()
    park.start()
    assert scales == [1.0, 1.25, 0.8]
    # A retreat cut short (Stop: no _done) does not leave the next move quick.
    park.penalize(5.0)
    park.start()
    park.start()
    assert scales[-2:] == [1.25, 0.8]
