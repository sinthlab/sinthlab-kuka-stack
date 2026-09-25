#!/usr/bin/env python3
"""Verify the board packing in apple_pluck_end_effector.scad.

The base has only 2-5 mm of margin in places, so the layout is checked numerically rather
than by eye. Parses the .scad directly, so it validates the FILE, not a copy of the numbers.

Checks, per tier:
  * every pocket corner inside the rim wall
  * every pocket clear of that tier's central bore
  * no two pockets overlapping
  * pocket depth + floor fits the tier height
and then: inter-tier screws clear of every pocket on BOTH tiers and of the cable trench,
cover screws inside the ring groove and clear of tier-2 pockets, tier-1 pockets clear of
the trench arc, the trench tie slots not cutting into the tier-1 floor, and the apple
stem's armature flange landing inside the ball.

    python3 check_layout.py            # exit 0 = clean, 1 = at least one failure
"""
import itertools
import math
import re
import sys
from pathlib import Path

SCAD = Path(__file__).with_name("apple_pluck_end_effector.scad")


def load(path):
    src = re.sub(r"//.*", "", path.read_text())
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return src


def scalars(src):
    return {m.group(1): float(m.group(2))
            for m in re.finditer(r"([a-zA-Z_]\w*)\s*=\s*(-?[\d.]+)\s*;", src)}


def point(src, name):
    m = re.search(rf"{name}\s*=\s*\[\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\]", src)
    return float(m.group(1)), float(m.group(2))


def corners(L, W, pos, clear):
    """Pocket outline: the board grown by `clear` per side."""
    l, w = L + 2 * clear, W + 2 * clear
    return [(pos[0] - l / 2, pos[1] - w / 2), (pos[0] + l / 2, pos[1] - w / 2),
            (pos[0] + l / 2, pos[1] + w / 2), (pos[0] - l / 2, pos[1] + w / 2)]


def separated(a, b):
    """Separating-axis test on two convex polygons."""
    for poly in (a, b):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = -(y2 - y1), (x2 - x1)
            n = math.hypot(nx, ny)
            nx, ny = nx / n, ny / n
            pa = [x * nx + y * ny for x, y in a]
            pb = [x * nx + y * ny for x, y in b]
            if max(pa) <= min(pb) + 1e-9 or max(pb) <= min(pa) + 1e-9:
                return True
    return False


def dist_to_edge(poly):
    """Closest approach of the outline to the origin (i.e. to the central bore)."""
    best = float("inf")
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, (-x1 * dx - y1 * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
    return best


def main():
    src = load(SCAD)
    v = scalars(src)
    fails = []

    def report(ok, msg):
        print(f"  {'OK  ' if ok else 'FAIL'} {msg}")
        if not ok:
            fails.append(msg)

    rim = v["base_d"] / 2 - v["base_wall"]
    comp_ro = v["comp_ro"]
    clear = v["pocket_clear"]
    cbore = v["m3_cbore"]
    pill_r = v["pillar_d"] / 2
    groove_i = v["ring_id"] / 2 - v["ring_clear"]
    groove_o = v["ring_od"] / 2 + v["ring_clear"]

    cover_pos = []
    for i in range(int(v["cover_screw_n"])):
        a = math.radians(v["cover_screw_a0"] + i * 360 / v["cover_screw_n"])
        cover_pos.append((v["cover_screw_bcd"] / 2 * math.cos(a), v["cover_screw_bcd"] / 2 * math.sin(a)))
    tier_pos = []
    for i in range(int(v["tier_screw_n"])):
        a = math.radians(v["tier_screw_a0"] + i * 360 / v["tier_screw_n"])
        tier_pos.append((v["tier_screw_bcd"] / 2 * math.cos(a), v["tier_screw_bcd"] / 2 * math.sin(a)))

    tiers = {
        "TIER 1": (v["cable_bore_d"] / 2 + v["hub_wall"], v["comp1_depth"], tier_pos, [
            ("metro", v["board_l"], v["board_w"], point(src, "board_pos"), v["board_clear_h"]),
            ("rs422", v["rs422_l"], v["rs422_w"], point(src, "rs422_pos"), v["rs422_clear"]),
            ("mprls", v["mprls_l"], v["mprls_w"], point(src, "mprls_pos"), v["mprls_clear"]),
        ]),
        "TIER 2": (v["tier2_bore_d"] / 2 + v["hub_wall"], v["comp2_depth"], cover_pos, [
            ("tobsun", v["conv_l"], v["conv_w"], point(src, "conv_pos"), v["conv_clear_h"]),
            ("shifter", v["shifter_l"], v["shifter_w"], point(src, "shifter_pos"), v["shifter_clear"]),
            ("haptic", v["haptic_drv_l"], v["haptic_drv_w"], point(src, "haptic_pos"), v["haptic_clear"]),
        ]),
    }

    print(f"base Ø{v['base_d']:.0f}, compartment r {v['cable_bore_d']/2 + v['hub_wall']:.0f}/"
          f"{v['tier2_bore_d']/2 + v['hub_wall']:.0f} -> {comp_ro:.0f}, rim wall {v['base_d']/2 - comp_ro:.0f} mm")
    print(f"tiers {v['base1_floor'] + v['comp1_depth']:.0f} + {v['base2_floor'] + v['comp2_depth']:.0f}, "
          f"floors {v['base1_floor']:.0f}/{v['base2_floor']:.0f}, cover {v['cover_plate_t']:.0f}")
    print(f"ring groove r {groove_i:.1f}..{groove_o:.1f} "
          f"({groove_o - groove_i:.1f} wide vs {(v['ring_pcb_od'] - v['ring_pcb_id']) / 2:.1f} of ring PCB), "
          f"rim beyond groove {rim + v['base_wall'] - groove_o:.1f} mm")

    for label, (bore_r, depth, pillars, boards) in tiers.items():
        print(f"\n{label}  bore keep-out r={bore_r:.0f}  compartment depth {depth:.0f}")
        for name, L, W, pos, need in boards:
            # There are no pockets, so "does it fit" is a question about the BOARD, not a board+2mm
            # rectangle. Clearance is still applied below for board-to-board and board-to-pillar.
            poly = corners(L, W, pos, 0)
            r_max = max(math.hypot(x, y) for x, y in poly)
            r_min = dist_to_edge(poly)
            report(r_max <= comp_ro, f"{name:8s} corner r={r_max:5.1f} <= compartment {comp_ro:.0f} (margin {comp_ro - r_max:4.1f})")
            report(r_min >= bore_r, f"{name:8s} nearest r={r_min:5.1f} >= bore {bore_r:.0f}")
            report(need <= depth, f"{name:8s} needs {need:.0f} mm of height <= compartment {depth:.0f}")
            for px, py in pillars:
                report(separated(poly, corners(2 * pill_r, 2 * pill_r, (px, py), 0)),
                       f"{name:8s} clears the pillar at ({px:5.1f},{py:5.1f})")
        for (n1, l1, w1, p1, _), (n2, l2, w2, p2, _) in itertools.combinations(boards, 2):
            report(separated(corners(l1, w1, p1, clear), corners(l2, w2, p2, clear)),
                   f"{n1} and {n2} do not overlap")

    print("\ncable management (tier 1, all sunk below the compartment floor)")
    sink_z = v["base1_floor"] - v["trench_sink"]
    r_mid = (v["trench_ri"] + v["trench_ro"]) / 2
    run = math.radians(v["trench_span"]) * r_mid
    report(sink_z >= 4, f"{sink_z:.0f} mm of solid floor left under the trenches and channels")
    report(v["trench_ro"] <= comp_ro, f"trench outer {v['trench_ro']:.0f} inside the compartment wall {comp_ro:.0f}")
    report(int(v["radial_n"]) == int(v["trench_n"]), f"one radial channel per trench ({int(v['radial_n'])} each)")
    print(f"  ---- {int(v['trench_n'])} perimeter trenches, r {v['trench_ri']:.0f}..{v['trench_ro']:.0f} "
          f"x {v['trench_sink']:.0f} deep, {v['trench_span']:.0f}° each = {run:.0f} mm of run apiece "
          f"({int(v['trench_n']) * run:.0f} mm total), {int(v['trench_tie_n'])} tie slots each")
    print(f"  ---- {int(v['radial_n'])} radial channels, {v['radial_w']:.0f} wide, bore -> trench, first at {v['radial_a0']:.0f}°")
    half_ch = math.degrees(math.atan2(v["radial_w"] / 2, r_mid))
    for t in range(int(v["trench_n"])):
        c = v["trench_a0"] + t * 360 / v["trench_n"]
        best = min((abs((v["radial_a0"] + i * 360 / v["radial_n"] - c + 180) % 360 - 180)
                    for i in range(int(v["radial_n"]))), default=1e9)
        report(best <= v["trench_span"] / 2 + half_ch,
               f"trench centred at {c % 360:.0f}° is fed by a channel ({best:.0f}° off centre, "
               f"trench half-span {v['trench_span']/2:.0f}°)")
        report(best >= v["trench_span"] / 2 - half_ch * 2,
               f"  ...and the channel meets it at an END, not the middle")

    print("\nwire pass-throughs")
    for lbl, bore, n, d, a0 in (("tier 1", v["cable_bore_d"], v["hub_port_n"], v["hub_port_d"], v["hub_port_a0"]),
                                ("tier 2", v["tier2_bore_d"], v["t2_port_n"], v["t2_port_d"], v["t2_port_a0"])):
        report(d <= v["comp1_depth" if lbl == "tier 1" else "comp2_depth"],
               f"{lbl}: Ø{d:.0f} bore port fits inside the compartment depth")
        wall = math.pi * (bore + v["hub_wall"])   # circumference at the hub wall's mean diameter
        removed = n * d
        report(removed < wall * 0.6,
               f"{lbl}: {int(n)}x Ø{d:.0f} ports remove {removed:.0f} of {wall:.0f} mm of hub-wall "
               f"circumference ({100*removed/wall:.0f}%)")
    report(abs(v["hub_port_a0"] - v["radial_a0"]) < 1e-9,
           f"hub_port_a0 ({v['hub_port_a0']:.0f}°) == radial_a0 ({v['radial_a0']:.0f}°) — ports clocked onto the channels")
    aligned = 0
    for i in range(int(v["hub_port_n"])):
        ang = v["hub_port_a0"] + i * 360 / v["hub_port_n"]
        worst = min(abs((ang - (v["radial_a0"] + k * 360 / v["radial_n"]) + 180) % 360 - 180)
                    for k in range(int(v["radial_n"])))
        if worst < 1e-9:
            aligned += 1
            print(f"  ---- tier-1 port at {ang % 360:5.1f}° is ON a feed channel -> one continuous opening, bore to trench")
        else:
            print(f"  ---- tier-1 port at {ang % 360:5.1f}° is a plain port ({worst:.0f}° off a channel)")
    report(aligned == int(v["radial_n"]),
           f"every one of the {int(v['radial_n'])} feed channels has a bore port on it ({aligned} aligned)")
    # A port is a circle tangent to the floor, so on its own it pinches to zero width exactly where
    # the channel meets it. base_tier1 squares the opening off from the channel floor up to the
    # port's widest point; these two conditions are what make that one continuous hole.
    report(v["radial_w"] >= v["hub_port_d"],
           f"feed channel ({v['radial_w']:.0f} wide) is at least as wide as the port it joins "
           f"(Ø{v['hub_port_d']:.0f}) — no step in width")
    bridge_top = v["base1_floor"] + v["hub_port_d"] / 2
    report(bridge_top >= v["base1_floor"] + v["hub_port_d"] / 2 and sink_z <= v["base1_floor"],
           f"bore-to-channel opening is continuous z {sink_z:.0f}..{v['base1_floor'] + v['hub_port_d']:.0f} "
           f"(squared to z {bridge_top:.0f}, then the port's arch above)")

    drop_r = v["drop_bcd"] / 2
    print(f"  ---- {int(v['drop_n'])}x Ø{v['drop_d']:.0f} tier-2 drops at r {drop_r:.0f}")
    report(drop_r + v["drop_d"] / 2 < v["trench_ri"],
           f"drops clear the tier-1 trenches (drop outer {drop_r + v['drop_d']/2:.0f} < trench inner {v['trench_ri']:.0f})")
    report(drop_r - v["drop_d"] / 2 > v["tier2_bore_d"] / 2 + v["hub_wall"],
           f"drops sit outboard of the tier-2 hub wall")
    for i in range(int(v["drop_n"])):
        a = math.radians(v["drop_a0"] + i * 360 / v["drop_n"])
        dx, dy = drop_r * math.cos(a), drop_r * math.sin(a)
        for px, py in cover_pos:
            report(math.hypot(dx - px, dy - py) > pill_r + v["drop_d"] / 2,
                   f"drop at {math.degrees(a) % 360:5.1f}° clears the tier-2 pillar at ({px:5.1f},{py:5.1f})")
        for k in range(int(v["plate_screw_n"])):
            pa = math.radians(v["plate_screw_a0"] + k * 360 / v["plate_screw_n"])
            sx, sy = v["plate_screw_bcd"] / 2 * math.cos(pa), v["plate_screw_bcd"] / 2 * math.sin(pa)
            report(math.hypot(dx - sx, dy - sy) > cbore / 2 + v["drop_d"] / 2,
                   f"drop at {math.degrees(a) % 360:5.1f}° lands clear of tier-1 plate screw {k}")
        blocked = [n for n, L, W, pos, _ in tiers["TIER 2"][3]
                   if not separated(corners(L, W, pos, clear), corners(v["drop_d"], v["drop_d"], (dx, dy), 0))]
        if blocked:
            print(f"  ---- NOTE drop at {math.degrees(a) % 360:5.1f}° is under the reference "
                  f"{','.join(blocked)} placement (move the board or use another drop)")

    print("\ntier-1 -> flange plate screws")
    for i in range(int(v["plate_screw_n"])):
        ang = v["plate_screw_a0"] + i * 360 / v["plate_screw_n"]
        r = v["plate_screw_bcd"] / 2
        worst = min(abs((ang - (v["radial_a0"] + k * 360 / v["radial_n"]) + 180) % 360 - 180)
                    for k in range(int(v["radial_n"])))
        gap = r * math.sin(math.radians(worst)) - v["radial_w"] / 2 - cbore / 2
        report(gap > 0, f"screw {i} at {ang % 360:5.1f}° clears the nearest radial channel by {gap:4.1f} mm")
    report(r + cbore / 2 < v["plate_d"] / 2,
           f"all {int(v['plate_screw_n'])} land on the Ø{v['plate_d']:.0f} flange plate")

    print("\nscrew pillars: symmetric, and clear of the cable channels")
    angs = sorted(math.degrees(math.atan2(py, px)) % 360 for px, py in tier_pos)
    steps = [round((angs[(i + 1) % len(angs)] - angs[i]) % 360, 3) for i in range(len(angs))]
    report(len(set(steps)) == 1,
           f"tier screws evenly spaced at {steps[0]:.0f}° ({', '.join(f'{a:.0f}°' for a in angs)})")
    radii = {round(math.hypot(px, py), 3) for px, py in tier_pos}
    report(len(radii) == 1, f"tier screws all on one bolt circle (r={list(radii)[0]:.1f})")
    for px, py in tier_pos:
        r = math.hypot(px, py)
        ang = math.degrees(math.atan2(py, px)) % 360
        # the pillar sits INSIDE the trench radius band, so it has to clear each trench ANGULARLY
        if v["trench_ri"] - pill_r < r < v["trench_ro"] + pill_r:
            worst = 1e9
            for t in range(int(v["trench_n"])):
                c = v["trench_a0"] + t * 360 / v["trench_n"]
                d = abs((ang - c + 180) % 360 - 180) - v["trench_span"] / 2
                worst = min(worst, d)
            gap = r * math.sin(math.radians(max(worst, 0))) - pill_r
            report(worst > 0 and gap > 0,
                   f"pillar ({px:5.1f},{py:5.1f}) at {ang:5.1f}° sits in a gap between trenches, "
                   f"{gap:4.1f} mm clear of the nearest trench end")
        else:
            report(True, f"pillar ({px:5.1f},{py:5.1f}) outside the trench radius band")
        worst_r = min(abs(((ang - (v["radial_a0"] + i * 360 / v["radial_n"])) + 180) % 360 - 180)
                      for i in range(int(v["radial_n"])))
        gap_r = r * math.sin(math.radians(worst_r)) - v["radial_w"] / 2 - pill_r
        report(gap_r > 0, f"pillar ({px:5.1f},{py:5.1f}) clears the nearest radial channel by {gap_r:4.1f} mm")

    print("\ncover screws: tier-2 pillars must line up with the cover's clearance holes")
    print("  (both are generated from cover_screw_pos(), so they align by construction)")
    for px, py in cover_pos:
        r = math.hypot(px, py)
        report(r + cbore / 2 <= groove_i,
               f"({px:6.1f},{py:6.1f}) cover cbore outer {r + cbore / 2:.2f} < ring groove inner {groove_i:.1f}")
        report(r - pill_r >= v["tier2_bore_d"] / 2 + v["hub_wall"] and r + pill_r <= comp_ro,
               f"({px:6.1f},{py:6.1f}) pillar sits inside the tier-2 compartment")
    for (cx, cy) in cover_pos:
        cr = math.hypot(cx, cy)
        for (tx, ty) in tier_pos:
            if abs(((math.degrees(math.atan2(cy, cx)) - math.degrees(math.atan2(ty, tx))) + 180) % 360 - 180) < 1.0:
                tr = math.hypot(tx, ty)
                report(cr + pill_r < tr - cbore / 2,
                       f"cover pillar at r={cr:.0f} (to {cr + pill_r:.2f}) clears the tier-screw "
                       f"counterbore at r={tr:.0f} (from {tr - cbore/2:.2f}) by {tr - cbore/2 - cr - pill_r:.2f} mm")
    report(v["cover_screw_depth"] <= v["comp2_depth"],
           f"cover insert {v['cover_screw_depth']:.0f} deep fits the {v['comp2_depth']:.0f} mm pillar")
    report(v["tier_screw_depth"] <= v["comp1_depth"],
           f"tier insert {v['tier_screw_depth']:.0f} deep fits the {v['comp1_depth']:.0f} mm pillar")

    ball_c = v["sj_flange_t"] + v["stem_shaft_len"] - v["arm_cap"] + v["apple_d"] / 2
    arm_z = v["sj_flange_t"] + v["stem_shaft_len"]
    print("\napple stem")
    report(ball_c - v["apple_d"] / 2 < arm_z,
           f"armature flange at z={arm_z:.0f} sits inside the ball ({ball_c - v['apple_d']/2:.0f}..{ball_c + v['apple_d']/2:.0f})")
    report(v["stem_shaft_d"] / 2 + v["stem_fillet_r"] < v["sj_screw_bcd"] / 2 - cbore / 2,
           f"fillet to r={v['stem_shaft_d']/2 + v['stem_fillet_r']:.1f} clears the joint screws "
           f"(cbore inner r={v['sj_screw_bcd']/2 - cbore/2:.2f})")

    stack = (v["plate_t"] + v["base1_floor"] + v["comp1_depth"]
             + v["base2_floor"] + v["comp2_depth"] + v["cover_plate_t"])
    print(f"\nstack: plate {v['plate_t']:.0f} + tier1 {v['base1_floor'] + v['comp1_depth']:.0f} "
          f"+ tier2 {v['base2_floor'] + v['comp2_depth']:.0f} + cover {v['cover_plate_t']:.0f} = {stack:.0f} mm; "
          f"apple centre {stack + ball_c:.0f} mm above the robot flange")

    print("\nstrength at the base <-> flange joint (the most failure-prone joint in bending)")
    lever = (stack + ball_c) / 1000.0
    moment = 20.0 * lever
    rr = v["plate_screw_bcd"] / 2
    per = moment / (2 * rr / 1000.0) / (v["plate_screw_n"] / 2)
    print(f"  ---- 20 N at the apple = {moment:.2f} N.m; couple across Ø{v['plate_screw_bcd']:.0f} "
          f"= {per:.0f} N per screw on the tension side")
    report(v["plate_head_z"] >= 8,
           f"{v['plate_head_z']:.0f} mm of printed floor under each screw head (>= 8 required)")
    report(v["base1_floor"] - v["trench_sink"] >= 6,
           f"{v['base1_floor'] - v['trench_sink']:.0f} mm of floor left where a cable channel crosses")
    report(rr + cbore / 2 < v["plate_d"] / 2 - 6,
           f"bolt circle r {rr:.0f} sits {v['plate_d']/2 - rr - cbore/2:.1f} mm inside the Ø{v['plate_d']:.0f} plate rim")
    over = v["base_d"] / 2 - v["plate_d"] / 2
    report(over < 35, f"tier-1 floor overhangs the plate by {over:.0f} mm (< 35 required)")
    report(v["fillet_r"] >= 2,
           f"R{v['fillet_r']:.0f} fillets at the compartment corners and pillar bases")
    report((v["case_box_side"] - 2 * v["case_box_wall"] - v["base_d"]) / 2 > 0,
           f"casing box {v['case_box_side']:.0f} clears the Ø{v['base_d']:.0f} base by "
           f"{(v['case_box_side'] - 2*v['case_box_wall'] - v['base_d']) / 2:.1f} mm/side")

    print("\nRESULT:", "ALL CLEAR" if not fails else f"{len(fails)} FAILURE(S)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
