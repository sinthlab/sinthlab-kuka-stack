# Reward calibration, 2026-11-23: gravity-fed reservoir (provisional)

Measured on the rig with `analysis/reward_calibration.py` (sweep, then `fit --model interp`), valve on cabinet output 4 (SYNC 4), juice (density taken as 1.0 g/mL), weighed per batch of 10 rewards, 1 s between rewards (1.5 s for 3-pulse bursts). **Provisional:** the reservoir is gravity-fed, so the flow falls as it empties; redo the sweep once the pressure system is in, in a new folder `analysis/reward_calibrations/<date>_<setup>/` (the dashboard uses the newest folder with a fitted profile). The dashboard's Reward calibration page does the sweep, the fit and the pre-session check with buttons.

The dashboard (`experiment_ctrl_gui/reward_volume.py`) reads the `*.json` profiles here and shows each reward in mL, the juice given in a run, and lets the reward be set by volume.

| Reward | Profile | Calibrated widths | mL per reward | Spread (CV) |
|---|---|---|---|---|
| 1 pulse | `single.json` | 100–400 ms | 0.78–1.69 | ≤ 2.4 % |
| 2 pulses, 100 ms apart | `burst2.json` | 30–100 ms each | 0.81–1.35 | ≤ 2.1 % |
| 3 pulses, 100 ms apart | `burst3.json` | 20–50 ms each | 1.01–1.48 | ≤ 1.7 % |

Mean volume per reward, for reference: single pulse 100 / 150 / 200 / 300 / 400 ms = 0.78 / 0.92 / 1.06 / 1.36 / 1.69 mL; 2 pulses 30 / 50 / 75 / 100 ms = 0.81 / 1.00 / 1.18 / 1.35 mL; 3 pulses 20 / 30 / 50 ms = 1.01 / 1.17 / 1.48 mL.

Things to know:

- **Every opening costs about 0.45 mL** before the width counts (the fits' intercepts: 0.46 mL for one pulse, 0.60 for two, 0.70 for three): a 30 ms single pulse still gave 0.5 mL in the pilot. So more pulses give more juice for the same valve time, and the smallest reward is about 0.5 mL.
- **The reservoir level matters**: the pilot (`pilot.csv`, earlier, fuller reservoir, 20 rewards per batch) gave 0.82–0.87 mL at 100 ms against 0.78 mL here, 5–10 % more. Read these volumes as ±10 %.
- **Bursts were calibrated with a 100 ms gap**; the task YAMLs' default `reward_pulse.gap_ms` is 300. The dashboard warns when a burst's gap differs from its calibration.
- Widths outside a profile's range are extrapolated along its straight-line fit and marked as such.
