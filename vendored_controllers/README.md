# Vendored KUKA LBR Controllers

This directory contains vendored copies of real-time ROS 2 controllers for the KUKA LBR series,
developed by the [IDRA Lab](https://idra-lab.dii.unitn.it/) (University of Trento).

## Included Packages
- `kuka_clik_controller`
- `controller_base`
- `debug_msg`

## Provenance
Vendored from **[`idra-lab/ros2_effort_controller`](https://github.com/idra-lab/ros2_effort_controller)**,
branch **`kuka-prop-ctrl`** — that repository is embedded as the `controllers` submodule of
[`idra-lab/kuka_lbr_control`](https://github.com/idra-lab/kuka_lbr_control).

Copied ~2026-06-01. When re-syncing from upstream, record the exact upstream commit here so the
next update is a clean diff.

## License
The upstream repository is licensed under the **Apache License 2.0** — see [`LICENSE`](./LICENSE),
copied verbatim from upstream and covering all packages in this directory.

> Note: some upstream `package.xml` files declare a different license name (e.g. `BSD`). Those tags
> are kept verbatim as vendored, but the repository `LICENSE` (Apache 2.0) is the governing license.

## Local modifications (re-apply if re-vendoring)
- `kuka_clik_controller/src/kuka_clik_controller.cpp` — `on_init()` now `auto_declare`s the six
  CLIK parameters (`max_linear_velocity`, `max_angular_velocity`, `clik_dt`, `clik_it_max`,
  `clik_eps`, `clik_filter_alpha`) before reading them. Upstream read them via `get_parameter`
  without declaring, which threw `ParameterNotDeclaredException` at controller init under ROS 2 Jazzy.
- `controller_base/src/controller_base.cpp` — `on_init()` now `auto_declare`s `command_interfaces`
  (was commented out) and `nullspace_desired_configuration` (read in `on_configure` but never
  declared → `ParameterNotDeclaredException`). Also dedupes the `HW_IF_POSITION` append in
  `on_configure` so a config that already lists `position` doesn't claim that interface twice.

## Credits
Original authors: Luca Beber, Davide Nardi, et al. (IDRA Lab, University of Trento).
