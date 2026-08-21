# `ros2_effort_controller` — VENDORED third-party code

> **This directory is NOT sinthlab code.** It is a vendored copy of an external, third-party ROS 2
> package set. Everything here is owned and copyrighted by its original authors (below). We keep a
> local copy only because it carries small local patches (see "Local patches") that upstream does not have.

## Origin & ownership

| | |
|---|---|
| **Upstream repository** | <https://github.com/idra-lab/ros2_effort_controller> |
| **Vendored from commit** | `7c9b3faaf3c2ad61e77eab2306701a84ff88b822` (`main`, 2026-06-04) |
| **Authors / owners** | **IDRA Lab**, University of Trento — Luca Beber, Davide Nardi |
| **License** | Apache License 2.0 — see [`LICENSE`](LICENSE) (unmodified) |
| **Used via** | <https://github.com/idra-lab/kuka_lbr_control> (their KUKA integration + wiki) |

All original copyright notices and the upstream per-package `README.md` files are left intact.

## What is vendored (and what is not)

Copied from the upstream repo:

| Package | Why we need it |
|---|---|
| `effort_controller_base` | base class both impedance controllers build on (**patch #1**) |
| `debug_msg` | message package both controllers depend on |
| `cartesian_impedance_controller` | the **virtual-fixture** controller (restricted-plane + maze) |
| `joint_impedance_controller` | **move-to-start / recover** under FRI torque mode (**patch #2**) |

Deliberately **not** copied:

- `gravity_compensation` — not referenced by `sinthlab_bringup`, nothing here depends on it.
  (It is a handy "does the arm float?" diagnostic; copy it from upstream if you ever want it.)
- `README.md` (upstream root), `clang-format.txt`, `.github/`, `.gitignore` — repo scaffolding.

Everything these packages depend on beyond the above (`rclcpp`, `controller_interface`, `kdl_parser`,
`trajectory_msgs`, `pluginlib`, `builtin_interfaces`) is stock ROS 2 and comes from `rosdep`.

## Why vendored instead of imported via `vcs`

These packages were briefly imported through `sinthlab_lbr_stack.repos`. They are vendored instead
because they need the local patches below, and **the only repository we may modify is
`sinthlab-kuka-stack`** — patching an imported checkout would be silently discarded by the next
`vcs import`. The `.repos` file therefore no longer lists `ros2_effort_controller`; importing it
again alongside this copy would give colcon duplicate package names.

## Local patches

Each patch is marked in the source with a `>>> SINTHLAB PATCH` comment so it can be found and
re-applied when re-syncing with upstream.

### Patch #1 — Jazzy `get_value()` throws `std::bad_optional_access`

- **File:** `effort_controller_base/src/effort_controller_base.cpp`, `updateJointStates()`
- **Symptom:** on controller activation the controller_manager logged
  `Caught exception of type : St19bad_optional_access while updating controller
  'joint_impedance_controller': bad optional access`, deactivated the controller, and overran the
  control loop (~39 ms).
- **Cause:** upstream reads the joint state interfaces with `get_value()`, which in Jazzy
  `ros2_control` is `get_optional().value()` and **throws while an interface is still empty**. The
  joint *position* interface has a URDF `initial_value` so it is always populated; the *velocity*
  interface has none, so it stays `std::nullopt` until the FRI hardware `read()` delivers its first
  **commanding** frame. Since FRI only reaches commanding once a controller is active, the very first
  `update()` always ran against an empty velocity interface.
- **Fix:** read both via `get_optional()` and `continue` (keep the previous joint state) until values
  exist, so the first few updates are no-ops instead of an exception.
- **Note:** this is *not* fixable by delaying activation (we tried) — it is a deadlock: commanding
  needs an active controller, and the controller needs commanding-populated velocity.
- **Upstream status:** still present on `main` as of the vendored commit. Worth contributing back;
  if upstream fixes it, this patch can be dropped and the packages re-synced.

### Patch #2 — joint-space target for `joint_impedance_controller` (bypasses the IK round trip)

- **Files:** `joint_impedance_controller/{include/.../joint_impedance_controller.h, src/joint_impedance_controller.cpp}`,
  plus `std_msgs` added to its `package.xml` / `CMakeLists.txt`.
- **Symptom:** `Could not find IK solution` logged every control cycle and **the arm never moved**.
- **Cause:** `computeTorque()` calls `Base::computeIKSolution(m_target_frame, m_q_desired)` first. On
  failure that helper logs and returns *without touching* `m_q_desired`, which still holds the value
  captured at activation — so `tau = K*(q_activation - q) ~ 0`. Zero torque means the arm does not
  move, the next cycle solves the same IK from the same configuration, and it fails again: a
  deadlock. The solver itself (KDL `ChainIkSolverPos_NR_JL` + `ChainIkSolverVel_pinv`) is unreliable
  on this 7-DOF redundant chain and blows up near singular configurations.
- **Deeper issue:** our move-to-start already *knows* the goal joint configuration
  (`move_to_start.target_joint_position`). Going joints -> FK -> Cartesian pose -> IK -> joints is a
  pointless round trip that only adds a fragile solver.
- **Fix:** the controller now subscribes to `<controller>/target_joints`
  (`std_msgs/Float64MultiArray`, radians, one value per joint) and uses it directly as `m_q_desired`,
  skipping the IK. The upstream `target_frame` -> IK path is kept as a fallback and is used whenever
  nothing publishes on `/target_joints`, so upstream behaviour is unchanged for other users.
- **Gotcha (fixed):** `m_q_target` must be sized in `on_configure`, not `on_activate` — the
  `/target_joints` subscription goes live at the end of `on_configure`, so a message can arrive while
  the controller is still INACTIVE; writing into a default-constructed (size 0) Eigen vector
  segfaults. The callback also re-checks the size defensively.
- **ROS side:** `sinthlab_bringup`'s `MoveToJointTargetAction`
  (`actions/move_to_joint_target.py`) ramps joints and publishes them on that topic.
- **Upstream status:** upstream has no joint-space target path. Would need discussion before a PR.

## Re-syncing with upstream

```bash
git clone https://github.com/idra-lab/ros2_effort_controller /tmp/rec
# copy the four package directories over this folder, then re-apply the patches above
# (grep for "SINTHLAB PATCH" in the OLD copy first to see exactly what to port)
```
Update the commit in the table above whenever you re-sync.
