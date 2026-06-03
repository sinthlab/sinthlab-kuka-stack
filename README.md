# Sinthlab KUKA Stack

A ROS 2 (Jazzy) control stack for running **compliant "apple‑pluck" experiments** on a KUKA
LBR iiwa7. It is built as an *overlay* on top of the [`lbr_fri_ros2_stack`][^1], adding the
high‑level experiment orchestration, motion actions, and the KUKA Sunrise (FRI) applications
needed to drive the arm.

In every hardware scenario the **Cartesian compliance ("spring") runs on the KUKA cabinet at
1000 Hz**, while ROS 2 streams a moving target pose to it through the `kuka_clik_controller`.
Python state machines sequence each trial (move → cue → monitor displacement → recoil → repeat).

## Table of Contents
- [1. Hardware Setup (KUKA Arm)](#1-hardware-setup-kuka-arm)
- [2. Windows Laptop Setup](#2-windows-laptop-setup)
- [3. Building the Stack](#3-building-the-stack)
- [4. Simulation & Visualization (no hardware)](#4-simulation--visualization-no-hardware)
- [5. Running Experiments on Hardware](#5-running-experiments-on-hardware)
- [6. Software Architecture](#6-software-architecture)
- [7. Troubleshooting](#7-troubleshooting)
- [8. Development & Contributing](#8-development--contributing)
- [Acknowledgement](#acknowledgement)

---

## 1. Hardware Setup (KUKA Arm)
- Use the "Quick Start guide" to cable up the Arm, Cabinet and the SmartPad.
- For the Safety on the X11 interface in the Cabinet:
  - First create 6 jumper cables with the provided pins.
  - Based on the PIN diagram (pg‑53 of `Spez_KUKA_Sunrise_Cabinet_en.pdf`, section 6.6.2),
    jump pins 1/2, 10/11 (external E‑Stop), 3/4, 12/13 (Operator Safety) and 5/6, 14/15
    (Safety stop 1).

---

## 2. Windows Laptop Setup
> **Why Windows + WSL2?** KUKA's robot software (Sunrise Workbench) requires Windows, while our
> stack requires Ubuntu. We use the Windows laptop to install applications onto the robot, and
> run our stack inside WSL2 (Ubuntu) on the same laptop to control the arm.

### Prerequisites
- Install Ubuntu 24.04 with `wsl --install -d Ubuntu-24.04` (the version matters — do **not**
  install the default).
- Confirm you are on WSL version 2 (`wsl -l -v` shows the version and the installed distro).
- Make sure the robot controller box is on.
- Install `Sunrise Workbench` on the laptop. For our arm version we use
  `SunriseWorkbench-1.17.0.4-setup.exe`, provided by KUKA Support.
- Install the FRI plugin in the Sunrise Workbench project (steps to be elaborated).

### Connection Verification
- Connect your computer to the robot controller at **X66** (default IP `172.31.1.147`) via an
  ethernet cable.
- Put your computer on the same network — set your IP to `172.31.1.148` (or anything else in the
  `/16` subnet).
  - On Windows: Settings → Network & Internet → Ethernet, edit the IPv4 manual setup to IP
    `172.31.1.148` and gateway `255.255.0.0`; leave other options as is and save.
- Ping the robot from PowerShell — `ping 172.31.1.147` — and expect something like:
  ```
  PING 172.31.1.147 (172.31.1.147) 56(84) bytes of data.
  64 bytes from 172.31.1.147: icmp_seq=1 ttl=64 time=0.868 ms
  ```
- Enable Hyper‑V on the laptop.
- Set up networking for WSL2: open WSL settings (in the Start menu) and change the Networking
  mode to **Mirrored**.
  <img width="1582" height="630" alt="image" src="https://github.com/user-attachments/assets/fe185222-e883-4979-a1f6-e80539cf166a" />
- In a PowerShell window **with admin privileges**, allow inbound Hyper‑V connections:
  ```powershell
  Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
  ```

### Install Application to Robot
Follow [these steps](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html#install-applications-to-the-robot)
to install the application to the robot.

---

## 3. Building the Stack
> These steps run inside **WSL2 / Ubuntu 24.04**.

1. Install ROS 2 development tools (see this [guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for details):
   ```bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
   curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
   sudo dpkg -i /tmp/ros2-apt-source.deb
   sudo apt update && sudo apt install ros-dev-tools
   sudo apt update
   sudo apt upgrade
   sudo apt install ros-jazzy-desktop
   sudo apt install python3-pip
   ```
2. Create a workspace, clone, and install dependencies:
   ```bash
   source /opt/ros/jazzy/setup.bash
   mkdir -p lbr-stack/src && cd lbr-stack
   vcs import src --input https://raw.githubusercontent.com/sinthlab/sinthlab-kuka-stack/main/sinthlab_lbr_stack.repos
   rosdep install --from-paths src -i -r -y
   ```
3. Install the required Python packages (only needed once per WSL session install):
   ```bash
   pip install pyoptas ruckig --break-system-packages
   ```
4. Build:
   ```bash
   colcon build --symlink-install
   ```

---

## 4. Simulation & Visualization (no hardware)

**Mock setup in RViz**
```bash
# Terminal 1 — launch the mock setup
source install/setup.bash
ros2 launch sinthlab_bringup iiwa7_mock_apple.launch.py
```
```bash
# Terminal 2 — visualize
source install/setup.bash
ros2 launch lbr_bringup rviz.launch.py \
  rviz_cfg_pkg:=lbr_bringup \
  rviz_cfg:=config/mock.rviz
```

**Physics simulation in Gazebo**
```bash
source install/setup.bash
ros2 launch sinthlab_bringup iiwa7_gazebo_apple.launch.py
```

**MoveIt with apple (mock or gazebo)**
```bash
source install/setup.bash

# Mock + MoveIt + RViz
ros2 launch sinthlab_bringup iiwa7_moveit_apple.launch.py mode:=mock rviz:=true

# Gazebo + MoveIt + RViz
ros2 launch sinthlab_bringup iiwa7_moveit_apple.launch.py mode:=gazebo rviz:=true
```

---

## 5. Running Experiments on Hardware
> Python dependencies (`pyoptas`, `ruckig`) must be installed once per WSL session install (see
> [Building the Stack](#3-building-the-stack)).
>
> ⚠️ **Safety first.** On the first run of any scenario, operate in **T1** (reduced speed) with a
> hand on the E‑stop. The arm is actively controlled the moment a SmartPad application is running.

### Scenario quick reference
| # | Scenario | Launch file | SmartPad app (FRI) | ROS controller |
|---|----------|-------------|--------------------|----------------|
| 1 | Apple Pluck          | `iiwa7_apple_pluck_impedance_control.launch.py` | `LbrImpedanceControlServer` | `kuka_clik_controller` |
| 2 | Restricted on Plane  | `iiwa7_move_restricted_plane.launch.py`         | `LBRServer` (POSITION_CONTROL) | `kuka_clik_controller` |
| 3 | Apple Pluck Perturb  | `iiwa7_apple_pluck_impedance_perturb.launch.py` | `LbrImpedanceControlServer` | `kuka_clik_controller` |

All scenarios use FRI **POSITION** command mode: ROS sends a target pose, and the cabinet (or the
Python admittance engine, for Scenario 2) provides the compliance.

### Scenario 1 — Apple Pluck
This scenario streams a target equilibrium pose to the `kuka_clik_controller` (IK‑based position
control), while the KUKA cabinet runs Cartesian impedance natively via the
`LbrImpedanceControlServer` FRI app. The arm acts as a virtual physical spring and recoils when
pushed off its commanded Cartesian anchor.

**Steps to run:**
1. Check that `update_rate` in
   `lbr-stack/src/lbr_fri_ros2_stack/lbr_description/ros2_control/lbr_controllers.yaml` is set to `200`.
2. Run the launch file:
   ```bash
   ros2 launch sinthlab_bringup iiwa7_apple_pluck_impedance_control.launch.py
   ```
3. On the KUKA SmartPad, start the **`LbrImpedanceControlServer`** application. It opens four
   selection dialogs in sequence — choose:

   | Prompt | Select |
   |--------|--------|
   | FRI send period [ms] | `10` |
   | Remote IP address | `172.31.1.148` (your ROS / WSL2 laptop IP) |
   | Cartesian stiffness (K diagonal) | `Medium Cartesian` |
   | Damping ratio (D0) | `0.7 (Standard)` |

   *This app is hard‑wired to Cartesian Impedance control in `POSITION` command mode. The other
   stiffness profiles (`Very Soft Z`, `Soft Z (Apple Pluck)`, `Stiff Cartesian`) and damping ratios
   (`0.3 (Underdamped)`, `1.0 (Critically Damped)`) are available if you want to change the feel.*
4. The arm moves to the start. Wait for the beep, then pull the end effector gently to trigger the
   0.2 m displacement threshold. A second beep plays, and the arm awaits physical recoil before
   restarting.

### Scenario 2 — Move Restricted on a Plane
This scenario uses the `kuka_clik_controller`, an exact‑tracking inverse‑kinematics (IK) solver
with no built‑in spring physics. The Python action scripts construct an instantaneous admittance
engine to apply mathematical virtual fixtures (sine waves, flat planes, bounding boxes) against
the user's pushing torques.

**Steps to run:**
1. Run the launch file:
   ```bash
   ros2 launch sinthlab_bringup iiwa7_move_restricted_plane.launch.py
   ```
2. On the KUKA SmartPad, start the **`LBRServer`** application. It opens four selection dialogs in
   sequence — choose:

   | Prompt | Select |
   |--------|--------|
   | FRI send period [ms] | `10` |
   | Remote IP address | `172.31.1.148` (your ROS / WSL2 laptop IP) |
   | FRI control mode | `POSITION_CONTROL` |
   | FRI client command mode | `POSITION` |

   *`LBRServer` also offers `JOINT_IMPEDANCE_CONTROL` / `CARTESIAN_IMPEDANCE_CONTROL` control modes
   and `WRENCH` / `TORQUE` command modes. This scenario streams joint positions from
   `kuka_clik_controller`, so it needs `POSITION_CONTROL` + `POSITION`.*

> **Tip:** Tweak `virtual_fixtures_params.yaml` to set the desired `virtual_fixture_profile`
> (`sine_wave`, `flat_table`, etc.).

### Scenario 3 — Apple Pluck Perturb
This scenario builds upon the Apple Pluck physics (cabinet‑side Cartesian impedance via
`LbrImpedanceControlServer`, with `kuka_clik_controller` streaming the equilibrium pose) but
introduces a sudden, programmatic Cartesian spatial shift right before the user acts, to study the
response to mechanical perturbation.

**Steps to run:**
1. Check that `update_rate` in
   `lbr-stack/src/lbr_fri_ros2_stack/lbr_description/ros2_control/lbr_controllers.yaml` is set to `200`.
2. Run the launch file:
   ```bash
   ros2 launch sinthlab_bringup iiwa7_apple_pluck_impedance_perturb.launch.py
   ```
3. On the KUKA SmartPad, start the **`LbrImpedanceControlServer`** application with the **same four
   selections as Scenario 1**:

   | Prompt | Select |
   |--------|--------|
   | FRI send period [ms] | `10` |
   | Remote IP address | `172.31.1.148` (your ROS / WSL2 laptop IP) |
   | Cartesian stiffness (K diagonal) | `Medium Cartesian` |
   | Damping ratio (D0) | `0.7 (Standard)` |
4. The arm acts exactly as the standard pluck, but automatically jerks to the side approximately
   1.5 seconds prior to the readiness cue.

---

## 6. Software Architecture
The stack separates **hard real‑time physical loops** from **high‑level orchestration**, so that
Python state transitions never compromise the 1000 Hz hardware control loops.

### 6.1 System Overview — data flow
The orchestrator sends Cartesian target poses *down* to the cabinet through `kuka_clik_controller`;
robot state flows *back up* to the Python monitors. The compliance ("spring") lives on the cabinet.

```mermaid
flowchart TB
    subgraph L3["Layer 3 · Orchestration (Python)"]
        ORCH["Orchestrator<br/>state machine<br/>(apple_pluck,<br/>perturb,<br/>restricted_plane)"]
        ACT["Modular actions:<br/>MoveToPosition<br/>DisplacementMonitor<br/>RestrictedPlane<br/>AudioCue"]
    end
    subgraph L2["Layer 2 · Kinematics (Python)"]
        OPTAS["optas<br/>FK and<br/>Jacobian"]
    end
    subgraph L1["Layer 1 · Real-time control (C++)"]
        CLIK["kuka_clik_controller<br/>IK: Cartesian target<br/>to joint position"]
        BCAST["Broadcasters:<br/>lbr_state<br/>force_torque<br/>estimated_wrench"]
    end
    subgraph CAB["KUKA Cabinet · 1000 Hz"]
        APP["LbrImpedanceControlServer<br/>(FRI app)<br/>Cartesian impedance<br/>POSITION cmd mode"]
        ARM["iiwa7 arm"]
    end

    ORCH <--> ACT
    ACT -. "FK /<br/>Jacobian" .-> OPTAS
    ACT -- "PoseStamped<br/>target_frame" --> CLIK
    CLIK -- "joint position<br/>cmd (FRI)" --> APP
    APP -- "compliant<br/>motion" --> ARM
    ARM -- "measured<br/>state" --> APP
    APP -- "FRI<br/>state" --> BCAST
    BCAST -- "LBRState /<br/>wrench" --> ACT
```

### 6.2 Hardware Bring‑up Sequence
Controllers are spawned in a deliberate order: `joint_state_broadcaster` first (it needs no URDF
and proves the controller_manager has received the robot description), then the controllers that
parse the URDF in `on_init()`.

```mermaid
sequenceDiagram
    actor Op as Operator
    participant ROS as ROS 2 (WSL2)
    participant CM as controller_manager
    participant CAB as KUKA cabinet (SmartPad)

    Op->>ROS: ros2 launch ... apple_pluck_impedance_control.launch.py
    ROS->>CM: start ros2_control_node (FRI client) + robot_state_publisher
    Op->>CAB: Start LbrImpedanceControlServer (FRI 10 ms)
    CAB-->>CM: FRI session established (COMMANDING_ACTIVE)
    CM->>CM: spawn joint_state_broadcaster
    Note over CM: only after it activates (URDF received)
    CM->>CM: spawn estimated_wrench · lbr_state · force_torque · kuka_clik_controller
    ROS-->>Op: Orchestrator starts trial — arm moves to start pose
```

### 6.3 Layers
**Layer 1 — Real‑time control (C++ / ros2_control)**
- **Cabinet‑side Cartesian impedance (`LbrImpedanceControlServer`):** the KUKA cabinet runs the
  Cartesian‑impedance virtual spring at 1000 Hz; ROS only streams the equilibrium pose to it.
- **`kuka_clik_controller` (IDRA Lab):** a pure Closed‑Loop Inverse Kinematics tracker (no built‑in
  elasticity), used to convert a Cartesian target into joint‑position commands.
- *Message interface:* receives `geometry_msgs/PoseStamped` target points on
  `…/kuka_clik_controller/target_frame`.

**Layer 2 — Kinematics math (Python)**
- **`optas`:** used inside the Python actions for fast Forward Kinematics (FK) and analytical
  Jacobian conversions, avoiding singular‑matrix crashes when reading Cartesian poses or mapping
  forces.

**Layer 3 — State‑machine orchestration (Python)**
The experimental flows are orchestrated entirely by high‑level `rclpy` nodes calling modular
actions (`MoveToPositionAction`, `CartesianImpedanceDisplacementMonitor`,
`MoveRestrictedOnAPlaneAction`, `AudioCue`). The per‑scenario flows are below.

### 6.4 Experiment State Flows

**Flow 1 — Apple Pluck**
```mermaid
stateDiagram-v2
    [*] --> MoveToStart : Automated trajectory
    MoveToStart --> QuietWindow : Wait 2.0s
    QuietWindow --> AudioCue : Trigger audio driver
    AudioCue --> DisplacementMonitor : Calculate tf2 offset
    DisplacementMonitor --> Snap : User pulls > 0.2m Z-axis
    Snap --> WaitRecoil : Wait for cabinet impedance to recoil
    WaitRecoil --> MoveToStart
```

**Flow 2 — Restricted Virtual Fixtures**
```mermaid
stateDiagram-v2
    [*] --> MoveToStart : Rise to workspace
    MoveToStart --> QuietWindow : Wait 2.0s
    QuietWindow --> AudioCue
    AudioCue --> FixtureAdmittance
    note right of FixtureAdmittance
      Calculates pseudo-inverse Jacobian.
      Applies Moore-Penrose mapping from tool torque
      to admittance shifts on mathematical rails.
    end note
    FixtureAdmittance --> SnapThreshold : Pull thresholds broken
    SnapThreshold --> WaitRecoil
    WaitRecoil --> MoveToStart
```

**Flow 3 — Perturb Experiment**
```mermaid
stateDiagram-v2
    [*] --> MoveToStart : Automated trajectory
    MoveToStart --> QuietWindow : Wait 2.0s
    QuietWindow --> PerturbShift : Sudden shift (1.5s delay)
    PerturbShift --> AudioCue : Trigger audio driver
    AudioCue --> DisplacementMonitor : Calculate tf2 offset
    DisplacementMonitor --> Snap : User pulls > threshold
    Snap --> WaitRecoil : Recoil physics
    WaitRecoil --> MoveToStart
```

---

## 7. Troubleshooting
- **RViz shows nothing / Gazebo crashes immediately.** The graphics library may be trying to use a
  GPU that isn't available. Force software rendering, then relaunch:
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```

---

## 8. Development & Contributing
- This stack follows an **underlay → overlay** structure. It reuses
  [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack)[^1], which in turn has
  ROS 2 as its underlay — so our code imports classes and functions from `lbr_fri_ros2_stack`.
- The `sinthlab_lbr_stack.repos` file lists every repository dependency that gets imported during
  setup. After setup, `~/lbr-stack/src` (the default) contains 4 repositories.
- **We own and manage only `sinthlab-kuka-stack`** — make changes only in that folder. Its git
  repository is [here](https://github.com/sinthlab/sinthlab-kuka-stack).
  - To contribute, create your own branch and open a pull request on GitHub so changes can be
    tracked and approved.
- If you use Copilot (or another AI‑assisted editor), open the codebase from the **root**
  (`~/lbr-stack` by default) so the agent can index the whole codebase including its dependencies.

> **Maintainer (Navin Modi) disclosure:** for my development I have used VS Code and GitHub Copilot
> agents for assisted development.

### Testing / rebuild loop
After making changes, rebuild from the workspace root (`~/lbr-stack` by default), re‑source, and
relaunch:
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

## Acknowledgement
This work is built on top of Huber et al.[^1]; all original credit for `lbr_fri_ros2_stack` goes to
that team.

[^1]: LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots, Journal of Open Source Software. [doi](https://doi.org/10.21105/joss.06138)
