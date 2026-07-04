# ROS 2 on Windows for Chrono::ROS: findings and recommendation (2026-07-04)

Timeboxed diagnostic (probes only, nothing installed). Question: can ROS-dependent benchmark
tasks run on this Windows/AMD machine, and what specifically stands in the way?

## Findings on this machine

1. There is NO ROS anywhere on this box: WSL is not installed, no native ROS 2 tree
   (`C:\dev\ros2_*`, `C:\opt\ros`, `C:\ros2` all absent), no chocolatey (the native-install
   tooling route), no RoboStack conda environments.
2. The Windows conda PyChrono package (the pinned `pychrono10` env) ships the ROS *demos*
   (`demos/ros/demo_ROS_*.py`) but NO `pychrono.ros` module: the ROS bindings are not built into
   Windows packages, so those demos cannot run from the package as shipped. (Packaging finding;
   worth an upstream note.)
3. `src/chrono_ros/CMakeLists.txt` hard-requires a SOURCED ROS 2 environment at configure time
   (`ROS_DISTRO` and `AMENT_PREFIX_PATH` env vars, plus `rclcpp`, `tf2_ros`, `*_msgs` via
   find_package); absent that, the module silently disables. The file IS Windows-aware (WIN32
   blocks for DLL placement, MSVC warning suppression), so upstream anticipates Windows builds,
   given a working ROS 2 installation.

## The three routes to ROS 2 on Windows, assessed

1. **Native binaries.** Official ROS 2 Windows support has degraded to community tier in recent
   distributions; binary archives lag, the dependency chain (Visual Studio version pinning,
   OpenSSL, Qt) is brittle, and every upgrade is manual. Workable for a determined lab, but high
   maintenance for benchmark infrastructure.
2. **RoboStack (conda-forge).** ROS 2 packaged on conda-forge; installs into a conda env on
   Windows, which fits this lab's conda-first convention exactly, and provides the
   `ROS_DISTRO`/`AMENT_PREFIX_PATH` environment Chrono::ROS's CMake needs. Two risks: (a)
   RoboStack builds with MSVC, while our HIP Chrono build uses ROCm clang (MSVC ABI-targeting, so
   linking should work, but it is an untested compiler-mix for this module); (b) DDS behavior on
   Windows (discovery, firewalls) is historically the flakiest layer, and Chrono has already
   needed DDS-conflict fixes (PR #718).
3. **WSL2.** The robust ROS experience, but WSL is not installed here, and putting ROS in Linux
   while Chrono runs on Windows splits the system across an OS boundary (cross-boundary DDS
   discovery is its own project). Only sensible if the whole Chrono+ROS stack moves into WSL, at
   which point the native-Linux machine is strictly better.

## Recommendation

1. **Keep ROS-dependent benchmark tasks deferred to the Linux/NVIDIA machine** (where ROS 2 is a
   first-class citizen and Chrono::Sensor will also live). ROS tasks stay in the designed-but-
   deferred backlog; nothing about the current 12-task suite is affected.
2. If a Windows-local ROS experiment becomes worthwhile before then, **RoboStack is the route to
   try first** (one conda env, no system-level installs), building Chrono::ROS against it in a
   separate MSVC build tree (NOT the HIP/clang tree) to avoid the untested compiler mix.
3. Upstream note worth filing: the Windows conda PyChrono package ships ROS demos without the
   `pychrono.ros` module, either build the module where feasible or exclude the demos, since the
   present state misleads users (this is exactly the class of version/packaging drift the
   benchmark's L1 gate exists to catch).
