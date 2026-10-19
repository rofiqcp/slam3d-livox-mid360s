# Instructions for AI agents — Livox MID-360S SLAM

These project rules apply to the entire checkout, including nested ROS packages. Keep solutions reproducible on Sirobo and MiniPC1, with no generated artifacts on GitHub.

## Workflow and scope

1. Read `README.md`, `ROADMAP.md`, relevant package manifests/CMakeLists and `git status`. Ensure `v1` tracks `origin/v1`.
2. Pull fast-forward only with a clean worktree. Preserve existing local edits and stop if a merge would overwrite work. Never force-push or use destructive Git cleanup commands.
3. Change only files required for the requested function. Never casually bulk-replace third-party source, change ROS message names, reformat unrelated packages or remove license headers.
4. Use ROS 2 Humble, modern C++ where appropriate, package-local launch/config and portable workspace-relative scripts. Configurable IPs, network interface names, topic names and device/calibration values must not be hard-coded in programs.
5. Separate acquisition, odometry, mapping, relocalization, Nav2 and dashboard logic. Confirm TF chain, timestamps, frames and covariance semantics before changing navigation behavior.
6. All motion must fail closed if localization, TF, IMU, LiDAR freshness or map alignment is invalid. Never disable a safety gate to make a demo appear to work.
7. **Do not alter host networking, flash controllers, arm or drive motors, overwrite maps or execute a real autonomous mission unless separately and explicitly authorized.**
8. Maintain `README.md`, `ROADMAP.md` and curated `docs/`; historical audits, logs, rosbags, point clouds, benchmarking outputs and snapshots belong in ignored `.local-only/`, `datasets/`, `results/` or outside this checkout, not as dated Markdown at the root.
9. Retain upstream licensing and required source under `external/`, `third_party/`, and source vendors. Vendor test code that is a declared CMake dependency is not a disposable local experiment.
10. Before committing: run `python3 tools/check_repo_hygiene.py`, `bash -n` on changed shells, compile changed Python, and run a targeted colcon build/test if a prepared host is available. Explain tests actually run, results, and blockers. Never invent success.
11. Stage only intended source/docs, review diffs, commit with a descriptive message and push `v1`. Recheck and pull on the other computer. Do not copy cache/build/install/log directories between machines.

## Engineering style

Favor small typed APIs, explicit errors/timeouts, bounded queues and resource use, and stable ROS interfaces. No dead code, duplicate helpers, circular launches, hardcoded home paths, sample secrets, ad-hoc fallback hacks or noisy reports. Avoid non-portable dependencies unless isolated and documented. Confirm a build change independently on each PC; one host's success is not proof for another.
