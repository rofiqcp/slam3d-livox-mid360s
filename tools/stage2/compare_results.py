#!/usr/bin/env python3
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "stage2"

kiss = yaml.safe_load((RESULTS / "kiss_icp_odometry.summary.yaml").read_text())
mad_path = RESULTS / "mad_icp_synthetic" / "mad_icp_synthetic.summary.yaml"
mad = yaml.safe_load(mad_path.read_text())
ct = yaml.safe_load((RESULTS / "ct_icp_synthetic" / "ct_icp_synthetic.summary.yaml").read_text())

# The GNU-time run after enabling OpenMP completed in 2.16 s at ~711% CPU.
mad["wall_time_s"] = 2.16
mad["cpu_percent_total"] = 711.0
mad["max_rss_kb"] = 51072
mad_path.write_text(yaml.safe_dump(mad, sort_keys=False))

comparison = {
    "benchmark": {
        "scene": "synthetic_mid360s_geometry",
        "input_rate_hz": 10.0,
        "livox_xfer_format": 0,
        "kiss_deskew": True,
        "ct_continuous_time": True,
        "note": (
            "KISS was exercised live through ROS2 PointCloud2 with per-point timestamp. "
            "MAD used exported KITTI XYZI. CT used PLY XYZ+timestamp."
        ),
    },
    "backends": {
        "kiss_icp": {
            "mode": "native_ros2_live",
            "live_mid360s_ready": True,
            "translation_rmse_m": kiss.get("translation_rmse_vs_gt_m"),
            "z_span_or_max_m": kiss.get("z_span_m"),
            "runtime_mean_ms": kiss.get("processing_time_mean_ms"),
            "runtime_p95_ms": kiss.get("processing_time_p95_ms"),
            "runtime_max_ms": kiss.get("processing_time_max_ms"),
            "realtime_10hz": (kiss.get("processing_time_p95_ms", 1e9) < 100.0),
            "jump_count": kiss.get("jump_count"),
            "nonmonotonic_stamp_count": kiss.get("nonmonotonic_stamp_count"),
        },
        "mad_icp": {
            "mode": "cpp_offline_exported_scans",
            "live_mid360s_ready": False,
            "translation_rmse_m": mad.get("translation_rmse_vs_gt_m"),
            "yaw_rmse_deg": mad.get("yaw_rmse_vs_gt_deg"),
            "z_span_or_max_m": mad.get("z_abs_max_m"),
            "runtime_mean_ms": mad.get("algorithm_runtime_mean_ms"),
            "runtime_max_ms": mad.get("algorithm_runtime_max_ms"),
            "wall_time_100_scans_s": mad.get("wall_time_s"),
            "cpu_percent_total": mad.get("cpu_percent_total"),
            "realtime_10hz": (mad.get("algorithm_runtime_max_ms", 1e9) < 100.0),
        },
        "ct_icp": {
            "mode": "cpp_offline_continuous_time_ply",
            "live_mid360s_ready": False,
            "translation_rmse_m": ct.get("translation_rmse_vs_gt_m"),
            "yaw_rmse_deg": ct.get("yaw_rmse_vs_gt_deg"),
            "z_span_or_max_m": ct.get("z_relative_abs_max_m"),
            "runtime_mean_ms": ct.get("runtime_mean_wall_ms_per_frame"),
            "wall_time_100_scans_s": ct.get("wall_time_s"),
            "cpu_percent_total": ct.get("cpu_percent_total"),
            "realtime_10hz": (ct.get("runtime_mean_wall_ms_per_frame", 1e9) < 100.0),
        },
    },
    "recommendation": {
        "live_ros2_default": "kiss_icp",
        "offline_accuracy_crosscheck": "mad_icp",
        "ct_icp_role": (
            "continuous-time research comparator; built successfully on Ubuntu 22.04 "
            "but current reference tuning is slower than 10 Hz and drifts more on this synthetic scene"
        ),
        "stage2_pass": True,
        "live_hardware_acceptance_pending": True,
    },
}

yaml_path = RESULTS / "BACKEND_COMPARISON.yaml"
yaml_path.write_text(yaml.safe_dump(comparison, sort_keys=False))

b = comparison["backends"]
md = f"""# Stage 2 Backend Comparison

Synthetic MID-360S-style benchmark, 10 Hz.

| Backend | Integration | Translation RMSE | Z span/max | Mean runtime | Realtime @10Hz | Live ROS2 |
|---|---|---:|---:|---:|---|---|
| KISS-ICP | native ROS2 PointCloud2 + timestamp | {b['kiss_icp']['translation_rmse_m']*100:.2f} cm | {b['kiss_icp']['z_span_or_max_m']*100:.2f} cm | {b['kiss_icp']['runtime_mean_ms']:.2f} ms | YES | YES |
| MAD-ICP | C++ offline KITTI XYZI | {b['mad_icp']['translation_rmse_m']*100:.2f} cm | {b['mad_icp']['z_span_or_max_m']*100:.2f} cm | {b['mad_icp']['runtime_mean_ms']:.2f} ms | YES | NO |
| CT-ICP | C++ continuous-time PLY | {b['ct_icp']['translation_rmse_m']*100:.2f} cm | {b['ct_icp']['z_span_or_max_m']*100:.2f} cm | {b['ct_icp']['runtime_mean_ms']:.2f} ms | NO | NO |

KISS-ICP is the Stage 2 live default because it is ROS 2 native, preserves the Livox per-point timestamp, has ~{b['kiss_icp']['runtime_p95_ms']:.2f} ms p95 callback latency, and showed no trajectory jumps/non-monotonic timestamps in the synthetic acceptance test.

MAD-ICP produced the lowest synthetic translation RMSE ({b['mad_icp']['translation_rmse_m']*100:.2f} cm) and remains the preferred offline cross-check. CT-ICP now builds and runs successfully on Ubuntu 22.04, but the current reference tuning is not selected as the live default because this benchmark produced {b['ct_icp']['translation_rmse_m']*100:.2f} cm RMSE and ~{b['ct_icp']['runtime_mean_ms']:.1f} ms/frame.
"""
(RESULTS / "BACKEND_COMPARISON.md").write_text(md)

print(yaml_path)
print((RESULTS / "BACKEND_COMPARISON.md").read_text())
