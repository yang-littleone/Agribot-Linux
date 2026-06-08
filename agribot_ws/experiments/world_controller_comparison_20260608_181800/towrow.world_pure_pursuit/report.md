# SCI Batch Experiment Report

- Generated: 2026-06-08T18:19:46
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 12.1723 | 0.0156 |
| distance | 0.0001 | 0.0000 |
| final_x | 0.0001 | 0.0000 |
| mean_abs_y | 0.0000 | 0.0000 |
| max_abs_y | 0.0000 | 0.0000 |
| mean_confidence | 0.0000 | 0.0000 |
| min_confidence | 0.0000 | 0.0000 |
| mean_width | 0.0000 | 0.0000 |
| min_width | 0.0000 | 0.0000 |
| mean_safety_margin | 0.0000 | 0.0000 |
| centerline_loss_ratio | 1.0000 | 0.0000 |
| obstacle_ratio | 0.0000 | 0.0000 |
| mean_speed_cmd | 0.0000 | 0.0000 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 12.1612 | 12.2060 | 0.0001 | 0.0001 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0000 | 0.0000 | no_valid_centerline_at_start |
| 2 | 12.1614 | 12.2060 | 0.0001 | 0.0001 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0000 | 0.0000 | no_valid_centerline_at_start |
| 3 | 12.1944 | 12.2400 | 0.0001 | 0.0001 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 | 0.0000 | 0.0000 | no_valid_centerline_at_start |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
