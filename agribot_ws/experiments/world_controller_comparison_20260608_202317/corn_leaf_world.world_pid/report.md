# SCI Batch Experiment Report

- Generated: 2026-06-08T20:33:28
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 59.9463 | 0.0070 |
| distance | 4.9123 | 0.0333 |
| final_x | 4.9505 | 0.0349 |
| mean_abs_y | 0.0219 | 0.0006 |
| max_abs_y | 0.0420 | 0.0029 |
| mean_confidence | 0.9132 | 0.0004 |
| min_confidence | 0.8379 | 0.0058 |
| mean_width | 1.3588 | 0.0004 |
| min_width | 1.2377 | 0.0157 |
| mean_safety_margin | 0.4046 | 0.0010 |
| centerline_loss_ratio | 0.0000 | 0.0000 |
| obstacle_ratio | 0.0000 | 0.0000 |
| mean_speed_cmd | 0.2851 | 0.0012 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 59.9410 | 34.4420 | 4.8908 | 4.9179 | 0.0368 | 0.0227 | 0.0461 | 0.9129 | 0.8352 | 1.3591 | 1.2227 | 0.4034 | 0.2408 | 0.0000 | 0.0000 | 0.2867 | timeout |
| 2 | 59.9417 | 34.4420 | 4.8867 | 4.9346 | 0.0397 | 0.0217 | 0.0397 | 0.9130 | 0.8326 | 1.3582 | 1.2594 | 0.4058 | 0.2940 | 0.0000 | 0.0000 | 0.2839 | timeout |
| 3 | 59.9563 | 34.5100 | 4.9593 | 4.9988 | 0.0401 | 0.0212 | 0.0401 | 0.9138 | 0.8459 | 1.3590 | 1.2309 | 0.4046 | 0.3016 | 0.0000 | 0.0000 | 0.2846 | timeout |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
