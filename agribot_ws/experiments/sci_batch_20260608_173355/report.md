# SCI Batch Experiment Report

- Generated: 2026-06-08T17:48:14
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 170.4006 | 1.4278 |
| distance | 6.4818 | 0.0110 |
| final_x | 6.4498 | 0.0021 |
| mean_abs_y | 0.0634 | 0.0015 |
| max_abs_y | 0.1132 | 0.0027 |
| mean_confidence | 0.7699 | 0.0010 |
| min_confidence | 0.5214 | 0.0023 |
| mean_width | 0.4466 | 0.0016 |
| min_width | 0.3606 | 0.0027 |
| mean_safety_margin | 0.0255 | 0.0004 |
| centerline_loss_ratio | 0.0137 | 0.0017 |
| obstacle_ratio | 0.8866 | 0.0028 |
| mean_speed_cmd | 0.2804 | 0.0010 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 170.0545 | 48.2120 | 6.4626 | 6.4482 | 0.1107 | 0.0616 | 0.1107 | 0.7707 | 0.5229 | 0.4444 | 0.3612 | 0.0252 | 0.0001 | 0.0169 | 0.8885 | 0.2790 | centerline_lost_or_stopped |
| 2 | 171.7254 | 49.2320 | 6.4899 | 6.4491 | 0.1130 | 0.0640 | 0.1130 | 0.7711 | 0.5207 | 0.4488 | 0.3577 | 0.0262 | 0.0001 | 0.0129 | 0.8812 | 0.2822 | centerline_lost_or_stopped |
| 3 | 167.7946 | 48.3480 | 6.4885 | 6.4532 | 0.1123 | 0.0635 | 0.1123 | 0.7683 | 0.5181 | 0.4458 | 0.3588 | 0.0255 | 0.0003 | 0.0133 | 0.8875 | 0.2803 | centerline_lost_or_stopped |
| 4 | 170.8964 | 49.0280 | 6.4916 | 6.4510 | 0.1123 | 0.0658 | 0.1183 | 0.7701 | 0.5206 | 0.4483 | 0.3655 | 0.0254 | 0.0000 | 0.0131 | 0.8868 | 0.2800 | centerline_lost_or_stopped |
| 5 | 171.5323 | 48.7560 | 6.4762 | 6.4475 | 0.1114 | 0.0622 | 0.1114 | 0.7696 | 0.5248 | 0.4458 | 0.3597 | 0.0252 | 0.0001 | 0.0123 | 0.8891 | 0.2803 | centerline_lost_or_stopped |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
