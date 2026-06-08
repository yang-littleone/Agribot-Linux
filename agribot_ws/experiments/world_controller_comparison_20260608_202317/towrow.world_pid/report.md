# SCI Batch Experiment Report

- Generated: 2026-06-08T20:26:36
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 60.0130 | 0.0333 |
| distance | 4.0486 | 0.0559 |
| final_x | 4.0695 | 0.0550 |
| mean_abs_y | 0.0303 | 0.0012 |
| max_abs_y | 0.1335 | 0.0040 |
| mean_confidence | 0.7057 | 0.0003 |
| min_confidence | 0.5614 | 0.0010 |
| mean_width | 0.8336 | 0.0005 |
| min_width | 0.7667 | 0.0008 |
| mean_safety_margin | 0.1642 | 0.0011 |
| centerline_loss_ratio | 0.0000 | 0.0000 |
| obstacle_ratio | 0.0000 | 0.0000 |
| mean_speed_cmd | 0.3117 | 0.0077 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 60.0337 | 27.7100 | 3.9940 | 4.0182 | 0.1295 | 0.0287 | 0.1295 | 0.7058 | 0.5613 | 0.8342 | 0.7669 | 0.1655 | 0.0320 | 0.0000 | 0.0000 | 0.3019 | timeout |
| 2 | 59.9660 | 27.8800 | 4.0262 | 4.0444 | 0.1321 | 0.0305 | 0.1321 | 0.7059 | 0.5602 | 0.8331 | 0.7675 | 0.1628 | 0.0317 | 0.0000 | 0.0000 | 0.3126 | timeout |
| 3 | 60.0392 | 28.2540 | 4.1254 | 4.1458 | 0.1388 | 0.0317 | 0.1388 | 0.7053 | 0.5627 | 0.8334 | 0.7656 | 0.1643 | 0.0320 | 0.0000 | 0.0000 | 0.3206 | timeout |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
