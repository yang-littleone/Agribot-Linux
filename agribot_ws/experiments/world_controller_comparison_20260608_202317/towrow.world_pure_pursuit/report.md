# SCI Batch Experiment Report

- Generated: 2026-06-08T20:30:00
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 59.9923 | 0.0365 |
| distance | 6.3204 | 0.0590 |
| final_x | 6.3813 | 0.0617 |
| mean_abs_y | 0.0329 | 0.0008 |
| max_abs_y | 0.1206 | 0.0006 |
| mean_confidence | 0.6868 | 0.0017 |
| min_confidence | 0.5509 | 0.0009 |
| mean_width | 0.7904 | 0.0012 |
| min_width | 0.6237 | 0.0013 |
| mean_safety_margin | 0.1324 | 0.0018 |
| centerline_loss_ratio | 0.0007 | 0.0009 |
| obstacle_ratio | 0.0020 | 0.0028 |
| mean_speed_cmd | 0.4997 | 0.0005 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 59.9797 | 29.9540 | 6.3262 | 6.3802 | 0.1198 | 0.0324 | 0.1199 | 0.6874 | 0.5502 | 0.7902 | 0.6219 | 0.1322 | -0.0421 | 0.0000 | 0.0000 | 0.5000 | timeout |
| 2 | 59.9553 | 29.6140 | 6.2454 | 6.3063 | 0.1208 | 0.0323 | 0.1208 | 0.6884 | 0.5521 | 0.7920 | 0.6245 | 0.1348 | -0.0412 | 0.0000 | 0.0000 | 0.5000 | timeout |
| 3 | 60.0419 | 30.3280 | 6.3895 | 6.4575 | 0.1201 | 0.0340 | 0.1212 | 0.6844 | 0.5503 | 0.7890 | 0.6248 | 0.1303 | -0.0421 | 0.0020 | 0.0059 | 0.4990 | timeout |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
