# SCI Batch Experiment Report

- Generated: 2026-06-08T20:19:13
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 119.9832 | 0.0385 |
| distance | 0.0003 | 0.0000 |
| final_x | 0.0003 | 0.0000 |
| mean_abs_y | 0.0000 | 0.0000 |
| max_abs_y | 0.0000 | 0.0000 |
| mean_confidence | 0.5682 | 0.0001 |
| min_confidence | 0.5435 | 0.0000 |
| mean_width | 0.7661 | 0.0000 |
| min_width | 0.7561 | 0.0000 |
| mean_safety_margin | 0.0580 | 0.0002 |
| centerline_loss_ratio | 0.0000 | 0.0000 |
| obstacle_ratio | 0.0000 | 0.0000 |
| mean_speed_cmd | 0.0000 | 0.0000 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 120.0300 | 57.1880 | 0.0003 | 0.0003 | 0.0000 | 0.0000 | 0.0000 | 0.5682 | 0.5435 | 0.7661 | 0.7561 | 0.0582 | 0.0463 | 0.0000 | 0.0000 | 0.0000 | timeout |
| 2 | 119.9840 | 58.7520 | 0.0003 | 0.0003 | 0.0000 | 0.0000 | 0.0000 | 0.5681 | 0.5435 | 0.7660 | 0.7561 | 0.0578 | 0.0461 | 0.0000 | 0.0000 | 0.0000 | timeout |
| 3 | 119.9356 | 58.2420 | 0.0003 | 0.0003 | 0.0000 | 0.0000 | 0.0000 | 0.5682 | 0.5435 | 0.7661 | 0.7561 | 0.0580 | 0.0461 | 0.0000 | 0.0000 | 0.0000 | timeout |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
