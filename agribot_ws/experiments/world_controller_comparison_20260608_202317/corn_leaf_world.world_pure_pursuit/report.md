# SCI Batch Experiment Report

- Generated: 2026-06-08T20:36:53
- Launch: `ros2 launch diff_drive_robot robot.launch.py`
- Perception: `corn_row_detector_projection`
- Control: `cornfield_navigation_node` / PID controller
- World reset service: `/reset_world`

## Result Summary

| Metric | Mean | Std |
|---|---:|---:|
| duration_wall | 59.9705 | 0.0061 |
| distance | 7.2738 | 0.0190 |
| final_x | 7.3618 | 0.0213 |
| mean_abs_y | 0.0055 | 0.0006 |
| max_abs_y | 0.0216 | 0.0010 |
| mean_confidence | 0.9116 | 0.0003 |
| min_confidence | 0.8343 | 0.0144 |
| mean_width | 1.3540 | 0.0010 |
| min_width | 1.2409 | 0.0336 |
| mean_safety_margin | 0.4018 | 0.0007 |
| centerline_loss_ratio | 0.0000 | 0.0000 |
| obstacle_ratio | 0.0152 | 0.0009 |
| mean_speed_cmd | 0.5000 | 0.0000 |

## Trial Table

| trial | duration_wall | duration_sim | distance | final_x | final_y | mean_abs_y | max_abs_y | mean_confidence | min_confidence | mean_width | min_width | mean_safety_margin | min_safety_margin | centerline_loss_ratio | obstacle_ratio | mean_speed_cmd | end_reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 59.9697 | 34.4080 | 7.2472 | 7.3328 | 0.0202 | 0.0046 | 0.0202 | 0.9119 | 0.8354 | 1.3549 | 1.2525 | 0.4022 | 0.2852 | 0.0000 | 0.0159 | 0.5000 | timeout |
| 2 | 59.9634 | 34.5780 | 7.2836 | 7.3692 | 0.0226 | 0.0062 | 0.0226 | 0.9117 | 0.8513 | 1.3543 | 1.2750 | 0.4008 | 0.2882 | 0.0000 | 0.0139 | 0.5000 | timeout |
| 3 | 59.9783 | 34.6460 | 7.2907 | 7.3835 | 0.0219 | 0.0057 | 0.0219 | 0.9112 | 0.8161 | 1.3526 | 1.1951 | 0.4023 | 0.2847 | 0.0000 | 0.0159 | 0.5000 | timeout |

## Outputs

- Samples CSV: `samples.csv`
- Summary CSV: `summary.csv`
- Trajectory plot: `trajectory.png`
- Metric time series: `metrics_timeseries.png`
