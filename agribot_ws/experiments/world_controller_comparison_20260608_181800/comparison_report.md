# 不同世界与控制算法对比实验结果

## 实验目的

为进一步检验当前冠下走廊估计方法在不同仿真场景和不同控制算法下的适应性，本文对 `towrow.world` 和 `corn_leaf_world.world` 两个 Gazebo 世界进行了对比测试。同时，对比 PID 控制器和纯追踪控制器在相同感知输出条件下的闭环表现。Gazebo 仿真以 `gui:=false` 方式运行，以减少图形界面对仿真速度的影响。

实验组合如下：

- 世界：`towrow.world`、`corn_leaf_world.world`
- 控制算法：PID、纯追踪
- 每组重复次数：3 次
- 终止条件：中心线持续为空且机器人无有效运动，判定为 `no_valid_centerline_at_start`；若机器人正常运动后中心线丢失，则判定为中心线丢失或停车。

## 汇总结果

| 世界 | 控制器 | 成功起步率 | 平均行驶距离 (m) | 平均置信度 | 中心线丢失率 | 终止原因 |
|---|---|---:|---:|---:|---:|---|
| corn_leaf_world.world | PID | 0/3 | 0.00007 | 0.000 | 1.000 | no_valid_centerline_at_start |
| corn_leaf_world.world | 纯追踪 | 0/3 | 0.00007 | 0.000 | 1.000 | no_valid_centerline_at_start |
| towrow.world | PID | 0/3 | 0.00007 | 0.000 | 1.000 | no_valid_centerline_at_start |
| towrow.world | 纯追踪 | 0/3 | 0.00007 | 0.000 | 1.000 | no_valid_centerline_at_start |

## 结果分析

实验结果显示，`towrow.world` 和 `corn_leaf_world.world` 两个场景下，PID 与纯追踪控制器均未能成功起步。四组实验的中心线丢失率均为 1.0，走廊置信度和走廊宽度均为 0，机器人行驶距离约为 `7e-5 m`，可视为没有发生有效运动。

该结果说明，当前系统在这两个世界中的失败主要发生在感知阶段，而不是控制阶段。由于 `/corn_row_center_line` 从起点即为空，PID 和纯追踪控制器都没有获得可跟踪路径，因此二者表现一致，均保持停车状态。换句话说，本组实验不能用于比较 PID 和纯追踪的控制性能，但可以用于说明当前走廊感知算法对不同世界结构的适配性不足。

与此前在 `twoworld.world` 中的结果相比，当前方法能够在紧密双行玉米场景中稳定生成中心线并完成约 6.48 m 的闭环行驶；但在 `towrow.world` 和 `corn_leaf_world.world` 中，默认参数无法从初始位姿提取有效左右边界。这表明当前 ROI、左右行分割、最内侧行提取、最小点数阈值或前方植物检测条件可能对场景几何和点云密度较敏感。

## 暴露的问题

1. 当前感知参数对 `twoworld.world` 适配较好，但对 `towrow.world` 和 `corn_leaf_world.world` 适配不足。
2. 控制器对比需要建立在感知模块能稳定输出中心线的前提下；当前两个新增世界没有满足该前提。
3. 起点位姿可能不在有效行间中心，或者点云 ROI 与世界中的作物分布不匹配，导致左右边界点不足。
4. `corn_leaf_world.world` 注释中为单行稀疏真实场景，而当前算法默认需要左右两侧边界，因此单行场景天然不容易生成完整行间走廊。

## 后续测试建议

后续应先针对这两个世界进行感知参数适配，再重新比较控制算法。建议按以下顺序进行：

1. 在 RViz 中显示 `/point_cloud_projected`、`/left_row_points`、`/right_row_points`、`/corn_row_center_line_viz`，确认点云是否落入当前 ROI。
2. 针对 `towrow.world` 放宽 `min_cluster_size`、`min_section_points`，并调整 `y_min/y_max`、`x_min/x_max`、`desired_row_separation` 等参数。
3. 针对 `corn_leaf_world.world` 判断其是否为单行场景；如果是单行，需要增加“单侧边界 + 目标行距先验”的退化模式。
4. 当两个世界均能稳定输出非空中心线后，再重新进行 PID 与纯追踪控制器对比，评价行驶距离、横向误差、速度平滑性和中心线丢失后的停车行为。

## 输出文件

- 总汇总 CSV：`combined_summary.csv`
- 对比表格图片：`comparison_table.png`
- `corn_leaf_world.world` + `PID`：`corn_leaf_world.world_pid/`
- `corn_leaf_world.world` + `纯追踪`：`corn_leaf_world.world_pure_pursuit/`
- `towrow.world` + `PID`：`towrow.world_pid/`
- `towrow.world` + `纯追踪`：`towrow.world_pure_pursuit/`
