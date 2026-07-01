# 中心线检测诊断话题说明

本文档记录当前为小论文实验新增的中心线检测诊断输出。该改动只增加观测指标，不改变中心线提取和 PID 控制行为。

## 1. 新增话题

```text
/centerline_detection_diagnostics
```

消息类型：

```text
std_msgs/msg/Float32MultiArray
```

该话题由节点发布：

```text
corn_row_detector_projection
```

## 2. 数据字段顺序

`data` 数组字段含义如下：

| 下标 | 名称 | 单位 | 含义 |
|---:|---|---|---|
| 0 | valid | - | 当前帧是否生成有效中心线，1 为有效，0 为无效 |
| 1 | left_points | 个 | 左侧内侧行点数 |
| 2 | right_points | 个 | 右侧内侧行点数 |
| 3 | corridor_width | m | 当前通行走廊宽度 |
| 4 | corridor_safety_margin | m | 当前安全裕度 |
| 5 | corridor_confidence | - | 当前走廊置信度 |
| 6 | row_yaw | rad | 估计的作物行方向角，相对于车体坐标系 |
| 7 | center_offset | m | 在关联距离处的中心线横向偏移 |
| 8 | left_slope | - | 左侧行拟合线斜率 |
| 9 | left_intercept | m | 左侧行拟合线截距 |
| 10 | right_slope | - | 右侧行拟合线斜率 |
| 11 | right_intercept | m | 右侧行拟合线截距 |
| 12 | line_lost_count | 帧 | 连续使用历史行模型的帧数 |
| 13 | path_points | 个 | 当前发布中心线路径点数 |

## 3. 对小论文有用的评价指标

基于该话题可以计算以下指标：

1. 中心线有效率：

$$
R_{valid}=\frac{N_{valid}}{N_{total}}
$$

2. 中心线横向抖动：

$$
\sigma_y=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
(y_{c,i}-\bar{y}_c)^2
}
$$

其中 $y_{c,i}$ 对应 `center_offset`。

3. 行方向角抖动：

$$
\sigma_\theta=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
(\theta_i-\bar{\theta})^2
}
$$

其中 $\theta_i$ 对应 `row_yaw`。

4. 平均走廊置信度：

$$
\bar{C}=\frac{1}{N}\sum_{i=1}^{N}C_i
$$

5. 平均安全裕度：

$$
\bar{M}=\frac{1}{N}\sum_{i=1}^{N}M_i
$$

6. 历史模型保持比例：

$$
R_{hold}=\frac{N(line\_lost\_count>0)}{N_{total}}
$$

该指标可以反映中心线检测短时不稳定时，时序保持机制介入的频率。

## 4. 推荐论文实验用法

建议在以下场景中记录该话题：

1. 车辆初始正对作物行。
2. 车辆初始向左偏航。
3. 车辆初始向右偏航。
4. 多行玉米干扰场景。
5. 点云稀疏或局部缺株场景。

每个场景至少重复 5 次，统计：

1. `valid` 有效率。
2. `center_offset` 均值、RMSE、标准差和最大值。
3. `row_yaw` 标准差。
4. `corridor_confidence` 均值和低置信比例。
5. `corridor_safety_margin` 均值和低安全裕度比例。
6. `line_lost_count > 0` 的比例。

这些指标可以支撑论文中的“偏航鲁棒性”“多行干扰鲁棒性”和“中心线稳定性”分析。

## 5. 记录脚本

`scripts/run_quality_pid_trial.sh` 已加入该话题记录。运行实验后，rosbag 中会包含：

```text
/centerline_detection_diagnostics
```

脚本结束时还会保存一次最终快照：

```text
final_centerline_detection_diagnostics.txt
```

