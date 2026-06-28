# 当前项目对话与需求检查点

本文档用于记录本窗口中关于农作物冠下无人运动平台、中心线提取算法、代码修改和小论文创新点讨论的完整上下文。后续如果对话过长导致上下文丢失，可先让 Codex 阅读本文档，再继续工作。

## 1. 项目背景

当前项目是一个 ROS 2 + Gazebo 农业机器人仿真与控制项目，工作空间位于：

```bash
/home/xkai/agribot/agribot_ws
```

主要使用流程为：

1. 启动机器人仿真：

```bash
ros2 launch diff_drive_robot robot.launch.py
```

2. 启动玉米行中心线检测节点：

```bash
ros2 run centerline_extraction corn_row_detector_projection
```

3. 使用路径跟踪算法跟踪检测出的中心线。

用户的大论文题目方向为：

```text
农作物冠下无人运动平台控制系统研究
```

当前小论文方向聚焦于：

```text
面向冠下无人平台导航的作物行走廊检测与中心线提取方法
```

## 2. 重要文件

当前最重要的代码文件：

```text
src/centerline_extraction/src/corn_row_detector_projection.cpp
src/centerline_extraction/include/centerline_extraction/corn_row_detector_projection.hpp
src/diff_drive_robot/launch/robot.launch.py
```

已经生成的重要文档：

```text
docs/under_canopy_corridor_paper_plan.md
docs/under_canopy_corridor_sci_draft.md
docs/conversation_context_checkpoint.md
```

其中：

- `under_canopy_corridor_paper_plan.md`：早期小论文创新方案；
- `under_canopy_corridor_sci_draft.md`：已生成的小论文初稿，包含公式、结构、流程图和实验设计；
- `conversation_context_checkpoint.md`：本文档，用于保存当前对话上下文。

## 3. 用户最初需求

用户最开始要求：

1. 阅读当前项目，说明项目用途、技术栈、入口文件、启动方式和新手阅读顺序；
2. 分析当前中心线提取算法的不足；
3. 对中心线提取算法进行代码优化；
4. 针对当前代码库和冠下无人平台方向提出小论文创新点；
5. 进一步把方案写成 Markdown 文档，并对小论文创新点对应地修改代码。

后续需求逐渐聚焦为：

```text
如何让玉米冠下机器人在多行点云、车头偏航、点云遮挡和局部抖动情况下稳定提取左右作物行与中心线。
```

## 4. 用户实际使用场景

用户当前主要在 Gazebo/RViz2 中做简化测试：

- Gazebo 中模型看起来是成熟期玉米；
- 但实际上碰撞体/点云目标可简化为竖直圆柱；
- 因此 RViz2 中投影点云表现为规整的小圆点；
- 用户关注的是算法能否在车体偏航、多行干扰情况下仍然正确识别最近左右两行和中心线。

用户明确指出：

```text
检测不应该使用外侧第二行、第三行等点云，而应该只通过最靠近小车的左右各一行进行检测。
```

## 5. 主要问题演变

### 5.1 多行干扰问题

最初问题：

```text
当小车左右只有单行时还可以；
出现多行之后，左右行拟合线会被第二行等点云干扰，导致错误拟合。
```

对应思路：

- 不再直接使用左右所有点云拟合；
- 对左右点云做横向聚类；
- 选择距离机器人中心线最近的左右作物行；
- 剔除外侧第二、第三行。

### 5.2 车头偏航问题

用户发现：

```text
小车正朝前时结果较准确；
车头角度稍微偏一点，中心线就被干扰，三条绿色线明显歪斜。
```

分析后认为：

```text
原始算法过度依赖 base_link 中的车头方向，把车头方向近似当成作物行方向。
```

当车头偏航后，作物行在 `base_link` 下会出现明显斜率，直接按车体 y 方向分割和拟合容易失败。

### 5.3 RViz2 坐标系问题

用户发现：

```text
在 RViz2 固定坐标系为 odom 时，小车外观不跟随点云和拟合线移动。
```

检查后发现：

- `odom -> base_footprint -> base_link` TF 链存在；
- `/tf` 中有 `odom -> base_footprint`；
- RViz2 一度 `use_sim_time = false`，而 Gazebo/robot_state_publisher 使用仿真时间；
- 这会导致 RViz2 用真实时间查询仿真时间 TF，出现 `odom` 下显示异常。

建议：

```bash
ros2 param set /rviz use_sim_time true
```

或者启动 RViz2 时：

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

另外，检测节点曾经把调试点云强制标记为 `odom`，这会造成 RViz2 显示混乱。后来已修改为：

```text
调试点云发布在 base_link，由 RViz2 通过 TF 显示到 odom。
```

## 6. 已完成的主要代码改动

修改文件：

```text
src/centerline_extraction/src/corn_row_detector_projection.cpp
src/centerline_extraction/include/centerline_extraction/corn_row_detector_projection.hpp
```

### 6.1 删除无用里程计订阅

原代码订阅 `/odom` 并保存：

```text
robot_current_x_
robot_current_y_
```

但检测流程完全未使用这些变量。已删除：

- `/odom` 订阅；
- `odom_callback()`；
- `robot_current_x_`；
- `robot_current_y_`；
- 相关 `nav_msgs/msg/odometry.hpp` include。

### 6.2 调试点云统一发布在 base_link

原先调试点云被转换或强制标记为 `odom`，容易造成显示和 TF 语义混乱。

已调整为：

```text
/point_cloud_projected
/left_row_points
/right_row_points
```

均发布为：

```text
frame_id = base_link
```

控制用中心线 `/corn_row_center_line` 仍可根据 `output_frame` 输出，默认 `odom`。

### 6.3 删除旧版 create_path()

旧的 `create_path()` 只生成单直线路径，当前主流程已使用：

```text
create_corridor_path()
```

因此删除了旧函数，避免代码混乱。

### 6.4 检测失败时清空旧显示

当点数不足、无前方植株、行模型不稳定时，已修改为同时清空：

- 中心线；
- 左右边界线；
- base_link 可视化中心线；
- 投影点云；
- 左右行点云；
- 通道宽度；
- 安全裕度；
- 置信度；
- 路径历史。

这样 RViz2 不会残留上一帧结果，减少误判。

### 6.5 加入全局行向估计与车体姿态解耦

为解决车头偏航问题，加入：

```cpp
estimate_global_row_yaw(...)
lookup_output_from_base_transform(...)
normalize_angle(...)
filter_global_row_yaw(...)
```

核心思想：

1. 使用 TF 查询 `odom <- base_link`；
2. 将点云从 `base_link` 变换到 `odom` 的相对局部坐标；
3. 在 `odom` 稳定坐标系下估计全局作物行方向；
4. 锁定并平滑全局作物行方向；
5. 减去机器人自身 yaw，得到作物行相对车体角度；
6. 用该角度把点云旋转到作物行坐标系；
7. 再进行左右分割、内侧行提取和中心线生成。

注意：曾出现全局 yaw 与局部 yaw 共用同一个状态变量的问题，导致几秒后线消失。后续已修复。

### 6.6 拆分全局行向和局部行模型状态

新增：

```cpp
bool has_tracked_global_row_yaw_;
float tracked_global_row_yaw_;
```

保留：

```cpp
bool has_tracked_row_yaw_;
float tracked_row_yaw_;
```

含义区分：

```text
tracked_global_row_yaw_：只表示 odom 下的全局作物行方向；
tracked_row_yaw_：只表示局部行模型相关方向。
```

修复了两个 yaw 状态互相污染导致线条消失的问题。

### 6.7 最近内侧作物行选择

在作物行坐标系中：

1. 按 y 正负分为左侧和右侧；
2. 对每侧点云按横向 y 进行聚类；
3. 选择距离机器人中心线最近且满足最小点数要求的簇；
4. 只用该簇作为当前左右作物行边界。

目标：

```text
剔除外侧第二行、第三行对拟合线的干扰。
```

### 6.8 平行作物行模型

为解决小车近处几个点把整条线拉偏的问题，加入：

```cpp
bool use_parallel_row_model_ = true;
float line_fit_x_min_ = 0.20;
```

对应参数：

```text
use_parallel_row_model = true
line_fit_x_min = 0.20
```

当前默认逻辑：

- 在作物行坐标系中，作物行应近似水平；
- 不再自由拟合斜率 `slope`；
- 只估计左右行横向截距 `intercept`；
- 截距使用中位数估计；
- 默认忽略车前 0.2 m 内近场点云。

这样可以减少近距离个别点对整条边界方向的影响。

### 6.9 时序一致性约束

已有并继续保留：

```cpp
line_filter_alpha_
max_line_lateral_jump_
max_row_width_jump_
max_row_yaw_jump_
max_line_lost_frames_
tracked_left_line_
tracked_right_line_
```

当前 `max_line_lost_frames` 从 5 提高到：

```text
30
```

作用：

```text
短时间检测失败时保持上一帧模型，避免线条几帧后直接消失。
```

## 7. 当前主要参数

当前重要参数包括：

```text
base_frame = base_link
output_frame = odom
use_row_yaw_estimation = true
use_simple_inner_row_mode = true
use_parallel_row_model = true
line_fit_x_min = 0.20
lateral_cluster_eps = 0.1
min_cluster_size = 10
line_filter_alpha = 0.25
max_line_lateral_jump = 0.18
max_row_width_jump = 0.25
max_row_yaw_jump = 0.45
max_line_lost_frames = 30
desired_row_separation = 0.8
platform_width = 0.30
```

如果近距离点仍影响边界，可优先调整：

```text
line_fit_x_min = 0.30 或 0.40
```

如果左右内侧行聚类不稳定，可调整：

```text
lateral_cluster_eps
min_cluster_size
```

## 8. 当前算法流程

当前中心线提取流程为：

```text
输入 /mid360_PointCloud2
  ↓
转换点云到 base_link
  ↓
ROI 滤波 + 体素降采样
  ↓
投影到地面平面
  ↓
查询 odom <- base_link TF
  ↓
在 odom 下估计并锁定全局作物行方向
  ↓
减去机器人 yaw，得到作物行相对车体角度
  ↓
点云旋转到作物行坐标系
  ↓
左右点云分割
  ↓
横向聚类选择最近左右内侧行
  ↓
平行作物行模型估计左右横向截距
  ↓
时序一致性约束，拒绝跳变并短时保持历史模型
  ↓
生成中心线、左右边界
  ↓
输出宽度、安全裕度、置信度
```

## 9. 当前效果与剩余问题

当前用户反馈：

```text
现在效果还可以，但有时候仍会有些许偏差，例如小车左侧第一二个点会让直线有轻微偏差。
```

针对该问题已加入：

```text
use_parallel_row_model = true
line_fit_x_min = 0.20
```

后续如仍有偏差，优先考虑：

1. 增大 `line_fit_x_min`；
2. 锁死全局行向，减少每帧估计；
3. 使用初始化阶段估计一次作物行方向，后续只小范围更新；
4. 对左右边界截距进一步采用滑动窗口中位数滤波；
5. 在 `odom` 下直接做最近两条平行线选择，而不是每帧局部变换。

## 10. 小论文创新点

当前建议的小论文创新点凝练为三点：

### 10.1 车体姿态解耦的全局行向锁定方法

解决：

```text
车头偏航时中心线识别偏斜。
```

核心：

```text
在 odom 下估计并锁定作物行方向，再减去机器人 yaw，得到作物行相对车体方向。
```

### 10.2 多行干扰下的最近内侧作物行选择与平行边界估计

解决：

```text
外侧第二、第三行点云干扰当前行间通道边界。
```

核心：

```text
横向聚类，选择距离机器人中心最近的左右内侧行；
在作物行坐标系中采用平行行模型，仅估计左右截距。
```

### 10.3 面向路径跟踪的时序一致性走廊跟踪与通道质量评价

解决：

```text
连续行驶中中心线抖动、短时遮挡、偶发丢线。
```

核心：

```text
边界跳变约束、行距跳变约束、历史模型保持、通道宽度、安全裕度和置信度输出。
```

## 11. 已生成小论文初稿

已生成：

```text
docs/under_canopy_corridor_sci_draft.md
```

该文档包含：

- 题目；
- 摘要；
- 关键词；
- 引言；
- 系统组成与问题定义；
- 方法；
- 公式；
- Mermaid 流程图；
- 控制接口；
- 仿真和实地实验设计；
- 对比方法；
- 评价指标；
- 消融实验；
- 参数敏感性实验；
- 预期结果；
- 结论；
- 参考文献检索方向；
- 实验记录表；
- 图表清单。

小论文当前建议题目：

```text
面向冠下无人平台导航的全局行向约束内侧作物行走廊检测方法
```

英文可写为：

```text
A Global Row-Orientation Constrained Inner-Crop-Row Corridor Detection Method for Under-Canopy Navigation of Agricultural Robots
```

## 12. 必做实验清单

### 12.1 对比方法

建议至少对比：

1. 普通最小二乘左右直线拟合；
2. RANSAC 左右直线拟合；
3. 最近内侧行选择 + 自由直线拟合；
4. 全局行向锁定 + 最近内侧行；
5. 完整方法：全局行向锁定 + 最近内侧行 + 平行约束 + 时序一致性；
6. 完整方法 + 置信度控制。

### 12.2 仿真实验工况

建议包括：

1. 标准直行作物行；
2. 车头偏航 5°、10°、15°；
3. 多行作物可见；
4. 单侧作物行局部缺失；
5. 双侧短时遮挡；
6. 行距变化；
7. 点云噪声增强；
8. 冠层叶片侵入行间；
9. 曲线作物行；
10. 不同行驶速度。

### 12.3 实地实验工况

如具备真实平台，建议：

1. 苗期、中期、成熟期；
2. 晴天、阴天、傍晚；
3. 平直行、弯曲行、缺株行；
4. 不同初始车头偏航角；
5. 单侧遮挡、双侧遮挡；
6. 不同行距地块。

### 12.4 评价指标

感知指标：

- 中心线横向误差；
- 航向误差；
- 左右边界误差；
- 检测丢失率；
- 中心线抖动；
- 单帧处理时间。

控制指标：

- 路径跟踪横向误差；
- 最大横向误差；
- 最小作物安全距离；
- 任务成功率；
- 平均速度。

消融实验：

- 去掉全局行向锁定；
- 去掉最近内侧行选择；
- 去掉平行作物行约束；
- 去掉时序一致性；
- 去掉置信度评价。

## 13. 注意事项

### 13.1 RViz2 时间同步

RViz2 必须使用仿真时间：

```bash
ros2 param set /rviz use_sim_time true
```

否则 `odom` 下可能看不到机器人或 TF 显示异常。

### 13.2 重启节点

每次 `colcon build --packages-select centerline_extraction` 后，需要重新 source 并重启检测节点：

```bash
source install/setup.bash
ros2 run centerline_extraction corn_row_detector_projection
```

### 13.3 不要混淆调试可视化和控制输出

当前建议：

```text
调试点云、局部中心线可视化：base_link
控制中心线：odom 或控制器期望坐标系
```

### 13.4 后续若继续优化

优先方向：

1. 固定初始化行向，而不是每帧完全重新估计；
2. 左右截距使用滑动窗口中位数；
3. 引入更严格的行距先验；
4. 将置信度接入路径跟踪控制；
5. 增加实验记录脚本，自动保存误差、丢线率、处理时间。

## 14. 最近一次代码编译状态

最近多次执行：

```bash
colcon build --packages-select centerline_extraction
```

均通过。

## 15. 给后续 Codex 的提醒

如果后续继续对话，请先阅读本文档，再检查：

```bash
git diff -- src/centerline_extraction/src/corn_row_detector_projection.cpp src/centerline_extraction/include/centerline_extraction/corn_row_detector_projection.hpp
```

不要轻易回退用户已有改动。

不要再回到最初那种“直接在 base_link 下按 y 左右分割并自由拟合直线”的方案，那是导致车头偏航错误的主要原因。

当前最重要的算法主线是：

```text
全局行向锁定
+ 最近内侧作物行选择
+ 平行作物行约束
+ 时序一致性跟踪
+ 通道质量评价
```

## 16. SCI 论文相关工作与文献检索进展

后续用户指出：

```text
论文数量太少，并且缺少今年和去年的论文；不能只引用时间较久的文献。
```

因此已经使用联网检索补充了一批 2025/2026 近期文献，并更新到了：

```text
docs/under_canopy_corridor_sci_draft.md
```

### 16.1 已写入 SCI 初稿的相关工作差异小节

在 `docs/under_canopy_corridor_sci_draft.md` 中已经新增并扩写：

```text
1.1 与现有作物行检测方法的区别
```

该节详细对比了：

1. 最小二乘直线拟合；
2. RANSAC；
3. Hough 变换；
4. DBSCAN/HDBSCAN/欧式聚类；
5. 纯 `base_link` 局部检测；
6. 深度学习语义分割；
7. 2025—2026 年端到端导航线提取、轻量化语义分割、LiDAR 作物行检测、行约束 SLAM 等最新趋势。

该节核心观点是：

```text
本文方法的创新不在于提出一种新的通用直线拟合器，
而在于将冠下导航中的作物行检测问题重新定义为：
全局行向约束下的当前可通行走廊边界选择问题。
```

同时强调：

```text
别人多数是在“找作物行/找线”；
本文是在“找当前机器人能够行驶的通道左右内侧边界”。
```

### 16.2 已补充的近两年重点文献

已补充到 SCI 初稿参考文献部分的近期文献包括：

1. Zheng and Wang, 2026, CCRDNet, Frontiers in Plant Science  
   中心作物行检测网络，直接检测 central crop row 并进行导航线拟合。

2. Liu and Wang, 2026, LCD-Net, IEEE Journal on Selected Topics in Signal Processing  
   端到端农业机器人导航线提取与作物行检测网络。

3. Navarro Gómez et al., 2026, Agriculture  
   基于 LaneATT 的温室环境实时行检测与导航。

4. Wan et al., 2026, Agronomy  
   CK-SLAM，玉米冠下四足机器人作物行和运动学约束 SLAM。

5. Wu et al., 2026, Sensors  
   改进 DeepLabV3+ 的作物行分割与导航线提取。

6. Sun et al., 2025, Agriculture  
   GD-YOLOv10n-Seg 用于大豆玉米复合种植作物行检测，结合 PCA 拟合。

7. Liu, Yandún and Kantor, 2025, ICRA  
   Crop-Agnostic LiDAR-Based Crop-Row Detection in Arable Fields。

8. Baltazar, Coelho and Brandão, 2025, CROS  
   Row Navigation using LiDAR in Autonomous Agricultural Vehicles。

9. Yang et al., 2024, IEEE Transactions on Instrumentation and Measurement  
   基于 3D LiDAR 的不同生育期作物行检测。

这些文献用于说明：

```text
近两年作物行导航研究正在向轻量化语义分割、
端到端导航线提取、LiDAR 作物行检测、
行约束 SLAM 和边缘端实时部署发展。
```

### 16.3 论文中与最新文献的差异定位

当前论文应突出与最新工作的区别：

- 2025/2026 视觉深度学习方法通常依赖图像标注和训练数据；
- 端到端方法重在直接预测导航线，但可解释性和几何约束较弱；
- LiDAR 作物行检测方法多关注检测作物行本身，不一定解决“当前通道左右内侧边界选择”；
- 行约束 SLAM 关注定位与建图，不是局部可通行走廊检测；
- 本文方法基于几何先验，不需要训练数据，重点解决冠下导航中的：

```text
车体偏航解耦
多行结构化干扰
最近内侧行选择
平行行约束
时序一致性
通道质量评价
```

因此后续写论文时，不要只说“比 RANSAC 稳定”，而要说：

```text
RANSAC 解决随机离群点问题；
本文解决多行平行作物带来的结构化干扰问题。
```

也不要只说“比深度学习简单”，而要说：

```text
本文利用行作物几何结构和机器人运动约束，
在无需训练数据的条件下实现当前可通行走廊边界估计。
```

### 16.4 仍需继续补充的文献方向

当前参考文献已经扩展到 18 篇左右，但 SCI 投稿仍建议扩展到 25-40 篇。后续应继续检索：

1. `Computers and Electronics in Agriculture` 近五年 crop row navigation；
2. `Biosystems Engineering` 农业机器人导航；
3. `Journal of Field Robotics` 冠下导航和行间机器人；
4. `IEEE Transactions on Instrumentation and Measurement` LiDAR 作物行检测；
5. `IEEE Robotics and Automation Letters` 农业机器人感知；
6. `Sensors` / `Agronomy` / `Agriculture` 2025/2026 crop row detection；
7. confidence-aware navigation / uncertainty-aware path tracking；
8. row-crop corridor detection / traversable corridor estimation。

## 17. 最近用户打开文件状态

最近 IDE 中活跃文件变为：

```text
src/centerline_extraction/src/main.cpp
```

打开标签还包括：

```text
src/centerline_extraction/src/pid_controller.cpp
src/centerline_extraction/CMakeLists.txt
src/diff_drive_robot/launch/robot.launch.py
src/centerline_extraction/src/corn_row_detector copy.cpp
```

这说明用户可能接下来会关注：

1. `main.cpp` 和节点入口；
2. `pid_controller.cpp` 路径跟踪控制；
3. `CMakeLists.txt` 编译目标；
4. 旧版本 `corn_row_detector copy.cpp` 与当前算法对比；
5. 感知中心线如何接入路径跟踪。

后续如果用户要求检查当前工程，优先查看这些文件以及当前 `CMakeLists.txt` 中实际编译了哪些节点，避免误以为运行的是已经修改过的 `corn_row_detector_projection.cpp`，但实际启动的是另一个旧节点。

## 18. 置信度与安全裕度接入 PID 控制器方案

用户确认当前测试启动链路为：

```bash
ros2 launch diff_drive_robot robot.launch.py world:=yumidiliuhang.world
ros2 run centerline_extraction corn_row_detector_projection
ros2 run centerline_extraction cornfield_navigation_node
```

实际闭环链路为：

```text
Gazebo / mid360 点云
  ↓
/mid360_PointCloud2
  ↓
corn_row_detector_projection
  ↓
/corn_row_center_line
/corridor_confidence
/corridor_safety_margin
  ↓
cornfield_navigation_node 中的 PIDController
  ↓
/cmd_vel
  ↓
Gazebo 差速驱动插件
```

用户要求先梳理如何将 `corridor_confidence` 和 `corridor_safety_margin` 接入 PID，再按该思路修改代码。

### 18.1 控制思想

中心线负责决定车辆如何转向：

```text
/corn_row_center_line -> lateral_error / heading_error -> PID angular velocity
```

置信度和安全裕度负责决定：

```text
能不能走
走多快
是否需要限制角速度
是否需要短时保持历史路径
是否需要停车
```

其中：

```text
corridor_confidence：表示当前中心线是否可信
corridor_safety_margin：表示当前通道空间是否足够安全
```

### 18.2 建议状态

建议控制器形成四种状态：

```text
NORMAL   高置信度且安全裕度足够，正常速度跟踪
CAUTION  中等置信度或安全裕度偏小，降速跟踪
RECOVERY 低置信度但仍有历史中心线，短时保持历史路径并低速
STOP     置信度太低太久或安全裕度不足，停车
```

### 18.3 速度调节公式

建议线速度由三部分共同决定：

```text
linear_speed = max_linear_speed
             * confidence_factor
             * safety_factor
             * turning_factor
```

其中：

```text
confidence_factor = clamp(confidence, confidence_min_factor, 1.0)
```

安全裕度因子可采用分段形式：

```text
margin >= safety_margin_high:      safety_factor = 1.0
safety_margin_mid <= margin < high: safety_factor = 0.6
safety_margin_stop <= margin < mid: safety_factor = 0.3
margin < safety_margin_stop:        safety_factor = 0.0
```

转向因子沿用原有逻辑：

```text
turning_factor = 1 - 0.5 * abs(angular_vel) / max_angular_speed
```

### 18.4 角速度限制

安全裕度越小，最大角速度越小：

```text
current_max_angular_speed = max_angular_speed * safety_factor
```

这可以避免窄通道中急转导致车体扫到作物。

### 18.5 低置信度历史路径保持

当当前中心线置信度低时，不应立即停车。应保存：

```text
last_valid_center_line
low_confidence_count
max_low_confidence_frames
```

逻辑：

```text
高/中置信度：
    使用当前中心线，并保存为 last_valid_center_line

低置信度但未超过 max_low_confidence_frames：
    使用 last_valid_center_line，低速前进

低置信度连续超过 max_low_confidence_frames：
    停车
```

### 18.6 对小论文的意义

接入后可以把论文从“中心线提取算法”提升为：

```text
置信度与安全裕度感知的冠下行间路径跟踪控制方法
```

对应实验可以增加：

1. 固定速度 PID vs 置信度感知 PID；
2. 普通中心线跟踪 vs 低置信度历史路径保持；
3. 无安全裕度限制 vs 安全裕度限制；
4. 对比横向误差、最大偏差、最小作物安全距离、成功率和平均速度。
