# 专利撰写话题交接记录

记录时间：2026-06-28

本文档用于在新对话中恢复“面向玉米冠下导航的感知质量约束路径跟踪方法及系统”专利撰写上下文。后续新开话题时，可直接让 Codex 读取本文件继续对接。

## 1. 当前专利主题

拟写专利名称：

一种面向玉米冠下导航的感知质量约束路径跟踪方法及系统。

核心技术方向：

面向玉米冠下无人运动平台导航，将中心线提取结果、走廊置信度 `corridor_confidence` 和安全裕度 `corridor_safety_margin` 接入路径跟踪控制，使平台在中心线可靠时正常跟踪，在中心线低置信但未完全失效时降级低速跟踪，在安全裕度不足或中心线失效时安全停车。

## 2. 专利创新点定位

当前专利不是单纯保护传统 PID，也不是单纯保护作物行中心线提取，而是保护“感知质量约束路径跟踪”的闭环方法。

主要创新点包括：

1. 将作物行中心线检测质量显式量化为走廊置信度。
2. 将通行走廊宽度与车辆宽度关系量化为安全裕度。
3. 将走廊置信度和安全裕度接入路径跟踪控制器，对线速度、角速度限幅、降级跟踪和停车逻辑进行约束。
4. 建立正常跟踪、降级跟踪、安全停车三状态控制机制。
5. 在冠下 GNSS 受限、视觉不稳定、点云局部缺失的场景下提高导航连续性和安全性。

## 3. 当前已生成文件

基础版专利草稿：

`docs/patent_quality_aware_path_tracking_under_canopy.md`

增强版技术交底书：

`docs/patent_quality_aware_path_tracking_under_canopy_enhanced.md`

可浏览公式和附图的 HTML 版：

`docs/patent_quality_aware_path_tracking_under_canopy_enhanced.html`

按模板生成的 DOCX 技术交底书：

`patent/一种面向玉米冠下导航的感知质量约束路径跟踪方法及系统_技术交底书.docx`

原始模板：

`patent/技术交底书.docx`

## 4. 当前已生成附图

附图目录：

`docs/patent_figures/`

当前附图包括：

1. `fig1_overall_flow.svg`：方法总体流程图。
2. `fig2_system_architecture.svg`：系统结构框图。
3. `fig3_error_model.svg`：横向误差和航向误差计算示意图。
4. `fig4_state_machine.svg`：正常跟踪、降级跟踪、安全停车状态转换图。
5. `fig5_control_block.svg`：质量感知路径跟踪控制框图。

每张图均保留了对应 Graphviz 源文件：

1. `fig1_overall_flow.dot`
2. `fig2_system_architecture.dot`
3. `fig3_error_model.dot`
4. `fig4_state_machine.dot`
5. `fig5_control_block.dot`

选择 SVG 而不是图片生成模型的原因：

专利附图更适合线条清晰、结构明确、可编辑和可复现的矢量图。流程图、结构图、误差模型图和控制框图不应使用写实图片生成。

## 5. 增强版文档包含的主要内容

增强版文档 `docs/patent_quality_aware_path_tracking_under_canopy_enhanced.md` 已包含：

1. 发明名称。
2. 技术领域。
3. 背景技术。
4. 发明目的。
5. 总体技术方案。
6. 关键变量定义。
7. 中心线和走廊质量计算。
8. 走廊置信度计算。
9. 安全裕度计算。
10. 目标路径点和误差模型。
11. 双误差 PID 路径跟踪控制。
12. 感知质量约束速度调节。
13. 正常、降级、停车状态机。
14. 可选曲率前馈和自适应前视扩展。
15. 权利要求建议稿。
16. 附图说明。
17. 有益效果。
18. 实验支撑。

其中公式使用 LaTeX 形式，HTML 版通过 MathJax 渲染。

## 6. 关键公式概要

点云 ROI：

$$
\mathcal{P}_{roi}=
\left\{
p_i\in\mathcal{P}
\mid
x_{\min}\le x_i\le x_{\max},
y_{\min}\le y_i\le y_{\max},
z_{\min}\le z_i\le z_{\max}
\right\}
$$

左右内侧行选择：

$$
R_L=\arg\min_{R_k:\bar y_k>0}|\bar y_k|,
\quad
R_R=\arg\min_{R_k:\bar y_k<0}|\bar y_k|
$$

走廊宽度与中心线：

$$
W=y_L-y_R,
\quad
y_c=\frac{y_L+y_R}{2}
$$

中心线参数方程：

$$
\mathbf{c}(s)=
\mathbf{p}_0
+
s
\begin{bmatrix}
\cos\theta_r\\
\sin\theta_r
\end{bmatrix}
+
y_c
\begin{bmatrix}
-\sin\theta_r\\
\cos\theta_r
\end{bmatrix}
$$

走廊置信度：

$$
C=w_p C_p+w_w C_w+w_s C_s+w_h C_h
$$

安全裕度：

$$
M=\frac{W-B}{2}-B_s
$$

目标点：

$$
s^\ast=\arg\min_s \left\|\mathbf{c}(s)-\mathbf{p}_r\right\|,
\quad
\mathbf{p}_t=\mathbf{c}(s^\ast+L_d)
$$

角速度控制：

$$
\omega =
\operatorname{sat}
\left(
\omega_y+\omega_\psi,\,
-\omega_{\max}(M),\,
\omega_{\max}(M)
\right)
$$

质量约束线速度：

$$
v = v_{\max}\cdot f_C(C)\cdot f_M(M)\cdot f_\omega(|\omega|)
$$

## 7. 当前实验支撑

已有质量感知 PID 与普通 PID 对比实验记录：

`docs/pid_quality_control_experiment_record.md`

`docs/zhenshi4hang16m_quality_vs_baseline_analysis.md`

结论要点：

1. 质量感知控制并未显著降低厘米级横向误差，因为普通 PID 在中心线有效时也能跟踪。
2. 质量感知控制的主要优势是减少低质量阶段的停车比例，提高中心线可用率和安全裕度。
3. 该结论适合作为专利“有益效果”和论文实验支撑：不是强调极限跟踪误差降低，而是强调感知不确定场景下控制连续性、安全性和鲁棒性提升。

已有 PID 路径跟踪精度测试文档：

`docs/pid_tracking_accuracy_test.md`

最新有效测试结论：

1. 机器人到达参考路径终点，停止原因为 `reached_reference_goal`。
2. 总体横向误差 RMSE 约为 0.0185 m。
3. 第一段直线 RMSE 约为 0.0026 m。
4. 圆弧段 RMSE 约为 0.0292 m。
5. 第二段直线 RMSE 约为 0.0077 m。

注意：旧结果中曾出现第二段直线 RMSE 约 1.0441 m，原因是路径连续性和停止逻辑未修正，已被新结果取代。

## 8. 后续建议继续完善方向

专利文本还可以继续加强以下内容：

1. 将增强版 Markdown 内容进一步整理成正式“技术交底书”结构。
2. 将权利要求从“建议稿”扩展为更完整的独立权利要求和从属权利要求。
3. 增加“实施例”部分，结合当前 ROS2/Gazebo 项目描述传感器、节点、话题、参数和控制流程。
4. 增加“对比现有技术”的段落，突出区别于：
   - 单纯作物行识别方法；
   - 单纯 PID、Pure Pursuit 或 Stanley 路径跟踪；
   - 只根据横向误差控制而不考虑感知质量的方法；
   - 只在感知失效时停车而无降级跟踪机制的方法。
5. 增加“可替代实施方式”，例如置信度可由深度学习分割置信度、点云残差、左右行连续性、历史一致性等指标构成。
6. 根据专利代理人要求，可把 SVG 附图导出为 PNG 或插入 DOCX。

## 9. 新话题继续方式

新开对话后可以这样说：

“请读取 `/home/xkai/agribot/agribot_ws/docs/patent_topic_handoff.md`，继续帮我完善这个专利。”

建议下一步优先任务：

1. 把增强版 Markdown 改成更像正式技术交底书的完整版本。
2. 补充具体实施例和权利要求。
3. 生成一个含公式和附图的可提交预览版本。

