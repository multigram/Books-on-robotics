# 机器人研发系统学习路线（版本：2025-09-28）

> 本文档整理自学习路线建议，可持续迭代。建议每周回顾 & 更新。 
> 建议配套建立：`logs/`（周日志）、`data/`（实验数据与 rosbag）、`scripts/`（分析脚本）。

---
## 目录
1. 总览阶段
2. 已有技能迁移策略
3. 知识图谱
4. 分阶段路线（含时间假设）
5. 前 8 周周计划示例
6. 实践项目建议
7. 工具与生态
8. 推荐学习资源
9. 评估指标体系
10. 常见踩坑与规避
11. 个性化当前行动（本周执行）
12. 精简版（每周 5 小时）优先级
13. 学习方法策略
14. 进阶延伸方向
15. 更新记录模板
16. 书籍与系统阅读路线（扩展）

---
## 1. 总览阶段
1. 夯实通用基础：数学 / 物理 / C/C++ / Python / 线代 / 控制
2. 硬件与传感器：电机/驱动/编码器/IMU/激光雷达/摄像头
3. 控制与运动学：正逆运动学、动力学、轨迹规划、控制（PID→LQR→MPC）
4. 嵌入式与实时系统：RTOS、总线协议（CAN/UART/SPI/I2C）、时间同步
5. ROS2 与软件架构：节点、Topic、Service、Action、TF、Launch
6. 感知与定位：视觉、激光/融合、SLAM（Cartographer / ORB-SLAM2/3）
7. 路径规划与导航：A*、RRT*、TEB、行为决策
8. 机器学习应用：检测、分割、强化学习
9. 仿真与数字孪生：Gazebo / Isaac Sim / Webots / Ignition + URDF/Xacro
10. 工程化与产品化：集成、测试（SIL/HIL/MIL）、CI/CD、部署、安全
11. 进阶：多机器人、机械臂、先进 SLAM、嵌入式 AI、边缘计算

### 控制算法层次说明（PID → LQR → MPC）
本路线中“控制（PID→LQR→MPC）”表示控制策略的三层递进：

| 算法 | 核心思想 | 优点 | 局限 | 典型用途 |
|------|----------|------|------|----------|
| PID | 直接对误差做比例/积分/微分叠加 | 简单易实现、调试快 | 无模型预测、难处理耦合与约束 | 单变量速度/姿态环、底层驱动 |
| LQR | 基于线性状态空间最小化二次型代价 J=∫(x?Qx+u?Ru) | 系统性设计、多变量耦合好 | 需要(线化)模型，不能显式约束 | 姿态/位姿稳定、耦合子系统 |
| MPC | 每周期滚动求解带约束优化预测未来 | 可处理约束、多目标、轨迹平滑 | 计算耗时、模型与延迟敏感 | 轨迹跟踪、自动驾驶局部控制 |

递进原因：PID → 经验式反馈；LQR → 理论最优状态反馈；MPC → “在线小规划器”同时考虑约束与未来轨迹。

简化数学（连续/离散形式略）：
- PID: u = Kp e + Ki ∫e dt + Kd de/dt
- LQR: 解黎卡提方程 A?P + P A ? P B R?? B? P + Q = 0，K = R?? B? P，u = ?K x
- MPC: min Σ (x?? Q x? + u?? R u?)  s.t. x_{i+1}=A x_i + B u_i, 约束 x,u

何时升级：
- PID → LQR：出现多变量耦合（姿态+速度）、PID 难兼顾超调与稳态。
- LQR → MPC：需要显式输入/状态/平滑/安全距离约束或高性能轨迹跟踪。

最小实践路径：
1. PID 阶跃速度环：调到超调 <10%，稳态误差≈0
2. 推导/线化差速车模型 → 求 LQR K，比较不同 Q 权重影响
3. 构建无约束 MPC（预测步长 N）验证与 LQR 性能接近
4. 加入输入/状态约束与 Δu 平滑项，用 OSQP / acados / CasADi 实现

常见坑：
- PID 积分饱和：增加积分限幅 / anti-windup
- LQR 不稳定：模型/线化或采样周期错误 → 校验 A,B 与频率
- MPC 求解慢：缩短时域、降维、预热求解器、使用稀疏 QP 求解器

迁移建议：先彻底掌握 PID 调参与数据分析，再学习状态空间 + LQR（帮助理解 Q/R 与权衡），最后实现一个“小型线性 MPC”，再考虑非线性/约束丰富场景。

### SLAM 核心说明（Simultaneous Localization And Mapping）
“SLAM（Cartographer / ORB-SLAM2/3）”指机器人在未知环境中一边估计自身位姿（Localization），一边构建环境地图（Mapping）的同步过程。其价值：提供全局一致的位姿参考（map→odom）与可被导航 / 规划复用的地图表示。

核心模块拆解：
1. 传感器输入层：Lidar 扫描 / 图像帧 / IMU / 里程计 / Wheel Encoder
2. 前端(Tracking / Odometry)：相邻帧匹配（特征/光流/ICP/NDT）生成相对位姿增量（约束）
3. 回环检测(Loop Closure)：发现“曾经到过”区域，加入闭合约束降低累计漂移
4. 后端优化(Back-End / Pose Graph / Bundle Adjustment)：全局图优化（g2o / Ceres）求解所有关键帧位姿使误差最小
5. 地图构建(Map Representation)：2D Occupancy Grid / 3D 点云 / OctoMap / TSDF / 栅格+语义层
6. 位姿表示：SE(2)/SE(3)，旋转常用 Quaternion 或 SO(3) 李代数（log/exp）

常见分类与代表：
| 类型 | 传感器 | 代表算法 | 特点 |
|------|--------|----------|------|
| 2D Lidar SLAM | 2D 激光 | GMapping / Hector / Cartographer(2D) | 平面场景成熟，适合室内导航 |
| 3D Lidar SLAM | 3D 激光 | LOAM / FAST-LIO / LIO-SAM | 精度高，抗光照，硬件成本高 |
| 视觉特征 SLAM | 单/双目/RGB-D | ORB-SLAM2/3 | 轻量，受光照/纹理影响 |
| 视觉惯性 SLAM (VIO) | 相机 + IMU | VINS-Mono / ORB-SLAM3 VIO | 抗短时退化，尺度更稳定 |
| 多传感融合 | Lidar + IMU + Odom | Cartographer / LIO-SAM | 鲁棒性与稳定性提升 |

选择建议（初学顺序）：
1. 2D 室内移动机器人：Cartographer（配置即用）
2. 有双目/RGB-D 相机：ORB-SLAM2/3（特征理解直观）
3. 需要高精度 & 室外：Lidar + IMU（LIO-SAM / FAST-LIO）
4. 低成本 + 有 IMU：VINS-Mono（学习多传感融合与滑动窗口 BA）

关键性能指标：
- ATE (Absolute Trajectory Error)：整段轨迹全局偏差
- RPE (Relative Pose Error)：短窗口相对位姿漂移 / 稳定性
- 回环召回率：回环检测成功率 / 误检率
- 漂移率：% / m 或 % / min（无回环时评估）
- 计算延迟：前端 / 后端耗时，实时性（Hz）

常见坑与定位策略：
| 问题 | 现象 | 排查重点 |
|------|------|----------|
| 时间同步不准 | 轨迹抖动/发散 | 对齐硬件时间戳 / 软件插值 |
| 传感器外参不准 | 地图扭曲 | 标定（相机-IMU / Lidar-IMU）|
| IMU 噪声参数错误 | 漂移速度异常 | 调整噪声/随机游走值 |
| 回环误检 | 突然大跳变 | 降低阈值 / 增加几何一致性校验 |
| 特征退化 | 空墙/弱纹理 | 融合 IMU / 切换 Lidar 辅助 |

学习路径最小实践：
1. 数据回放：下载公开 rosbag（如 EuRoC / KITTI），用 Cartographer 或 ORB-SLAM2 直接跑
2. 修改参数：调分辨率 / 特征数 / 回环阈值观察指标变化
3. 评估：使用 evo 工具计算 ATE / RPE
4. 增量理解：打开源代码入口（前端 tracking）→ 约束生成 → 图优化调用栈
5. 自建数据：用自己机器人录制 bag，对比官方数据表现差异（噪声/振动/畸变）

与导航关系：
- SLAM 输出 map + 关键帧位姿 + TF（map→odom）
- 局部规划使用 odom 框架短期稳定，全球定位使用 map 框架一致性

最小落地清单（建议写入学习日志）：
- [ ] 选择 1 套 2D（Cartographer）+ 1 套视觉（ORB-SLAM2）
- [ ] 各自完成：成功运行 / 生成地图截图 / 记录 ATE
- [ ] 修改 3 个关键参数前后对比曲线
- [ ] 撰写一页比较表（优缺点 + 适用场景 + 参数敏感项）

推荐先做：Cartographer 2D（理解约束 & 优化图）→ ORB-SLAM2（特征/回环/BA）→ VINS-Mono（滑动窗口 + IMU）→ 需要高精度再看 LIO-SAM。

---
## 2. 已有技能迁移策略
| 现有经验 | 可迁移方向 | 说明 |
|----------|------------|------|
| C 嵌入式驱动 | 底层电机/传感器采集 | 直接复用定时/中断/寄存器经验 |
| I2C/SPI/UART/NFC | 机器人外设/传感器/调试链路 | 快速接入 IMU/编码器/雷达模块 |
| 内存/性能优化 | 实时控制循环调度 | 优化调度、减少抖动与延迟 |
| 固件结构化 | ROS2 节点模块化 | 一致的软件分层思想 |

优先补齐：运动学/控制理论 → 学 ROS2 → 传感器融合。

---
## 3. 知识图谱（建议做思维导图）
- 数学：矩阵 / SVD / SO(3)/SE(3) / 雅可比 / 概率 / 贝叶斯 / KF(EKF/UKF)/粒子滤波 / 优化（非线性 least squares）
- 控制：PID → 状态空间 → LQR → MPC → 增强/自适应控制
- 运动学：差速 / 麦克纳姆 / Ackermann / 机械臂（DH、逆解、动力学）
- 感知：特征（ORB/SIFT）、光流、PnP、回环、深度获取（双目/结构光/雷达）
- 融合：IMU + 里程计 + Lidar + Camera (LOAM / VIO / VINS)
- 地图：Grid / Octomap / TSDF / NDT
- 规划：Dijkstra / A* / Hybrid A* / RRT* / TEB / MPC 局部优化
- 决策：有限状态机 / 行为树 / 分层架构
- 工程：ROS2 QoS、生命周期节点、零拷贝、实时调度、HIL/SIL/MIL

---
## 4. 分阶段路线（假设每周 10–15 小时）
| 阶段 | 周数 | 目标 | 产出 |
|------|------|------|------|
| 0 对齐 | 1–2 | 环境 + 坐标系 + 基础 ROS2 | 运行 TurtleBot3 nav2 demo |
| 1 控制/运动 | 3–6 | 差速运动学 + PID + 里程计 + EKF | 自建 URDF，小车能稳定跟随速度 |
| 2 感知定位 | 7–12 | SLAM + 融合 + 数据回放 | Cartographer / ORB-SLAM2 跑通 + 误差对比 |
| 3 规划导航 | 13–20 | 全局/局部规划 + 行为层 | A*/RRT 实现 + nav2 调参报告 |
| 4 工程化 | 21–28 | Docker + CI + 指标监控 | CI 流水线 + 统计脚本 |
| 5 进阶 | 29–36 | 深度感知 + MPC + 性能 | MPC 跟踪 + 目标检测辅助避障 |

---
## 5. 前 8 周周计划示例
| 周 | 学 | 做 | 输出 |
|----|----|----|------|
| 1 | ROS2 基础 | 运行 turtlesim / nav2 | 节点与话题结构图 |
| 2 | TF2 坐标树 | 伪里程计发布节点 | Rviz TF 可视化截图 |
| 3 | 差速运动学 | 编码器噪声仿真 | 误差曲线图 |
| 4 | PID 整定方法 | 速度环仿真 | 超调/稳态误差报告 |
| 5 | EKF 理论 | 2D EKF 实作 | 单元测试 + 轨迹对比 |
| 6 | Costmap 结构 | 障碍 + nav2 路径 | 调参笔记 |
| 7 | A* / Dijkstra | 手写栅格 A* | 路径长度/耗时对比 |
| 8 | RRT* 原理 | 基础 RRT | 多次统计报告 |

---
## 6. 实践项目建议
1. 仿真差速机器人导航
   - 产物：URDF/Xacro + nav2 启动 + 控制节点
   - 指标：随机 3 组起终点成功率 ≥ 90%
2. 多传感器定位融合
   - 指标：RMSE < 0.15 m（数据集回放）
3. 路径规划算法对比平台
   - 指标：10 张地图平均耗时 & 路径长度统计
4. 视觉辅助避障（YOLO + costmap 动态层）
   - 指标：动态障碍避障成功率 ≥ 95%
5. MPC 轨迹跟踪
   - 指标：最大偏差 < 0.05 m

---
## 7. 工具与生态
- 语言：C++17/20、Python
- 库：Eigen、Ceres/g2o、BehaviorTree.CPP、OpenCV、PCL
- 中间件：ROS2 (Humble) + DDS QoS 调参
- 仿真：Gazebo / Ignition / Webots / Isaac Sim
- 可视化：RViz2、PlotJuggler、rqt 工具集
- 测试：GTest、rosbag2、数据分析脚本（pandas + matplotlib）
- 部署：Docker 多阶段、交叉编译（aarch64）、TensorRT（可选）
- 性能：perf / tracing、fastdds monitor

---
## 8. 推荐学习资源
| 类别 | 资源 |
|------|------|
| 数学/控制 | Modern Robotics；Probabilistic Robotics；Feedback Systems |
| 规划 | Planning Algorithms；《机器人学：建模、规划与控制》 |
| SLAM | VINS-Mono、ORB-SLAM2/3、相关李群笔记 |
| ROS2 | 官方文档、nav2 源码结构 |
| AI | YOLOv8、Detectron2、Spinning Up in Deep RL |

---
## 9. 评估指标体系
| 维度 | 指标 | 目标示例 |
|------|------|----------|
| 代码质量 | clang-tidy 违规数 | < 5 / KLOC |
| 实时性 | 控制循环抖动 | < ±2 ms |
| 定位 | 轨迹 RMSE | < 0.1 m（10×10 m 室内） |
| 导航 | 成功率 | ≥ 95% |
| 规划 | 路径长度相对最优 | < +10% |
| 推理 | 模型延迟 | < 30 ms/frame (Arm) |
| 鲁棒性 | 掉帧恢复时间 | < 2 s |

---
## 10. 常见踩坑与规避
1. 过早堆叠复杂 → 先最小链路（感知→定位→规划→控制）
2. TF 关系混乱 → 严格 map→odom→base_link→sensor
3. 消息延迟无监控 → 记录时间戳差异 & 日志
4. PID 与执行耦合 → 分离模型与驱动层
5. SLAM 频繁丢失 → 校准/同步/畸变补偿先做扎实
6. 只在仿真验证 → 真机 ≥ 30% 时间
7. 数据无归档 → 统一 rosbag 命名 + 自动脚本

---
## 11. 个性化当前行动（本周）
- 新建本文件（已完成）
- 安装 Ubuntu + ROS2 Humble 环境
- 最小 C++ 节点发布 IMU 伪数据
- 修改 TurtleBot3 URDF 尺寸参数并重新加载 RViz
- 阅读 nav2 `costmap2d` 目录结构并列出子模块职责

> 可选扩展：建立 `scripts/analyze_bag.py` 用于误差对比。

---
## 12. 精简版（每周 5 小时）优先顺序
1. ROS2 Topic/TF 基础
2. 差速运动学与里程计
3. EKF（robot_localization）
4. A* + nav2 调参
5. SLAM（Cartographer）
6. TEB / MPC 局部优化
7. 深度学习感知

---
## 13. 学习方法策略
- 任务驱动：每学一概念 → 做最小可视化
- 周复盘：问题 / 假设 / 验证 / 结论
- 数据回放：固定输入 → 参数网格对比
- 源码阅读顺序：README → 架构 → 主循环 → 细节
- 版本演进：v0_baseline → v1_fusion → v2_planner → v3_mpc

---
## 14. 进阶延伸方向
- 多机器人协同：分布式 SLAM、任务分配（拍卖算法/MILP）
- 语义地图：检测结果融合占据栅格
- 世界模型 & 强化学习：Dreamer / Diffusion Policy
- 推理优化：ONNX → TensorRT → INT8 校准
- 实时：PREEMPT_RT + Executor 策略调整

---
## 15. 更新记录模板
```
### [2025-09-28] 第一次创建
- 初始化学习路线框架
- 制定前 8 周计划

### [YYYY-MM-DD] 更新
- [变更] …
- [新增资源] …
- [下一步] …
```

---
> 后续可按阶段拆分为多文件（如 `docs/slam.md`, `docs/planning.md`）。需要更多示例代码或脚手架可继续提出。

---
## 16. 书籍与系统阅读路线（扩展）
本节对“推荐学习资源”进行深度扩展：按照基础→数学→控制→感知/SLAM→规划→系统工程→优化/前沿给出书目与使用方式。可按需裁剪。

### 16.1 快速起步必读（建立全局框架）
| 书/资源 | 作用 | 建议阅读策略 |
|---------|------|--------------|
| Modern Robotics (Lynch) | 统一位姿表示/运动学/规划概念 | 第 2–8 章精读，后面按需回看 |
| Probabilistic Robotics (Thrun) | 滤波 / 定位 / SLAM 基石 | 第 2/3/4/7/10 章优先 |
| Planning Algorithms (LaValle) | 规划算法全景 | 前言 + 采样/差分约束章节 |

精简路径：Modern + Probabilistic（滤波核心）+ 实操 nav2/Cartographer，再回补理论。

### 16.2 数学与优化
| 主题 | 资源 | 说明 |
|------|------|------|
| 线性代数 | Linear Algebra Done Right | 概念清晰强化理解 |
| 数值线性代数 | Matrix Computations (Golub) | 做优化/SLAM 后端时参考 |
| 优化 | Numerical Optimization (Nocedal) | 关注线性/非线性最小二乘章节 |
| 李群与状态估计 | State Estimation for Robotics (Barfoot) | SE(3) 扰动模型/雅可比 |
| 李群速览 | Sola Lie Notes (免费) | 快速查公式 |
| 概率滤波 | Bayesian Filtering and Smoothing (S?rkk?) | KF/EKF/UKF/连续时间建模 |

### 16.3 建模 / 控制
| 资源 | 作用 | 重点 |
|------|------|------|
| Robot Modeling and Control (Spong) | 动力学 / 控制 | 拉格朗日、关节空间控制 |
| Robotics: Modelling, Planning and Control (Siciliano) | 覆盖面广 | 可与 Spong 互补 |
| Feedback Systems (?str?m & Murray) | 控制结构与直觉 | 状态空间/稳定性/频域 |
| Borrelli MPC | 线性 MPC 基础 | 预测时域、约束建模 |
| Rawlings & Mayne (Practical MPC) | 工程实践 | 约束处理与软约束 |

### 16.4 感知 / 视觉 / SLAM
| 类型 | 资源 | 用途 |
|------|------|------|
| 相机几何 | Multiple View Geometry (HZ) | 投影/三角化/位姿估计基础 |
| 视觉总览 | Computer Vision (Szeliski, 2nd) | 特征、光流、SfM、立体 |
| 视觉 SLAM | 视觉 SLAM 14 讲 | 中文入门捷径，后回论文验证 |
| 后端优化 | Ceres / g2o 文档 + BA 论文 | 残差 / 雅可比 / 结构化稀疏 |
| VIO | VINS-Mono / ORB-SLAM3 论文 | IMU 融合滑动窗口思路 |
| Lidar 融合 | LOAM / LIO-SAM / FAST-LIO | 高精度里程计与回环策略 |

### 16.5 规划 / 导航 / 行为
| 资源 | 范畴 | 关注点 |
|------|------|------|
| LaValle | 采样 / 差分约束 | RRT/RRT* 理论界限 |
| OMPL 源码 | 采样规划实现 | 抽象接口/状态空间/PlannerBase |
| Behavior Trees in Robotics & AI | 行为树理论 | 节点类型/形式化语义 |
| TEB 论文与源码 | 局部优化 | 时间弹性带约束建模 |
| Hybrid A* 论文 | 曲率约束路径 | 搜索 + 连续插值 |

### 16.6 系统与工程化
| 主题 | 资源 | 说明 |
|------|------|------|
| ROS2 | 官方文档 / nav2 源码 | QoS / BT Navigator / costmap2d |
| C++ 实践 | Effective Modern C++ / Clean Code | 可维护性与接口设计 |
| 性能调优 | Linux Performance / Brendan Gregg | 采样 & tracing 手段 |
| 实时系统 | Real-Time Systems (Buttazzo) | 调度 / 抖动控制 |
| 架构 | Designing Data-Intensive Applications | 日志 / 一致性 / 可靠性 |

### 16.7 嵌入式与边缘推理
| 资源 | 说明 |
|------|------|
| Efficient Processing of DNNs (Sze) | 模型计算特征与硬件映射 |
| TensorRT / ONNX Runtime Docs | 推理部署与算子优化 |
| Real-Time Embedded Systems | 任务划分与调度策略 |

### 16.8 强化学习 / 前沿
| 主题 | 资源 | 重点 |
|------|------|------|
| 强化学习 | Sutton & Barto / Spinning Up | Policy / Value / Advantage |
| 世界模型 | Dreamer 系列论文 | 潜在动态建模 |
| 生成策略 | Diffusion Policy 论文 | 轨迹生成鲁棒性 |

### 16.9 “高性价比 8 本”清单
1. Modern Robotics  
2. Probabilistic Robotics  
3. Robotics: Modelling, Planning and Control（或 Spong 二选一）  
4. Computer Vision (Szeliski, 选读)  
5. Planning Algorithms（选读）  
6. Feedback Systems  
7. Borrelli MPC  
8. Behavior Trees in Robotics & AI（若重导航；否则用 Barfoot 替换）  

### 16.10 目标导向裁剪
| 目标 | 必读 | 暂缓 |
|------|------|------|
| 移动机器人导航 | Modern / Probabilistic / LaValle / Behavior Tree | 机械臂动力学全章 |
| SLAM 方向 | Probabilistic / Szeliski / 14 讲 / Barfoot / Ceres | MPC 深入 |
| 控制优化 | Modern / Feedback Systems / Borrelli / Nocedal | 视觉高级细节 |
| 边缘部署 | Modern / Probabilistic(滤波) / 性能调优书 | 深度 SLAM 全套 |

### 16.11 每本“读完”验证产出
| 书 | 最小产出验证 |
|----|---------------|
| Modern Robotics | 差速+麦克纳姆运动学仿真 & TF 可视化 |
| Probabilistic Robotics | 2D EKF + 粒子滤波定位对比误差曲线 |
| Szeliski | ORB 特征匹配 + 两帧位姿 PnP + RANSAC |
| LaValle | RRT/RRT* 实现 + 节点数 vs 障碍密度统计 |
| Borrelli | LQR vs MPC 跟踪误差/控制平滑度对比图 |
| Barfoot | 实现 SE3 exp/log + 小型 pose graph 优化 |
| Behavior Trees | 导航行为树：巡航→避障→返回充电 |

### 16.12 一周样例（首周）
| 日 | 任务 | 产出 |
|----|------|------|
| 1 | Modern 第 2 章 姿态表示 | 四元数/矩阵互转函数 + 单元测试 |
| 2 | Modern 第 3 章 运动学 | 差速正运动学 + 逆解校验脚本 |
| 3 | Probabilistic 第 2/3 章 | 1D KF 仿真误差曲线 PNG |
| 4 | Szeliski 特征章节 | ORB 匹配可视化图 |
| 5 | LaValle 采样章节 | RRT 动画可视化 GIF |
| 6 | 整理与复盘 | learning log 更新 |
| 7 | 疑问清单整理 | Q&A 列表输入下一周计划 |

### 16.13 阅读方法要点
- 抓“表示 / 误差模型 / 目标函数”三件套
- 先运行开源代码→再倒推理论；避免纯理论空转
- 建立术语索引表（SO(3), SE(3), Residual, Jacobian, Constraint, Cost）
- 每章设“退出条件”：实现一个最小脚本或可视化

### 16.14 后续可拆文件
- `docs/books.md`：如果内容继续扩展或需单独维护
- `docs/reading_log/`：按周记录（命名：YYYYWW.md）

> 版权提示：仅做学习笔记，不在仓库中存放原书扫描或大量原文复制；引用公式请注明来源章节。

