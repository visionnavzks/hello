# Motion Planner · 架构设计文档

> 一个面向差速 / 阿克曼式移动机器人的**分层 2D 运动规划栈**。  
> 全程只优化机器人的几何中心,只在碰撞校验时展开为真实底盘足迹(圆 / 矩形)。

---

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 核心设计理念](#2-核心设计理念)
- [3. 流水线全景](#3-流水线全景)
- [4. 数据结构与类型系统](#4-数据结构与类型系统)
- [5. 配置系统](#5-配置系统)
- [6. 模块详解](#6-模块详解)
  - [6.1 ESDF — 距离场构建](#61-esdf--距离场构建)
  - [6.2 A* — 中心路径全局搜索](#62-a--中心路径全局搜索)
  - [6.3 Shortcut — 折线去冗余](#63-shortcut--折线去冗余)
  - [6.4 Resample — 弧长均匀化](#64-resample--弧长均匀化)
  - [6.5 ESDF Smoother — XY 梯度平滑](#65-esdf-smoother--xy-梯度平滑)
  - [6.6 RS / Dubins Planner — 运动学航段生成](#66-rs--dubins-planner--运动学航段生成)
  - [6.7 SE(2) Smoother — 流形上的轨迹优化](#67-se2-smoother--流形上的轨迹优化)
  - [6.8 Validator — 三级碰撞检查](#68-validator--三级碰撞检查)
  - [6.9 Footprint — 底盘抽象](#69-footprint--底盘抽象)
  - [6.10 Pipeline — 组合外观](#610-pipeline--组合外观)
- [7. Web Demo 架构](#7-web-demo-架构)
- [8. API 参考](#8-api-参考)
- [9. 测试策略](#9-测试策略)
- [10. 扩展指南](#10-扩展指南)
- [11. 已知的局限与未来工作](#11-已知的局限与未来工作)

---

## 1. 项目概述

### 1.1 目标

为差速 / 阿克曼式移动机器人提供一套**端到端**的二维运动规划方案:输入一张静态 2D 地图(`MapSpec`)、起点 / 终点位姿(`Pose2D`)、机器人底盘(`Footprint`),输出一条**满足曲率、速度、加速度、jerk 约束且无碰撞**的 SE(2) 轨迹。

### 1.2 适用场景

- 室内 / 半结构化环境下的服务机器人、AGV 路径规划
- 需要在曲率约束下生成可执行轨迹的自动驾驶教学 / 仿真
- 作为高层规划结果做二次精修的离线规划器

### 1.3 非目标(v0.1)

- 动态障碍物(已有 `ESDF.update_obstacle` 桩接口,未实现)
- ROS / `nav_msgs` 桥接
- 严格阿克曼约束(目前只施加曲率上限,未区分前后转向)
- 3D 规划

---

## 2. 核心设计理念

### 2.1 中心 — 足迹分离原则

> **规划阶段只关心机器人的几何中心;只有碰撞校验才把足迹展开。**

这一原则带来三个好处:

1. **降维**:ESDF、A*、Shortcut、ESDF 平滑都在 **XY 二维空间** 进行,无需考虑朝向。
2. **解耦**:运动学约束(曲率、heading)在 RS / SE(2) 阶段引入,与早期路径无关。
3. **可复用**:同一份中心路径可与不同 footprint 重新校验,便于快速迭代安全距离。

### 2.2 分层精修

每一层只解决一类问题,层与层之间通过**显式数据结构**(`Path` / `SE2Trajectory`)解耦:

| 阶段 | 关注点 | 输出 |
| --- | --- | --- |
| ESDF | 距离场 | `(ny, nx)` 浮点数组 + 梯度查询 |
| A* | 拓扑可达性 + 粗成本 | 中心折线 |
| Shortcut | 去除冗余折点 | 稀疏折线 |
| Resample | 顶点密度归一 | 均匀弧长折线 |
| ESDF smooth | 平滑 + 离墙 | 平滑折线 |
| RS / Dubins | 朝向 + 曲率约束 | SE(2) 轨迹 |
| SE(2) smooth | 速度 / 加速度 / jerk | 最终轨迹 |
| Validator | 安全验证 | 校验报告 |

### 2.3 验证与修复的本地化

三级 Validator 不只返回成功 / 失败,还会返回**故障索引区间** `fault_range`,为后续**局部修复**留下入口。`HierarchicalPlanner` 在 L3 失败时会触发**有限次 SE(2) 再平滑**作为兜底,而**不重跑**整条流水线。

---

## 3. 流水线全景

```
┌────────────────────────────────────────────────────────────────────┐
│                          MapSpec (polygons, bounds)                │
└─────────────────────┬──────────────────────────────────────────────┘
                      ▼
       ┌──────────────────────────────┐
       │  1. ESDF                     │  ← distance_transform_edt
       │     occupancy → signed φ     │     双向 EDT,正距离=自由,负=障碍
       └────────────┬─────────────────┘
                    ▼
       ┌──────────────────────────────┐
       │  2. A* 2D                    │  ← 8-邻域堆搜索
       │     ESDF 斥力 + clearance    │     输出折叠共线后的中心折线
       └────────────┬─────────────────┘
                    ▼
       ┌──────────────────────────────┐
       │  3. Shortcut                 │  ← 顺序 + 随机
       │     双层碰撞检查             │     粗(包围圆)+ 精(footprint 采样)
       └────────────┬─────────────────┘
                    ▼
       ┌──────────────────────────────┐
       │  4. Resample                 │  ← 等弧长 ds
       │     端点保留                  │     给 smoother 留足够自由度
       └────────────┬─────────────────┘
                    ▼
       ┌──────────────────────────────┐
       │  5. ESDF smoother (XY)       │  ← CasADi + IPOPT(硬约束)
       │     曲率 + 斥力 + 锚点        │     ESDF / 曲率 全部 hard
       └────────────┬─────────────────┘
                     ▼
       ┌──────────────────────────────┐
       │  6. RS / Dubins              │  ← 自实现 6-pattern Dubins
       │     自适应锚点 + heading 弛豫 │     OMPL 兼容 API
       └────────────┬─────────────────┘
                     ▼
       ┌──────────────────────────────┐
       │  7. SE(2) smoother           │  ← CasADi + IPOPT(硬约束)
       │     push-out + 多项代价      │     ESDF + 车式曲率 全部 hard
       └────────────┬─────────────────┘
                    ▼
       ┌──────────────────────────────┐
       │  8. 3-level Validator        │  ← L1: 平滑 XY + 圆
       │     + 本地修复                │     L2: RS 轨迹 + 圆
       │                              │     L3: 最终 + footprint 全采样
       └──────────────────────────────┘
```

**单一总入口**:`HierarchicalPlanner.plan(start, goal)`(`motion_planner/pipeline.py:71`)返回 `PipelineResult`,包含 6 个中间产物 + 3 个验证结果。

---

## 4. 数据结构与类型系统

类型全部定义在 `motion_planner/types.py`,刻意保持**扁平、无依赖、可被 numpy 序列化**。

| 类型 | 内部表示 | 用途 |
| --- | --- | --- |
| `Pose2D` | `(x, y, theta)` 三个标量 | 单帧位姿,起点 / 终点 / 锚点 |
| `Path` | `(N, 2)` numpy 数组 | 中心折线(A* / Shortcut / Resample / Smooth) |
| `SE2Trajectory` | `(N, 3)` numpy 数组 | SE(2) 轨迹(RS / SE(2) smooth) |
| `MapSpec` | `polygons: List[Polygon]`, `bounds: (xmin, ymin, xmax, ymax)` | 静态地图规格,可 `to_dict/from_dict` 与 JSON 互转 |

辅助方法:

- `Pose2D.as_array()` / `Pose2D.from_array()`
- `SE2Trajectory.positions()` → `(N, 2)`, `.headings()` → `(N,)`
- `MapSpec.to_dict()` / `MapSpec.from_dict()` 用于 Web API 序列化

**约定**:`Path` 与 `SE2Trajectory` 在 `__post_init__` 中强校验维度,避免下游 shape 错误。

---

## 5. 配置系统

`motion_planner/config.py` 提供 `PlannerConfig` dataclass,集中管理所有可调参数。设计目标:**调参不需修改任何模块代码**。

按功能分组:

| 分组 | 关键字段 | 含义 |
| --- | --- | --- |
| ESDF | `esdf_resolution`, `safety_margin` | 网格分辨率,叠加安全边距 |
| Footprint | `default_radius` / `width` / `height` | 默认圆 / 矩形尺寸 |
| A* | `astar_resolution`, `astar_obstacle_weight`, `astar_heuristic_weight`, `astar_max_iter` | 网格、斥力、启发、迭代上限 |
| Shortcut | `shortcut_iters`, `shortcut_random_frac`, `shortcut_coarse_margin` | 随机迭代次数、圆检查额外 slack |
| Resample | `resample_step` | 弧长间隔 (m) |
| ESDF smooth | `smooth_w_*`, `smooth_iters` | 平滑 / 斥力 / 曲率 / 锚 / 长度 权重 |
| RS | `rs_delta_heading`, `rs_heading_samples`, `rs_min_turn_r`, `rs_anchor_spacing`, `rs_anchor_min`, `rs_step`, `rs_lookahead_*` | heading 弛豫幅度与采样数、最小转弯半径、锚点间距 / 下限、插值步长、首尾前瞻 |
| SE(2) smooth | `se2_w_*`, `se2_iters`, `se2_kappa_max`, `se2_v_max`, `se2_a_max`, `se2_j_max`, `se2_omega_max`, `se2_alpha_max` | 多项代价权重 + 运动学极限 |
| Validator | `validator_samples`, `validator_local_repair_iters` | L3 密集采样数、失败后 SE(2) 再平滑轮数 |
| Reproducibility | `random_seed` | 全局随机种子 |

`default_config()` 返回一份深拷贝,适合做"基线 + 局部微调"。

---

## 6. 模块详解

### 6.1 ESDF — 距离场构建

**文件**:`motion_planner/esdf.py`  
**入口**:`ESDF(map_spec, cfg)`(esdf.py:29)

**原理**

1. 栅格化:`bounds` 内以 `esdf_resolution`(默认 5 cm) 切分 cell,cell 中心为采样点。
2. 占用标记:用 `shapely.prepared.prep` 加速逐多边形 point-in-polygon,得到布尔 `occupancy`。
3. 双向 EDT:对 `~occupancy` 和 `occupancy` 各做一次 `scipy.ndimage.distance_transform_edt`,按符号拼接成有符号距离 `field`(自由区 `+`,障碍内 `-`)。
4. 连续查询:`_bilinear` 在 4 个 cell 中心做双线性插值,提供 `query(x, y)` 与向量化版 `query_batch(pts)`。
5. 梯度:`query_grad` 在插值后的场上做中心差分;当点**位于障碍内部**导致梯度为零时,会按角度等距扩大采样环直到逃出障碍再回退梯度。

**API**

| 方法 | 用途 |
| --- | --- |
| `world_to_grid / grid_to_world` | 世界 ↔ 栅格坐标 |
| `query(x, y)` | 单点距离(双线性) |
| `query_grad(x, y)` | 单点距离 + 解析梯度 |
| `query_batch(pts)` | `(N, 2)` 批量距离 |
| `min_clearance(pts)` | 批量最小距离(用于快速诊断) |
| `update_obstacle(poly)` | **桩**接口,目前 `raise NotImplementedError`,留作动态规划扩展点 |

### 6.2 A* — 中心路径全局搜索

**文件**:`motion_planner/astar_2d.py`  
**入口**:`AStar2D(esdf, radius, cfg)`(astar_2d.py:32)

**设计要点**

- 网格独立于 ESDF:`astar_resolution`(默认 10 cm)比 `esdf_resolution` 粗一档,搜索更快。
- **可行性预计算**:`_build_grid` 一次性算出每个 cell 的 `ESDF ≥ R + safety` 二值 + 连续 clearance。
- 8 邻域 + 欧氏代价 + 启发(欧氏 × `astar_heuristic_weight`)。
- 边代价 = 步长 × (1 + `astar_obstacle_weight` / margin),通过 `margin = max(clearance - threshold, ε)` 实现"远离墙更便宜"。
- 搜索状态用 1D 拉平索引 + numpy 数组,避免 Python dict 开销;最大迭代 `astar_max_iter` 防止无限循环。
- 路径返回前做 **共线折叠**(`_collapse_collinear`),消除 A* 的"格子拐角"。

**失败模式**

- 起点 / 终点出界 → `ValueError`
- 起点 / 终点位于 `feasible == False` → `ValueError`
- 搜索耗尽 → `RuntimeError("A* failed to find a path")`

### 6.3 Shortcut — 折线去冗余

**文件**:`motion_planner/shortcut.py`  
**入口**:`ShortcutSmoother(esdf, footprint, cfg).smooth(path)`(shortcut.py:43)

**两级碰撞检查**

1. **粗筛**:`_line_clear` 沿连线采样(步长 ≈ ESDF 分辨率),对每个采样点查 ESDF,要求 ≥ `R + safety + coarse_margin`。即"包围圆内不撞"。
2. **精筛**:对每个采样点用 `footprint.world_points_batch` 展开真实底盘采样点,逐一查 ESDF,要求 ≥ `safety`。即"底盘完整不撞"。

**两种策略**

- 顺序快捷:从 i=1 扫到 n-2,若 `pts[i-1] → pts[i+1]` 直线通畅则删 `pts[i]`。
- 随机快捷:重复 `shortcut_iters` 次,每次随机选两个不相同的下标,若中间一段直线通畅则删除所有中间点。

端点永远不删。

### 6.4 Resample — 弧长均匀化

**文件**:`motion_planner/resample.py`  
**入口**:`resample_path(path, step)` / `Resampler(cfg).resample(path)`(resample.py:26, 71)

**为什么需要**:Shortcut 输出的折线段长差异很大(可能数百米长直线 + 数厘米短边)。直接喂给 ESDF smoother 会因"自由度集中在大段直线、短边无顶点"导致**要么弯不过来,要么把单点掰进障碍**。

**做法**:沿累计弧长线性插值,在 0..total 区间均匀采样 `n_int = round(total / step) + 1` 个点。**端点完全保留**。

### 6.5 ESDF Smoother — XY 梯度平滑(CasADi + IPOPT)

**文件**:`motion_planner/esdf_smoother.py`  
**入口**:`ESDFSmoother(esdf, radius, cfg).smooth(path)`

**代价(软项)**

$$
L(P) = w_\text{smooth}\sum_i\|p_{i+1}-2p_i+p_{i-1}\|^2 + w_\text{length}\sum_i\|p_{i+1}-p_i\|^2
$$

**硬约束(IPOPT 强制)**

- 碰撞:每个内部点 $\phi(p_i) \ge R + s$,通过 `casadi.interpolant('linear', [xs, ys], field)` 在静态 ESDF 网格上构造**解析可微**的查询函数。
- 曲率:$|\kappa_i| \le \kappa_\text{max}$,展开为 $cross_i^2 \le (\kappa_\text{max}\cdot ds_i^3)^2$,在 $cross=0$ 处梯度光滑。

**实现细节**

- 参数化:`opti.variable(2, N)`,端点用 `opti.subject_to(X[:, 0] == a0)` 锚定;尾部数值再次硬覆盖回原值,消除 IPOPT 的微小 slack。
- 起点修复:IPOPT 的 bilinear 插值在障碍内部梯度近零,无法逃出。先调用 `_push_out`,沿 `ESDF.query_grad`(带环搜索 fallback)将每个内部点拉至 `R + safety`,**喂给 IPOPT 的初值始终严格可行**。
- 求解失败容错:`try/except RuntimeError` → `opti.debug.value(X)` 取最后一次迭代;残余碰撞交由下游 Validator 检出,平滑器允许"软失败"。
- IPOPT 选项:`max_iter = smooth_iters`,`tol = 1e-4`,`acceptable_tol = 1e-3`,`linear_solver = mumps`,默认全部静默(`print_level=0`, `sb=yes`)。
- 解析梯度复用:`_esdf_fn` 在构造时建立一次,所有 `smooth()` 调用共享(ESDF 是静态的)。

### 6.6 RS / Dubins Planner — 运动学航段生成

**文件**:`motion_planner/rs_planner.py`  
**入口**:`RSPlanner(esdf, radius, cfg).plan(start, goal, ref_xy)`(rs_planner.py:306)

**历史说明**

模块名仍叫 `RSPlanner` 是为了与设计规约保持一致;**生产路径**目前使用 Dubins(纯前向),因为解析 Reeds-Shepp 公式在远距离输入下偶尔不能终止于终点。Dubins 的 6 种模式(`LSL/LSR/RSL/RSR/RLR/LRL`)已按 LaValle《Planning Algorithms》第 15 章自实现,可缩放到任意最小转弯半径。`rs_length` / `rs_interpolate` 的 API 形状与 OMPL `ReedsSheppStateSpace` 兼容,可作为未来切换点。

**实际生产路径**:`rs_interpolate`(rs_planner.py:218)走"线性 XY + 短弧 heading 插值"策略,确保 100% 命中终点。后续 SE(2) 平滑把直线段"掰"成圆弧过渡,既稳又可调。

**自适应锚点**

1. 累计 `ref_xy`(上一步的平滑折线)弧长,定 `s_s = L_lookahead`, `s_g = total - L_lookahead`。
2. 当 `total > 2 * L + ε` 时,落 `P_s` / `P_g` 两个前瞻锚,并在 `(s_s, s_g)` 内按 `rs_anchor_spacing` 均匀再插 `n_extra` 个内部锚。
3. 路径过短(常见于空地、起点终点很近)则回退到 `_anchors_line_fallback`,在 `start→goal` 直线上均匀布点。

**Heading 弛豫**

对每个内部锚 `k`(`1..n-2`),在 `[-rs_delta_heading, +rs_delta_heading]` 区间采 `rs_heading_samples` 个候选 heading,选使"前后两段 Dubins 长度之和"最小者。**首末锚 heading 固定为 `start.theta` / `goal.theta`**,首尾两个前瞻锚的 heading 初始化为 `ref_xy` 在该弧长处的切线,以保证首尾段有空间转向。

### 6.7 SE(2) Smoother — 流形上的轨迹优化(CasADi + IPOPT)

**文件**:`motion_planner/se2_smoother.py`  
**入口**:`SE2Smoother(esdf, radius, cfg).smooth(traj)`

**代价(软项)**

$$
L = w_\text{smooth}\sum\|p_{i+1}-2p_i+p_{i-1}\|^2 + w_\text{vel}\sum\|\Delta p\|^2
+ w_\text{acc}\sum\|\Delta^2 p\|^2 + w_\text{jerk}\sum\|\Delta^3 p_{xy}\|^2
$$

**硬约束(IPOPT 强制)**

- 碰撞:每个中间位姿 $\phi(x_i, y_i) \ge R + s$,与 XY smoother 共用 CasADi LUT 形式的 ESDF 解析查询。
- 车式曲率:对每段 $i$,$\bigl(\theta_{i+1}-\theta_i\bigr)^2 \le \kappa_\text{max}^2 \cdot ds_i^2$,等价于 $|\kappa| \le \kappa_\text{max}$,在 $\Delta\theta=0$ 处梯度光滑。

**Heading 展开**

进入 IPOPT 前先用 `_unwrap_theta` 把 $\theta$ 累积展开(连续两点差不超过 $\pi$),让 $\Delta\theta$ 在求解期间是一条直线,而非 `atan2(sin,cos)` 的分段函数。求解结束再 `_wrap_pi` 折回 $[-\pi, \pi)$。

**Stage 0: push-out**

`linear` 插值的 ESDF 在障碍内部梯度近零,IPOPT 自己出不来。先调用 `_push_out_xy` 把每个内部位姿沿 `ESDF.query_grad`(带环搜索 fallback)的方向推到 `R + safety`,给 IPOPT 一个**严格可行**的起点。

**容错与求解配置**

IPOPT 失败时 (`Infeasible_Problem_Detected` 等) 取 `opti.debug.value(X)` 的最后一次迭代,残余碰撞交给 Validator;`max_iter = se2_iters`,`tol = 1e-4`,`linear_solver = mumps`,完全静默。

### 6.8 Validator — 三级碰撞检查

**文件**:`motion_planner/validator.py`  
**入口**:`TrajValidator(esdf, footprint, cfg).validate_l1/l2/l3` 与统一 `validate(traj, level)`(validator.py:50, 81, 96, 125)

| 级别 | 输入 | 检查内容 | 失败时返回 |
| --- | --- | --- | --- |
| **L1** | `Path`(smoother 输出) | 中心点 ESDF ≥ `R + safety`;曲率 ≤ 1.5 / `rs_min_turn_r` | 首个碰撞下标 `i`、最大下标 `j`、最小间距、最大曲率 |
| **L2** | `SE2Trajectory`(RS 输出) | 中心点 ESDF ≥ `R + safety`(包围圆粒度) | 故障索引区间 + 最小间距 |
| **L3** | `SE2Trajectory`(SE(2) 平滑后) | 沿轨迹密集采 200 个位姿,展开 footprint,对每个采样点 ESDF ≥ `safety` | 故障索引区间 + 最小间距 |

`fault_range` 设计为本地修复入口:`HierarchicalPlanner` 在 L3 失败时**仅重跑 SE(2) smoother `validator_local_repair_iters` 次**,不再走 A* 起点。

### 6.9 Footprint — 底盘抽象

**文件**:`motion_planner/footprint.py`

```text
Footprint (ABC)
  ├── world_points(pose) -> (N, 2)   # 中心 → 世界采样点
  ├── world_points_batch(poses) -> (M·N, 2)  # 批量版本
  └── bounding_radius() -> float     # 用于圆粗筛

CircleFootprint(R)  # 16 环周点 + 中心,共 17 点
RectFootprint(W, H)  # 4 角 + 4 边中点 + 中心,共 9 点
```

`RectFootprint.bounding_radius = 0.5 · √(W² + H²)`(最小包围圆)。

### 6.10 Pipeline — 组合外观

**文件**:`motion_planner/pipeline.py`  
**入口**:`HierarchicalPlanner(map_spec, footprint, cfg).plan(start, goal) → PipelineResult`

构造时一次性装配所有子模块;`plan()` 严格按 8 步执行,并在 L3 失败时尝试有限次本地修复。

`PipelineResult` 暴露全部中间产物(`raw_astar / shortcut / resampled / smoothed_xy / rs_trajectory / final_trajectory`)以及 3 个 `ValidationResult`,便于 UI、调试、教学场景逐步可视化。

**模块级懒加载**:`motion_planner/__init__.py` 使用 `__getattr__` 按需导入,避免部分模块在导入时挂掉阻塞测试。

---

## 7. Web Demo 架构

**目录**:`demo/web/`(Flask + 原生 JS + Canvas)

### 7.1 后端

| 文件 | 职责 |
| --- | --- |
| `app.py` | Flask 应用,4 个 REST 端点 + 4 个内置预设地图 |
| `templates/index.html` | 主画布 + 6 个阶段小画布 + 工具栏 / 状态栏 / 日志 |
| `static/app.js` | 交互逻辑(画多边形、拖动起终点、调用 API、动画播放) |
| `static/style.css` | 暗色主题样式 |

### 7.2 REST API

| 方法 | 路径 | 用途 |
| --- | --- | --- |
| GET | `/` | HTML 页面 |
| GET | `/api/health` | 心跳,返回 `{ok, version}` |
| GET | `/api/presets` | 内置预设地图列表 |
| POST | `/api/plan` | 执行完整流水线,返回 6 个阶段 + ESDF 热力图 + L1/L2/L3 |
| POST | `/api/footprint` | 单点位姿下 footprint 世界采样点 |

`/api/plan` 响应结构:

```json
{
  "ok": true,
  "elapsed_ms": 23.4,
  "esdf": { "bounds": [...], "resolution": 0.05, "stride": [...], "shape": [...], "values": [...] },
  "stages": {
    "raw_astar":   [[x, y], ...],
    "shortcut":    [[x, y], ...],
    "resampled":   [[x, y], ...],
    "smoothed_xy": [[x, y], ...],
    "rs":          [[x, y, θ], ...],
    "final":       [[x, y, θ], ...]
  },
  "validation": { "l1": {...}, "l2": {...}, "l3": {...} }
}
```

ESDF 热力图对原 grid 做步长 striding(目标 ≤ 220 × 220,值四舍五入到 4 位小数),JSON 体保持 < 200 KB。

**安全护栏**:`_validate_*` 系列对入参做类型 / 范围 / NaN / 退化多边形检查,`ApiError` 统一回 400;`start==goal` 的平凡情况走 `_trivial_result` 快速返回,避免触发规划器异常。

### 7.3 前端

- **画图**:世界 ↔ 屏幕坐标转换(翻转 Y);主画布 + 6 个阶段小画布共享同一绘制管线。
- **交互**:画多边形(双击 / Enter 收尾)、拖动起终点、橡皮擦工具、4 个内置预设。
- **可视化**:ESDF 用离屏 `ImageData` 按 HSL 染色;6 个阶段用不同颜色叠加,checkbox 切换。
- **动画**:以弧长 5 秒播放最终轨迹,`requestAnimationFrame` 驱动,`Stop` 可中断。

### 7.4 本地运行

```bash
pip install -e .[web]      # 或 pip install flask
python run_web.py          # 等价于 python demo/web/app.py
# 访问 http://127.0.0.1:5000
```

---

## 8. API 参考

### 8.1 规划核心

```python
from motion_planner import (
    HierarchicalPlanner, MapSpec, Pose2D,
    CircleFootprint, RectFootprint, default_config,
)
from shapely.geometry import Polygon

walls = [Polygon([(-1, -1.5), (-0.6, -1.5), (-0.6, -0.3), (-1, -0.3)])]
spec = MapSpec(polygons=walls, bounds=(-3, -3, 3, 3))
fp = RectFootprint(0.4, 0.2)
planner = HierarchicalPlanner(spec, fp, default_config())

result = planner.plan(Pose2D(-2.5, 0, 0), Pose2D(2.5, 0, 0))
print(result.l3.ok, result.final_trajectory.poses.shape)
```

### 8.2 单步使用

```python
from motion_planner import ESDF, AStar2D, ShortcutSmoother, Resampler, ESDFSmoother, \
    RSPlanner, SE2Smoother, TrajValidator, RectFootprint

esdf = ESDF(spec, cfg)
astar = AStar2D(esdf, fp.bounding_radius(), cfg)
raw = astar.plan((-2, 0), (2, 0))
sc = ShortcutSmoother(esdf, fp, cfg).smooth(raw)
ds = Resampler(cfg).resample(sc)
sm = ESDFSmoother(esdf, fp.bounding_radius(), cfg).smooth(ds)
rs = RSPlanner(esdf, fp.bounding_radius(), cfg).plan(start, goal, sm)
final = SE2Smoother(esdf, fp.bounding_radius(), cfg).smooth(rs)
v = TrajValidator(esdf, fp, cfg).validate(final, level=3)
```

### 8.3 启动入口汇总

| 入口 | 命令 | 输出 |
| --- | --- | --- |
| 单元 / 集成测试 | `pytest` | 通过 / 失败 |
| 4-panel matplotlib 演示 | `python demo/pipeline_demo.py` | `demo/result.png` |
| Web 演示 | `python run_web.py` | `http://127.0.0.1:5000` |
| 命令行(包安装) | `python -m motion_planner` | (未提供,直接调 `HierarchicalPlanner`) |

---

## 9. 测试策略

`tests/` 目录下每个模块都有独立测试文件,共 **11 个文件,约 1050 行**:

| 测试文件 | 关注点 |
| --- | --- |
| `test_esdf.py` | 距离符号、边界、双线性插值、批量查询、min_clearance |
| `test_astar.py` | 可行 / 不可行起点、找不到路径、共线折叠 |
| `test_shortcut.py` | 顺序 + 随机两种策略、碰撞保护 |
| `test_resample.py` | 端点保留、间距均匀、零长边界 |
| `test_esdf_smoother.py` | 收敛性、锚点保持、障碍斥力 |
| `test_rs.py` | 6 种 Dubins 模式、heading 弛豫、OMPL 兼容 API |
| `test_se2_smoother.py` | push-out 行为、推力 / 速度 / jerk 权重单调性 |
| `test_footprint.py` | 圆 / 矩形采样点数量与位置、包围圆 |
| `test_validator.py` | L1 / L2 / L3 在人造失败用例上的故障区间 |
| `test_pipeline.py` | 端到端空地、走廊、矩形 footprint 场景 |
| `test_web_api.py` | `/api/health` `/api/presets` `/api/plan` `/api/footprint` 协议、错误码、平凡情况 |

**运行**:`pytest`(已在 `pyproject.toml` 配好 testpaths / addopts `-q`)。

**建议**:在 `HierarchicalPlanner` 上做改动后,先跑 `test_pipeline.py` 与 `test_web_api.py`,再补专项测试。

---

## 10. 扩展指南

### 10.1 新增 Footprint 类型

1. 继承 `Footprint`,实现 `world_points(pose) → (N, 2)` 与 `bounding_radius() → float`。
2. 在 `motion_planner/__init__.py` 的 `__getattr__` 注册。
3. Web API `_validate_footprint` 增加 shape 分支。
4. 编写 `tests/test_footprint.py` 测试。

### 10.2 替换 ESDF 后端

`ESDF` 的公共面只有 `query` / `query_grad` / `query_batch` / `min_clearance` / `update_obstacle`。可以把内部存储换成 voxel hash、八叉树等,只要保持这 5 个方法的语义。

### 10.3 替换 / 升级 SE(2) 求解器

`SE2Smoother` 与 `ESDFSmoother` 接口简单(`smooth(...) → 同类型`),目前默认实现是 CasADi `Opti` + IPOPT(MUMPS 线性求解器)。可直接替换为 ACADO、Ceres、`scipy` L-BFGS-B、或第三方 `toppra` 等;只要保持端点锚定 + ESDF 硬约束 + 曲率硬约束的语义即可。注意 heading 差分必须用 `atan2(sin, cos)` 包裹或事先 `np.unwrap`,否则会在 2π 边界跳变。

### 10.4 增加新的运动学约束

在 `PlannerConfig` 加字段(如 `se2_steering_rate_max`),在 `SE2Smoother.smooth` 里追加 `opti.subject_to(...)` 即可(CasADi 自动求导);在 `RSPlanner._relax_headings` 中扩展搜索维度。

### 10.5 接入 ROS

新建 `motion_planner_ros/` 适配包,做以下映射:

- `nav_msgs/OccupancyGrid` → `MapSpec`(需注意 ESDF 构建前先做占据膨胀)
- `geometry_msgs/PoseStamped` ↔ `Pose2D`
- 输出轨迹 → `nav_msgs/Path`(XY 投影)+ `geometry_msgs/PoseArray`

---

## 11. 已知的局限与未来工作

| 局限 | 影响 | 可能的解法 |
| --- | --- | --- |
| 动态障碍未实现 | `ESDF.update_obstacle` 仅抛 `NotImplementedError` | 增量 EDT / 滚动窗口 |
| RS 真解析未启用 | 当前 RS planner 走 Dubins 直线插值,远距离易"不优雅" | 数值采样 RS / 调用 OMPL C++ 绑定 |
| 矩形 footprint 仅 9 个采样点 | 极端窄长矩形可能在角落漏检 | 自适应采样密度 / Minkowski 和 |
| L3 失败时仅做有限次 SE(2) 再平滑 | 复杂窄通道可能仍失败 | 引入 Corridor / 收缩迭代 / 重布锚点 |
| Ackermann 严格约束未建模 | 真实阿克曼车辆在原地掉头时不可行 | 切换为 State-lattice 规划 + 车辆模型 |
| ESDF 内存与地图大小成正比 | 10 m × 10 m @ 5 cm → 200×200 ≈ 160 KB,可接受;100 m × 100 m 即 100 MB | 分块 / 哈希存储 / 动态加载 |

---

## 附录 A:关键源码导航

| 主题 | 文件:行 |
| --- | --- |
| 流水线入口 | `motion_planner/pipeline.py:71` |
| ESDF 构造 | `motion_planner/esdf.py:29` |
| ESDF 梯度(含障碍内回退) | `motion_planner/esdf.py:92` |
| A* 主循环 | `motion_planner/astar_2d.py:108` |
| Shortcut 双层碰撞 | `motion_planner/shortcut.py:78` |
| ESDF smoother(CasADi Opti) | `motion_planner/esdf_smoother.py:68` |
| ESDF smoother push-out | `motion_planner/esdf_smoother.py:148` |
| RS 自适应锚点 | `motion_planner/rs_planner.py:348` |
| RS heading 弛豫 | `motion_planner/rs_planner.py:436` |
| SE(2) smoother(CasADi Opti) | `motion_planner/se2_smoother.py:88` |
| SE(2) push-out | `motion_planner/se2_smoother.py:166` |
| L1 / L2 / L3 验证 | `motion_planner/validator.py:50 / 81 / 96` |
| Web REST 端点 | `demo/web/app.py:292 / 297 / 302 / 307 / 360` |
| Web 前端交互 | `demo/web/static/app.js:546 / 733 / 777` |
| 配置项 | `motion_planner/config.py:12` |

## 附录 B:版本

`pyproject.toml` 当前版本:`0.1.0`
