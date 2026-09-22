# AUV-Path-Planning-PSO

> 水下航行器（AUV）三维路径规划 —— 基于改进粒子群优化算法
>
> 3D Path Planning for Autonomous Underwater Vehicles based on an Improved Particle Swarm Optimization

MATLAB 实现。在三维海洋环境中为 AUV 规划一条**避障、避地形、能耗最低**的平滑路径，
环境同时包含**静态障碍物**与**三维涡流场（洋流）**扰动。

---

## 1. 项目目的

本项目研究**分层洋流扰动下 AUV 的三维路径规划**问题，目标是：

- 在含海底地形起伏、多类型静态障碍物的三维海域中，为 AUV 规划一条**无碰撞**的可行路径；
- 在路径长度之外，进一步将**航行能耗**纳入优化目标，刻画顺流/逆流对能耗的影响；
- 针对标准 PSO 易早熟收敛的问题，引入**余弦自适应惯性权重**、**正弦异步学习因子**与
  **距离进化因子驱动的跳出机制**，提升全局搜索能力与收敛稳定性。

## 2. 环境要求

| 项目 | 要求 |
|---|---|
| MATLAB | R2020a 及以上（源码为 UTF-8 with BOM 编码；更早版本请转存为 GBK） |
| 工具箱 | 无额外依赖，仅使用 MATLAB 基础绘图与数值函数 |
| 操作系统 | Windows / Linux / macOS 均可 |

## 3. 安装

本仓库为纯 MATLAB 源码，**无需编译、无需安装任何第三方依赖**，克隆即用：

```bash
git clone https://github.com/lyruaa/AUV-Path-Planning-PSO-.git
cd AUV-Path-Planning-PSO-
```

克隆完成后，确认仓库根目录同时存在：

- 输入数据：`XYZmesh.mat`（海底地形高度网格）
- 全部源码：`main.m`、`createEnvironment.m`、`ParseSolution.m` 等 `.m` 文件

## 4. 使用

### 4.1 快速开始（改进算法）

在 MATLAB 命令窗口执行：

```matlab
% 1) 切换到仓库根目录（请替换为你的实际克隆路径）
cd('D:/code/AUV-Path-Planning-PSO-');

% 2) 确认输入数据已就位
assert(exist('XYZmesh.mat','file')==2, '缺少输入数据 XYZmesh.mat');

% 3) 运行改进算法（DENPSO）
main
```

运行结束后，命令窗口会打印收敛摘要，并在**当前目录自动生成**结果数据文件。

### 4.2 完整使用路径（从零到出图，可逐步核验）

下面的每一步都可在仓库内直接验证，命令与文件名严格对应。

| 步骤 | 在 MATLAB 中执行 | 预期结果 |
|---|---|---|
| 1 | `cd('D:/code/AUV-Path-Planning-PSO-')` | 当前文件夹切换到仓库根目录 |
| 2 | `dir('XYZmesh.mat')` | 列出 1 个文件（36360 字节的地形网格） |
| 3 | `main` | 弹出三维环境窗口 + 迭代过程路径动画 |
| 4 | 运行完毕后查看目录 `dir('*.mat')` | 新增 `best_cost1.mat`、`energy1.mat`、`violation1.mat`、`Symbol.mat`、`path1.mat`、`AUV_DATA1~3.mat` |
| 5 | `load('best_cost1.mat'); plot(BestCost)` | 绘制改进算法的收敛曲线 |
| 6 | `main_standard_pso` | 运行标准 PSO 对照实验，生成 `best_cost11.mat`、`energy11.mat`、`violation11.mat`、`path11.mat`、`AUV_DATA11/22/33.mat` |
| 7 | `compare_path` | 绘制改进算法 vs 标准 PSO 的三维路径对比图 |
| 8 | `comp_cost` | 绘制两者收敛曲线对比图 |
| 9 | `comp_energy` | 绘制两者能耗曲线对比图 |
| 10 | `comp_vio` | 绘制两者约束违反次数对比图 |
| 11 | `comp_symbol` | 绘制跳出机制触发次数曲线 |
| 12 | `plot_AUV_MO` | 绘制 AUV 轨迹与航向角、俯仰角变化曲线 |

> **依赖顺序说明**：步骤 6 必须在步骤 7~12 之前完成 —— 对比脚本需要读取
> **两组**数据（`*1.mat` 为改进算法、`*11.mat` 为标准 PSO）。若未先运行
> `main_standard_pso`，对比脚本会因缺少对照组数据而报错。

### 4.3 各脚本产出的数据文件对照

| 运行脚本 | 生成的数据文件 | 被哪些脚本读取 |
|---|---|---|
| `main` | `best_cost1.mat` | `comp_cost` |
| `main` | `energy1.mat` | `comp_energy` |
| `main` | `violation1.mat` | `comp_vio` |
| `main` | `Symbol.mat` | `comp_symbol` |
| `main` | `path1.mat` | `compare_path` |
| `main` | `AUV_DATA1/2/3.mat` | `plot_AUV_MO` |
| `main` | `sea_x/y/z.mat` | 流场数据，供上述脚本复用 |
| `main_standard_pso` | `best_cost11.mat` | `comp_cost` |
| `main_standard_pso` | `energy11.mat` | `comp_energy` |
| `main_standard_pso` | `violation11.mat` | `comp_vio` |
| `main_standard_pso` | `path11.mat` | `compare_path` |
| `main_standard_pso` | `AUV_DATA11/22/33.mat` | `plot_AUV_MO` |

> 上述结果数据均为算法运行产物，可由脚本随时重新生成，已列入 `.gitignore`
> **不纳入版本控制**。克隆仓库后需先运行 `main` 与 `main_standard_pso`，
> 再运行对比与可视化脚本。

## 5. 配置

主要参数集中在 `main.m`（与 `main_standard_pso.m` 保持一致），可直接修改：

| 参数 | 位置 | 取值 | 含义 |
|---|---|---|---|
| `x_limit / y_limit / z_limit` | `createEnvironment.m` | `[0,200] / [0,200] / [-5,10]` | 海域范围（m） |
| `start_point` / `goal_point` | `createEnvironment.m` | `[5,5,5] / [160,180,0]` | 起止点坐标 |
| `RADIUS` / `S` | `main.m` | `10 / 8` | 涡旋强度 / 涡核尺度 |
| `R_O` | `main.m` | 12 个涡心 | 洋流涡旋中心位置 |
| `MaxIt` / `nPop` | `main.m` | `200 / 40` | 最大迭代次数 / 种群规模 |
| `nVar` | `main.m` | `10` | 路径控制点数目 |
| `w_max / w_min` | `main.m` | `0.9 / 0.4` | 惯性权重上下限（余弦自适应） |
| `c_a, c_b, c_af, c_beta` | `main.m` | `1 / 1.5 / 1 / 1.5` | 学习因子参数（正弦异步） |
| `beta1 / beta2 / beta3` | `CostFunction.m` | `2 / 2 / 0.8` | 能耗 / 洋流 / 障碍物权重 |
| `beta` | `CostFunction.m` | `300` | 碰撞罚因子 |
| `v_max` | `CreateModel1.m` | `6` | AUV 最大速度（m/s） |
| `c_d` | `cost_energy_cal.m` | `3` | AUV 阻尼系数 |

## 6. 算法设计说明

在标准 PSO 基础上引入三项改进（对照实验 `main_standard_pso.m` 不含这些改进）：

**① 余弦自适应惯性权重**

```
w = (w_max - w_min)/2 · cos(π · t/T) + (w_max + w_min)/2
```

前期 `w` 大、全局搜索能力强；后期 `w` 小、利于局部收敛。

**② 正弦异步学习因子**

```
c1 = c_a · sin(π/2 · (T/2 - t)/(T/2)) + c_b       % 认知项，递减
c2 = c_af · sin(π/2 · (t - T/2)/(T/2)) + c_beta   % 社会项，递增
```

**③ 距离进化因子与跳出机制**

计算各粒子到全局最优粒子的平均距离并归一化为 `E_f ∈ [0,1]`：
`E_f < 0.25` 时种群聚集，按标准 PSO 精细搜索；`E_f ≥ 0.25` 时触发 `jump_to()`
对发散粒子施加扰动并保留精英，抑制早熟收敛。

**适应度函数**采用罚函数形式，同时优化路径长度与能耗：

```
J = w_obs · L · (1 + β · N_violation) + w_E · (E_path - E_min)² / E_min²
```

## 7. 文件说明

```
├── main.m                    # 主入口：改进 PSO（DENPSO）完整流程
├── main_standard_pso.m       # 对照实验：标准 PSO（LPSO）
├── createEnvironment.m       # 三维环境搭建（地形、起止点、多类型障碍物）
├── CreateModel1.m            # 构造模型结构体（边界、起止点、洋流极值）
├── ParseSolution.m           # 控制点 → 三次样条路径，碰撞检测
├── CostFunction.m            # 适应度函数（路径长度 + 能耗罚函数）
├── cost_energy_cal.m         # 分层洋流下的能耗积分计算
├── cal_distance.m            # 粒子间距离（距离进化因子输入）
├── cal_speed.m               # AUV 速度分量分解
├── jump_to.m                 # 跳出机制（抑制早熟收敛）
├── path_smooth.m             # 路径平滑（改进算法）
├── path_smooth1.m            # 路径平滑（对照算法）
├── compare_path.m            # 路径对比图
├── comp_cost.m               # 收敛曲线对比
├── comp_energy.m             # 能耗曲线对比
├── comp_symbol.m             # 跳出次数曲线
├── comp_vio.m                # 约束违反次数对比
├── plot_AUV_MO.m             # AUV 轨迹与姿态角可视化
├── ceshi.m                   # 障碍物几何形状调试脚本
├── XYZmesh.mat               # 海底地形高度网格（原始输入数据）
├── LICENSE                   # MIT 许可证
└── .gitignore                # 忽略规则（含算法运行产物）
```

## 8. 许可

本项目采用 **MIT License**，详见 [LICENSE](LICENSE)。
可自由用于学习、研究与商业用途，保留版权声明即可。
