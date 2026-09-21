# AUV-Path-Planning-PSO

> 水下航行器（AUV）三维路径规划 —— 基于改进粒子群优化算法

MATLAB 实现。在三维海洋环境中为 AUV 规划一条**避障、避地形、能耗最低**的平滑路径，
环境同时包含**静态障碍物**与**三维涡流场（洋流）**扰动。

---

## 1. 目的

本项目研究**分层洋流扰动下 AUV 的三维路径规划**问题，目标是：

- 在含海底地形起伏、多类型静态障碍物的三维海域中，为 AUV 规划一条**无碰撞**的可行路径；
- 在路径长度之外，进一步将**航行能耗**纳入优化目标，刻画顺流/逆流对能耗的影响；
- 针对标准 PSO 易早熟收敛的问题，引入改进策略以提升全局搜索能力与收敛稳定性。

## 2. 环境要求

| 项目 | 要求 |
|---|---|
| MATLAB | R2020a 及以上（源码为 UTF-8 with BOM 编码） |
| 工具箱 | 无额外依赖，仅使用 MATLAB 基础绘图与数值函数 |
| 操作系统 | Windows / Linux / macOS 均可 |

## 3. 安装

```bash
git clone https://github.com/lyruaa/AUV-Path-Planning-PSO-.git
cd AUV-Path-Planning-PSO-
```

无需编译与额外依赖安装。确认 `XYZmesh.mat` 与全部 `.m` 文件位于同一目录。

## 4. 使用

在 MATLAB 命令窗口中将本仓库根目录设为当前文件夹，然后执行主程序：

```matlab
% 方式一：cd 到仓库根目录后直接运行
cd('path/to/AUV-Path-Planning-PSO-');
main

% 方式二：先切换到目录，再运行（等效）
main
```

**可核验的完整使用路径（从零到出图）**

| 步骤 | 操作 | 预期产物 |
|---|---|---|
| 1 | `cd` 到仓库根目录 | 当前文件夹为该仓库 |
| 2 | 确认输入数据存在：`dir('XYZmesh.mat')` | 返回 1 个文件信息 |
| 3 | 运行 `main` | 三维环境渲染 + 迭代动画 |
| 4 | 运行结束后查看工作区变量 | `GlobalBest`、`BestCost`、`energy_opt` |
| 5 | 运行结束后查看目录 | 自动生成 `sea_x/y/z.mat` 等结果数据 |
| 6 | 依次运行 `compare_path` / `comp_cost` / `comp_energy` | 路径、收敛、能耗对比图 |

程序运行后将输出：

- 三维环境与流场实时渲染（含迭代过程路径动画）
- 收敛曲线、能耗曲线、碰撞次数统计
- 工作区变量 `BestCost`（各代最优代价）、`energy_opt`（各代最优能耗）、`Violation`（碰撞统计）
- 结果数据文件 `sea_x.mat` / `sea_y.mat` / `sea_z.mat`（流场数据，供对比脚本复用）

> 说明：算法运行产生的结果数据（`path*.mat`、`best_cost*.mat`、`energy*.mat`、`Violation*.mat`、
> `sea_*.mat` 等）已列入 `.gitignore`，不纳入版本控制，运行后于本地自动生成。

## 5. 配置

主要参数位于 `main.m` 与 `CreateModel1.m`，可直接修改：

| 参数 | 位置 | 取值 | 含义 |
|---|---|---|---|
| `x_limit / y_limit / z_limit` | `main.m` | `[0,200] / [0,200] / [-5,10]` | 海域范围（m） |
| `start_point` / `goal_point` | `main.m` | `[5,5,5] / [160,180,0]` | 起止点坐标 |
| `R_O` | `main.m` | 12 个涡心 | 洋流涡旋中心 |
| `RADIUS` / `S` | `main.m` | `10 / 8` | 涡旋强度与涡核尺度 |
| `MaxIt` / `nPop` | `main.m` | `200 / 40` | 最大迭代次数 / 种群规模 |
| `nVar` | `main.m` | `10` | 控制点数目 |
| `w_max / w_min` | `main.m` | `0.9 / 0.4` | 惯性权重上下限 |
| `v_max` | `CreateModel1.m` | `6` | AUV 最大速度 |

## 6. 许可

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE)。

## 说明

- 源码包含中文注释；若使用 R2020a 之前的版本导致中文乱码，可将文件转存为 GBK 编码。
- 若需复现论文中的对照实验，请保持随机数种子一致，或多次运行取统计均值。
