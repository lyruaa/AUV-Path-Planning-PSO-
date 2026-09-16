# AUV-Path-Planning-PSO

> 水下航行器（AUV）三维路径规划 —— 基于改进粒子群优化算法

MATLAB 实现。在三维海洋环境中为 AUV 规划一条**避障、避地形、能耗最低**的平滑路径，
环境同时包含**静态障碍物**与**三维涡流场（洋流）**扰动。

---

## 1. 目的

本项目研究**分层洋流扰动下 AUV 的三维路径规划**问题，目标是：

- 在含海底地形起伏、多类型静态障碍物的三维海域中，为 AUV 规划一条**无碰撞**的可行路径；
- 在路径长度之外，进一步将**航行能耗**纳入优化目标，刻画顺流/逆流对能耗的影响。

## 2. 环境要求

| 项目 | 要求 |
|---|---|
| MATLAB | R2020a 及以上（源码为 UTF-8 with BOM 编码） |
| 工具箱 | 无额外依赖，仅使用 MATLAB 基础绘图与数值函数 |

## 3. 安装

```bash
git clone https://github.com/lyruaa/AUV-Path-Planning-PSO-.git
cd AUV-Path-Planning-PSO-
```

无需编译与额外依赖安装。确认 `XYZmesh.mat` 与全部 `.m` 文件位于同一目录。

## 4. 使用

在 MATLAB 命令窗口中将本仓库根目录设为当前文件夹，然后执行主程序：

```matlab
cd('path/to/AUV-Path-Planning-PSO-');
main
```

程序运行后将输出三维环境与流场实时渲染，以及迭代收敛曲线。

## 5. 许可

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE)。
