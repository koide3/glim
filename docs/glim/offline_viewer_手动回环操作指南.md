# GLIM Offline Viewer 手动回环操作指南

## 1. 概述

GLIM 的 Offline Viewer 提供了一个交互式界面，用于在建图完成后**离线修正地图**。主要功能包括：

- 手动添加/删除回环约束（Loop Closure）
- 创建平面 Bundle Adjustment 约束
- 多 session 地图合并
- 导出点云（PLY 格式）

当自动回环检测失败或里程计出现跑飞时，可以通过手动指定正确的回环关系，重新优化位姿图（PGO），修复地图。

---

## 2. 前置条件

- 建图时需要开启 dump 功能，建图结束后会在指定目录生成 dump 数据
- dump 目录中包含 submap、位姿图、轨迹等数据

---

## 3. 启动 Offline Viewer

```bash
# 方式一：通过 launch 文件启动（推荐）
ros2 launch glim_ros offline_viewer.launch.py map_path:=<dump目录路径>

# 方式二：直接运行
ros2 run glim_ros offline_viewer <dump目录路径>
```

启动后会弹出一个带有 3D 可视化界面的窗口。

### 加载地图

如果启动时没有指定 `map_path`，可以通过菜单加载：

- `File` → `Open Map` → 选择 dump 目录

---

## 4. 手动创建回环约束（核心操作）

这是最常用的功能，用于手动指定两个 submap 之间的位姿约束。

### 4.1 操作步骤

1. **选择 Target（目标 submap）**
   - 在 3D 视图中找到对应的 submap 球体（sphere）
   - **右键点击**该球体 → 选择 `Loop begin`

2. **选择 Source（源 submap）**
   - 找到另一个需要建立回环关系的 submap 球体
   - **右键点击**该球体 → 选择 `Loop end`

3. **弹出手动回环对话框（Manual Loop Close Modal）**
   - 对话框包含一个 512×512 的 3D 预览窗口
   - **红色点云** = Target（目标）
   - **绿色点云** = Source（源）
   - 可以通过 **Gizmo 控件**手动拖动绿色点云进行粗对齐

4. **全局配准（Global Registration）** — 自动粗对齐
   - 选择配准方式：`RANSAC` 或 `GNC`（Graduated Non-Convexity）
   - 设置参数（见下方参数说明）
   - 点击 `Run global registration`
   - 等待配准完成，检查红绿点云是否大致对齐

5. **精配准（Fine Registration）** — ICP 精细对齐
   - 设置 `max_corr_dist`（最大对应点距离）
   - 点击 `Run fine registration`
   - 检查对齐效果

6. **创建因子**
   - 对齐满意后，点击 `Create Factor`
   - 系统会创建一个 `BetweenFactor<Pose3>` 加入位姿图
   - 位姿图会自动重新优化

7. **保存地图**
   - `File` → `Save`

### 4.2 参数说明

#### 全局配准参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fpfh_radius` | 5.0 | FPFH 特征提取的邻域搜索半径。室内约 2.5m，室外约 5.0m |
| `Global registration type` | RANSAC | 全局配准算法：RANSAC 或 GNC |
| `4dof` | 开启 | 使用 4 自由度（XYZ + 绕 Z 轴旋转）代替完整 6 自由度，适合地面机器人等平面运动场景 |

**RANSAC 模式参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_iterations` | 5000 | RANSAC 最大迭代次数 |
| `early_stop_rate` | 0.9 | 提前终止阈值 |
| `inlier_voxel_resolution` | 1.0 | 内点检查的体素分辨率 |

**GNC 模式参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_samples` | 10000 | GNC 最大特征采样数 |

#### 精配准参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_corr_dist` | 1.0 | ICP/GICP 最大对应点距离。室内约 1.0m，室外约 3.0m |
| `inf_scale` | 1.0 | 回环因子的信息矩阵缩放系数。值越大，优化时该约束权重越高 |

### 4.3 推荐参数预设

| 场景 | `min_distance` | `fpfh_radius` | `max_corr_dist` |
|------|---------------|---------------|-----------------|
| 室内 | 0.25 | 2.5 | 1.0 |
| 室外 | 0.5 | 5.0 | 3.0 |

---

## 5. 平面 Bundle Adjustment 约束

用于利用平面特征（如墙壁、地面）进一步约束位姿图，提高地图一致性。

### 操作步骤

1. 在 3D 视图中**右键点击**一个平坦表面上的点
2. 选择 `Bundle Adjustment (Plane)`
3. 调整球体大小，使其覆盖平面上足够多的点
4. 点击 `Create Factor`

---

## 6. 多 Session 地图合并

当需要将多次建图的结果合并为一张完整地图时使用。

### 操作步骤

1. `File` → `Open New Map` → 选择第一个 dump 目录 → 弹出对话框选 `Yes` 启用优化
2. `File` → `Open Additional Map` → 选择第二个 dump 目录
3. 点击 `Merge sessions`
4. 选择预设参数（`Indoor` 或 `Outdoor`），点击 `OK`
5. 对齐点云：
   - **自动对齐**：点击 `Run global registration`，检查粗对齐效果
   - **手动对齐**：通过 Gizmo 拖动绿色点云（source）与红色点云（target）大致对齐
6. 点击 `Run fine registration` 进行 ICP 精配准
7. 点击 `Create factor` 完成合并
8. （可选）点击 `Find overlapping submaps` 创建重叠区域的匹配代价因子，提高全局一致性
9. 点击 `Optimize` 多次运行优化（或勾选自动连续优化）

### 注意事项

- 如果加载时出现 `[global] [warning] X34 -> E69 is missing` 等警告，说明地图数据可能损坏
- 损坏的地图需要先修复：加载 → 点击 `Recover graph` → 保存 → 关闭，逐个修复后再合并

---

## 7. 导出点云

- `File` → `Export Points`
- 导出格式为 PLY

---

## 8. 常见问题

**Q: 回环选错了怎么办？**
A: 创建 Factor 之前可以点 `Cancel` 取消。如果已经创建，可以重新加载原始 dump 数据重新操作。

**Q: 全局配准失败怎么办？**
A: 先用 Gizmo 手动粗对齐，再直接跑 `Run fine registration` 跳过全局配准。

**Q: 4DOF 和 6DOF 怎么选？**
A: 如果机器人在平面上运动（如地面机器人），选 4DOF 更稳定；如果是手持或无人机等有较大俯仰/翻滚的场景，用 6DOF。
