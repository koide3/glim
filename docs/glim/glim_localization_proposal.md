# 基于 GLIM 的鲁棒 LiDAR-IMU 定位系统技术方案

## 文档信息

| 项目 | 内容 |
|------|------|
| 项目名称 | GLIM-Localization |
| 版本 | v1.0 |
| 日期 | 2024-12 |
| 作者 | Steve |

---

## 1. 背景与目标

### 1.1 问题背景

在已构建地图中进行机器人定位是一个常见需求。现有开源方案主要分为两类：

**方案A: 纯地图匹配（如 FAST_LIO_LOCALIZATION）**
- 将先验地图加载到 ikd-tree，每帧 LiDAR 直接匹配到先验地图
- 假设地图是完美真值
- **缺点**：地图质量不高或环境变化时，匹配失败导致 EKF 发散

**方案B: 松耦合切换（如 Lidar_IMU_Localization）**
- 地图匹配和 LIO 分开运行，自动切换
- **缺点**：切换时刻不连续，无法充分利用两种约束

**理想方案: 滑动窗口因子图优化（如 GLIL）**
- 同时优化 Scan-to-Scan、Scan-to-Map、IMU 因子
- 地图缺失时自动退化为 LIO，回到地图范围时平滑校正
- **问题**：GLIL 闭源商业化，需付费授权

### 1.2 项目目标

基于开源的 **GLIM** 框架，开发一个类似 GLIL 架构的鲁棒定位系统：

1. **滑动窗口因子图优化**：同时融合 Scan-to-Scan、Scan-to-Prior-Map、IMU 约束
2. **自动退化处理**：地图缺失/边界时平滑退化为纯 LIO
3. **退化感知**：检测并处理几何退化场景
4. **GPU 加速**：利用 gtsam_points 的 CUDA 实现实时运行

---

## 2. 技术选型

### 2.1 为什么选择 GLIM

| 特性 | GLIM | FAST-LIO | LIO-SAM |
|------|------|----------|---------|
| 状态估计 | 滑动窗口 FGO | ESKF | 因子图 |
| 配准方式 | Multi-scan GICP | ikd-tree | LOAM特征 |
| GPU 加速 | ✅ (gtsam_points) | ❌ | ❌ |
| 扩展机制 | ✅ 全局回调插槽 | ❌ | △ |
| 代码质量 | 高，模块化 | 高 | 中 |
| 维护状态 | 活跃 | 活跃 | 一般 |

**GLIM 的核心优势**：
- 已实现滑动窗口因子图优化框架
- 提供全局回调机制，可插入自定义因子
- GPU 加速的 VGICP 因子开箱即用
- 与 GLIL 同一作者，架构相似

### 2.2 核心依赖

```
GLIM (v1.0+)
├── gtsam_points (v1.2.0)
│   ├── IntegratedVGICPFactor / IntegratedVGICPFactorGPU
│   ├── IncrementalCovarianceVoxelMap
│   ├── FPFH + RANSAC 全局配准
│   └── KdTree (并行构建)
├── GTSAM (4.3a0)
│   ├── iSAM2
│   ├── Fixed-lag Smoother
│   └── IMU Preintegration
└── CUDA (12.2+)
```

---

## 3. 系统架构

### 3.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        GLIM-Localization System                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌────────────────┐                                                      │
│  │  Prior Map     │  ← PCD / VoxelMap / GLIM SimplMap                   │
│  │  (Offline)     │                                                      │
│  └───────┬────────┘                                                      │
│          │                                                               │
│          ▼                                                               │
│  ┌────────────────┐     ┌──────────────────┐                            │
│  │ Map Loader &   │────▶│ Gaussian Voxel   │                            │
│  │ Preprocessor   │     │ Map (GPU)        │                            │
│  └────────────────┘     └────────┬─────────┘                            │
│                                  │                                       │
│  ┌───────────────────────────────┼───────────────────────────────────┐  │
│  │          Sliding Window Factor Graph Optimization                  │  │
│  │                               │                                    │  │
│  │   LiDAR ──┬──▶ [Scan-to-Scan Factor] ──────────────┐              │  │
│  │           │                                         │              │  │
│  │           └──▶ [Scan-to-Prior-Map Factor] ◀────────┼── Prior Map  │  │
│  │                        │                            │              │  │
│  │                        ▼ (weight by overlap)        │              │  │
│  │   IMU ────▶ [IMU Preintegration Factor]             │              │  │
│  │                        │                            │              │  │
│  │                        ▼                            │              │  │
│  │   ┌─────────────────────────────────────────────────┘              │  │
│  │   │                                                                │  │
│  │   │    X(k-n) ═══ X(k-n+1) ═══ ... ═══ X(k-1) ═══ X(k)            │  │
│  │   │      ║          ║                    ║         ║               │  │
│  │   │    V,B(k-n)   V,B(k-n+1)          V,B(k-1)   V,B(k)            │  │
│  │   │                                                                │  │
│  │   │    ══════  IMU Factor                                          │  │
│  │   │    ──────  Scan-to-Scan Factor                                 │  │
│  │   │    ┄┄┄┄┄┄  Scan-to-Prior-Map Factor (adaptive weight)         │  │
│  │   │                                                                │  │
│  │   └────────────────────────────────────────────────────────────────┤  │
│  │                               │                                    │  │
│  │                    ┌──────────▼──────────┐                         │  │
│  │                    │   iSAM2 Optimizer   │                         │  │
│  │                    │   (Fixed-lag)       │                         │  │
│  │                    └──────────┬──────────┘                         │  │
│  │                               │                                    │  │
│  └───────────────────────────────┼────────────────────────────────────┘  │
│                                  │                                       │
│                                  ▼                                       │
│                    ┌─────────────────────────┐                          │
│                    │   Pose Output           │                          │
│                    │   T_world_imu @ 100Hz   │                          │
│                    └─────────────────────────┘                          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 核心模块划分

```
glim_localization/
├── include/glim_localization/
│   ├── prior_map/
│   │   ├── prior_map_manager.hpp       # 先验地图管理
│   │   ├── map_loader.hpp              # 地图加载器
│   │   └── local_map_extractor.hpp     # 局部地图提取
│   ├── factors/
│   │   ├── scan_to_prior_map_factor.hpp      # S2M因子
│   │   └── degeneracy_aware_factor.hpp       # 退化感知因子
│   ├── initial_localization/
│   │   ├── initial_localizer.hpp       # 初始定位接口
│   │   ├── fpfh_localizer.hpp          # FPFH+RANSAC
│   │   └── scan_context_localizer.hpp  # ScanContext重定位
│   └── localization_extension.hpp      # GLIM扩展入口
├── src/
│   └── ... (实现文件)
├── config/
│   └── localization_params.yaml
└── launch/
    └── localization.launch.py
```

---

## 4. 详细设计

### 4.1 先验地图管理器

#### 4.1.1 功能描述

- 支持多种地图格式：PCD、PLY、GLIM SimpleMap
- 构建 GPU 加速的 GaussianVoxelMap
- 支持大地图分块加载（内存管理）
- 提供局部地图裁剪功能

#### 4.1.2 接口定义

```cpp
namespace glim_localization {

/**
 * @brief 先验地图管理器
 */
class PriorMapManager {
public:
  struct Config {
    double voxel_resolution = 0.5;      // 体素分辨率 (m)
    double local_map_radius = 100.0;    // 局部地图半径 (m)
    bool use_gpu = true;                // 是否使用GPU
    int max_points_per_voxel = 20;      // 每个体素最大点数
  };

  explicit PriorMapManager(const Config& config);
  
  /**
   * @brief 加载先验地图
   * @param map_path 地图文件路径 (.pcd / .ply / .simplemap)
   * @return 是否加载成功
   */
  bool load_map(const std::string& map_path);
  
  /**
   * @brief 获取用于配准的体素地图
   * @return GaussianVoxelMap 指针
   */
  gtsam_points::GaussianVoxelMap::ConstPtr get_voxelmap() const;
  
  /**
   * @brief 提取当前位置周围的局部地图
   * @param center 中心位置
   * @param radius 半径 (m)
   * @return 局部点云
   */
  PointCloud::Ptr extract_local_map(
    const Eigen::Vector3d& center,
    double radius = -1  // -1 表示使用默认值
  ) const;
  
  /**
   * @brief 检查位置是否在地图范围内
   * @param position 待检查位置
   * @return 是否在范围内
   */
  bool is_in_map_bounds(const Eigen::Vector3d& position) const;
  
  /**
   * @brief 计算点云与地图的重叠率
   * @param cloud 输入点云
   * @param T_map_cloud 点云到地图的变换
   * @param max_dist 最大匹配距离
   * @return 重叠率 [0, 1]
   */
  double compute_overlap_ratio(
    const PointCloud::ConstPtr& cloud,
    const Eigen::Isometry3d& T_map_cloud,
    double max_dist = 1.0
  ) const;

  /**
   * @brief 获取地图边界
   */
  void get_map_bounds(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const;

private:
  Config config_;
  PointCloud::Ptr global_map_;
  gtsam_points::GaussianVoxelMap::Ptr voxelmap_;
  gtsam_points::KdTree::Ptr kdtree_;
  Eigen::Vector3d map_min_, map_max_;
};

} // namespace glim_localization
```

### 4.2 Scan-to-Prior-Map 因子

#### 4.2.1 功能描述

- 基于 VGICP 的点云配准因子
- 支持 GPU 加速
- 内置退化检测机制
- 自适应权重调整

#### 4.2.2 因子图结构

```
传统 FAST_LIO_LOCALIZATION:
  X(k-1) ──[IMU]── X(k)
                    │
                 [S2M]  ← 只有这一个LiDAR约束，地图出问题就完蛋
                    │
              Prior Map

本方案:
  X(k-1) ══[IMU]══ X(k)
    │               │
  [S2S]───────────[S2S]  ← Scan-to-Scan 约束（帧间里程计）
    │               │
  [S2M]           [S2M]  ← Scan-to-Prior-Map 约束（地图校正）
    │               │
  Prior Map ◀──────┘
  
  两类因子同时优化，地图出问题时 S2S 兜底
```

#### 4.2.3 接口定义

```cpp
namespace glim_localization {

/**
 * @brief Scan-to-Prior-Map 配准因子
 * 
 * 基于 gtsam_points::IntegratedVGICPFactor 设计，
 * 增加退化检测和自适应权重功能
 */
class ScanToPriorMapFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
  struct Config {
    double max_correspondence_dist = 1.0;   // 最大对应点距离
    int max_iterations = 10;                // 最大迭代次数
    double rotation_epsilon = 1e-4;         // 旋转收敛阈值
    double translation_epsilon = 1e-4;      // 平移收敛阈值
    
    // 退化检测参数
    double min_overlap_ratio = 0.05;        // 最小重叠率
    double degeneracy_eigenvalue_thresh = 0.01;  // 退化特征值阈值
  };

  /**
   * @brief 构造函数
   * @param pose_key 位姿变量 Key
   * @param source 当前帧点云
   * @param target_voxelmap 先验地图体素图
   * @param noise_model 噪声模型
   * @param config 配置参数
   */
  ScanToPriorMapFactor(
    gtsam::Key pose_key,
    const gtsam_points::PointCloud::ConstPtr& source,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
    const gtsam::SharedNoiseModel& noise_model,
    const Config& config = Config()
  );

  /**
   * @brief 计算误差
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3& pose,
    boost::optional<gtsam::Matrix&> H = boost::none
  ) const override;

  /**
   * @brief 检查因子是否退化
   * @return true 表示退化，应降低该因子权重
   */
  bool is_degenerate() const { return is_degenerate_; }
  
  /**
   * @brief 获取重叠率
   */
  double overlap_ratio() const { return overlap_ratio_; }
  
  /**
   * @brief 获取退化方向信息
   * @return 6x6 矩阵，对角线元素表示各自由度的约束强度
   */
  Eigen::Matrix<double, 6, 6> degeneracy_info() const { 
    return degeneracy_info_; 
  }

private:
  /**
   * @brief 分析 Hessian 矩阵，检测退化
   */
  void analyze_degeneracy(const Eigen::Matrix<double, 6, 6>& H);
  
  /**
   * @brief 计算对应点和重叠率
   */
  void compute_correspondences(const gtsam::Pose3& pose);

  Config config_;
  gtsam_points::PointCloud::ConstPtr source_;
  gtsam_points::GaussianVoxelMap::ConstPtr target_voxelmap_;
  
  mutable bool is_degenerate_ = false;
  mutable double overlap_ratio_ = 0.0;
  mutable Eigen::Matrix<double, 6, 6> degeneracy_info_;
};

} // namespace glim_localization
```

#### 4.2.4 退化检测算法

```cpp
void ScanToPriorMapFactor::analyze_degeneracy(
  const Eigen::Matrix<double, 6, 6>& H
) {
  // 对 Hessian 矩阵进行特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
  Eigen::Vector6d eigenvalues = solver.eigenvalues();
  Eigen::Matrix<double, 6, 6> eigenvectors = solver.eigenvectors();
  
  // 初始化退化信息矩阵为单位阵
  degeneracy_info_.setIdentity();
  
  is_degenerate_ = false;
  
  for (int i = 0; i < 6; ++i) {
    if (eigenvalues(i) < config_.degeneracy_eigenvalue_thresh) {
      // 该方向退化，降低权重
      is_degenerate_ = true;
      
      // 计算该特征向量在各自由度上的投影，降低对应权重
      Eigen::Vector6d ev = eigenvectors.col(i);
      for (int j = 0; j < 6; ++j) {
        double projection = ev(j) * ev(j);
        degeneracy_info_(j, j) *= (1.0 - projection * 0.9);  // 保留10%约束
      }
    }
  }
}
```

### 4.3 初始定位模块

#### 4.3.1 功能描述

- 支持多种初始化方式
- FPFH + RANSAC 全局配准
- RViz 2D Pose Estimate 手动指定
- GNSS 位置初始化

#### 4.3.2 接口定义

```cpp
namespace glim_localization {

/**
 * @brief 初始定位器接口
 */
class InitialLocalizer {
public:
  struct Result {
    bool success = false;
    Eigen::Isometry3d pose;
    double confidence = 0.0;      // 置信度 [0, 1]
    double fitness_score = 0.0;   // 配准得分
    std::string method;           // 使用的方法
  };

  virtual ~InitialLocalizer() = default;
  
  /**
   * @brief 执行初始定位
   * @param scan 当前帧点云
   * @param prior_map 先验地图
   * @param initial_guess 初始猜测 (可选)
   * @return 定位结果
   */
  virtual Result localize(
    const PointCloud::ConstPtr& scan,
    const PriorMapManager& prior_map,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity()
  ) = 0;
};

/**
 * @brief 基于 FPFH + RANSAC 的全局定位器
 */
class FPFHLocalizer : public InitialLocalizer {
public:
  struct Config {
    double fpfh_radius = 0.5;           // FPFH 计算半径
    double ransac_inlier_thresh = 0.3;  // RANSAC 内点阈值
    int ransac_max_iterations = 50000;  // RANSAC 最大迭代
    double icp_refine_max_dist = 1.0;   // ICP 精化最大距离
  };

  explicit FPFHLocalizer(const Config& config);
  
  Result localize(
    const PointCloud::ConstPtr& scan,
    const PriorMapManager& prior_map,
    const Eigen::Isometry3d& initial_guess
  ) override;

private:
  Config config_;
};

/**
 * @brief 基于手动指定的定位器 (RViz 2D Pose Estimate)
 */
class ManualLocalizer : public InitialLocalizer {
public:
  /**
   * @brief 设置手动指定的位姿
   */
  void set_initial_pose(const Eigen::Isometry3d& pose);
  
  /**
   * @brief 检查是否有待处理的手动位姿
   */
  bool has_pending_pose() const;
  
  Result localize(
    const PointCloud::ConstPtr& scan,
    const PriorMapManager& prior_map,
    const Eigen::Isometry3d& initial_guess
  ) override;

private:
  std::optional<Eigen::Isometry3d> pending_pose_;
  std::mutex mutex_;
};

} // namespace glim_localization
```

### 4.4 GLIM 扩展集成

#### 4.4.1 功能描述

- 通过 GLIM 回调机制注入定位因子
- 管理定位状态（初始化、正常运行、丢失）
- 自适应调整 S2M 因子权重

#### 4.4.2 接口定义

```cpp
namespace glim_localization {

/**
 * @brief GLIM 定位扩展模块
 * 
 * 通过 GLIM 的全局回调插槽机制，向因子图中注入
 * Scan-to-Prior-Map 约束
 */
class LocalizationExtension {
public:
  enum class State {
    UNINITIALIZED,    // 等待初始化
    INITIALIZING,     // 正在初始化
    TRACKING,         // 正常跟踪
    LOST              // 跟踪丢失
  };

  struct Config {
    std::string prior_map_path;           // 先验地图路径
    double voxel_resolution = 0.5;        // 体素分辨率
    
    // S2M 因子参数
    double s2m_min_overlap = 0.05;        // 最小重叠率
    double s2m_full_weight_overlap = 0.3; // 完全权重的重叠率
    double s2m_noise_rotation = 0.1;      // 旋转噪声 (rad)
    double s2m_noise_translation = 0.2;   // 平移噪声 (m)
    
    // 初始化参数
    std::string init_method = "manual";   // manual / fpfh / gnss
    double init_confidence_thresh = 0.5;  // 初始化置信度阈值
    
    // 丢失检测参数
    int lost_frame_thresh = 10;           // 连续低重叠帧数阈值
  };

  explicit LocalizationExtension(const Config& config);
  ~LocalizationExtension();
  
  /**
   * @brief 初始化模块，注册回调
   * @return 是否初始化成功
   */
  bool initialize();
  
  /**
   * @brief 获取当前状态
   */
  State get_state() const { return state_; }
  
  /**
   * @brief 请求重定位
   * @param initial_guess 初始猜测
   */
  void request_relocalization(
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity()
  );
  
  /**
   * @brief 设置 RViz 2D Pose Estimate 回调
   */
  void set_initial_pose_callback(
    std::function<void(const Eigen::Isometry3d&)> callback
  );

private:
  // GLIM 回调函数
  void on_new_frame(const glim::EstimationFrame::ConstPtr& frame);
  void on_smoother_update(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values
  );
  void on_marginalized_frames(
    const std::vector<glim::EstimationFrame::ConstPtr>& frames
  );
  
  /**
   * @brief 创建 S2M 因子
   */
  gtsam::NonlinearFactor::shared_ptr create_s2m_factor(
    const glim::EstimationFrame::ConstPtr& frame,
    double weight_scale = 1.0
  );
  
  /**
   * @brief 尝试初始化
   */
  bool try_initialize(const glim::EstimationFrame::ConstPtr& frame);
  
  /**
   * @brief 更新状态机
   */
  void update_state(double overlap_ratio);

  Config config_;
  State state_ = State::UNINITIALIZED;
  
  std::unique_ptr<PriorMapManager> map_manager_;
  std::unique_ptr<InitialLocalizer> localizer_;
  
  // 待添加到因子图的因子
  std::vector<gtsam::NonlinearFactor::shared_ptr> pending_factors_;
  std::mutex pending_factors_mutex_;
  
  // 状态统计
  int low_overlap_count_ = 0;
  double last_overlap_ratio_ = 0.0;
  
  // 回调连接句柄
  std::vector<boost::signals2::connection> callback_connections_;
};

} // namespace glim_localization
```

### 4.5 配置文件

```yaml
# config/localization_params.yaml

# 先验地图配置
prior_map:
  path: "/path/to/prior_map.pcd"
  voxel_resolution: 0.5          # 体素分辨率 (m)
  use_gpu: true                  # 使用GPU加速
  max_points_per_voxel: 20       # 每体素最大点数

# Scan-to-Prior-Map 因子配置
s2m_factor:
  enabled: true
  max_correspondence_dist: 1.0   # 最大对应点距离 (m)
  min_overlap_ratio: 0.05        # 最小重叠率 (低于此值不添加因子)
  full_weight_overlap: 0.30      # 完全权重重叠率
  noise:
    rotation: 0.1                # 旋转噪声 (rad)
    translation: 0.2             # 平移噪声 (m)
  degeneracy:
    enable_detection: true       # 启用退化检测
    eigenvalue_thresh: 0.01      # 退化特征值阈值

# 初始定位配置
initial_localization:
  method: "manual"               # manual / fpfh / gnss
  fpfh:
    radius: 0.5
    ransac_inlier_thresh: 0.3
    ransac_max_iterations: 50000
  manual:
    timeout: 30.0                # 等待手动输入超时 (s)
  confidence_threshold: 0.5      # 初始化置信度阈值

# 状态管理
state_management:
  lost_frame_threshold: 10       # 连续低重叠帧数触发LOST状态
  auto_relocalize: true          # 丢失时自动重定位
  
# 调试输出
debug:
  publish_local_map: true        # 发布局部地图
  publish_overlap_info: true     # 发布重叠信息
  save_trajectory: true          # 保存轨迹
```

---

## 5. 关键算法

### 5.1 自适应因子权重

根据 Scan 与 Prior Map 的重叠率，动态调整 S2M 因子权重：

```cpp
double compute_factor_weight(double overlap_ratio) {
  const double min_overlap = config_.s2m_min_overlap;      // 0.05
  const double full_overlap = config_.s2m_full_weight_overlap;  // 0.30
  
  if (overlap_ratio < min_overlap) {
    // 重叠率太低，不添加因子
    return 0.0;
  }
  
  if (overlap_ratio >= full_overlap) {
    // 重叠率足够，完全权重
    return 1.0;
  }
  
  // 线性插值
  return (overlap_ratio - min_overlap) / (full_overlap - min_overlap);
}
```

### 5.2 退化处理流程

```
┌─────────────────────────────────────────────────────────────┐
│                    每帧处理流程                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. 接收新帧 LiDAR 点云                                     │
│         │                                                   │
│         ▼                                                   │
│  2. 计算与 Prior Map 的重叠率                               │
│         │                                                   │
│         ├─── overlap < 5% ───▶ 不添加 S2M 因子             │
│         │                      (纯 LIO 模式)                │
│         │                                                   │
│         ├─── 5% ≤ overlap < 30% ──▶ 降权 S2M 因子          │
│         │                                                   │
│         └─── overlap ≥ 30% ───▶ 全权重 S2M 因子            │
│                   │                                         │
│                   ▼                                         │
│  3. 构建 S2M 因子，计算 Hessian                            │
│         │                                                   │
│         ▼                                                   │
│  4. 特征值分解检测退化方向                                  │
│         │                                                   │
│         ├─── 存在退化方向 ───▶ 修改噪声模型                │
│         │                      (退化方向噪声增大)           │
│         │                                                   │
│         └─── 无退化 ───▶ 使用原始噪声模型                  │
│                   │                                         │
│                   ▼                                         │
│  5. 将因子添加到待优化队列                                  │
│         │                                                   │
│         ▼                                                   │
│  6. GLIM 滑动窗口优化 (S2S + S2M + IMU)                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.3 状态机

```
                    ┌─────────────────┐
                    │  UNINITIALIZED  │
                    └────────┬────────┘
                             │
                   收到初始位姿/FPFH成功
                             │
                             ▼
                    ┌─────────────────┐
          ┌────────│  INITIALIZING   │────────┐
          │        └────────┬────────┘        │
          │                 │                 │
     初始化失败        初始化成功         超时
          │                 │                 │
          ▼                 ▼                 ▼
   ┌──────────┐      ┌──────────┐      ┌──────────┐
   │   LOST   │◀─────│ TRACKING │      │   LOST   │
   └────┬─────┘      └────┬─────┘      └────┬─────┘
        │                 │                  │
        │    连续N帧      │                  │
        │    低重叠率     │                  │
        │        ┌────────┘                  │
        │        │                           │
        └────────┼───────────────────────────┘
                 │
           请求重定位
                 │
                 ▼
        ┌─────────────────┐
        │  INITIALIZING   │
        └─────────────────┘
```

---

## 6. ROS2 接口

### 6.1 订阅话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D Pose Estimate |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS 位置 (可选) |

### 6.2 发布话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/localization/pose` | `geometry_msgs/PoseStamped` | 定位结果 |
| `/localization/status` | `std_msgs/String` | 定位状态 |
| `/localization/overlap` | `std_msgs/Float64` | 重叠率 |
| `/localization/local_map` | `sensor_msgs/PointCloud2` | 局部地图 (调试) |

### 6.3 服务

| 服务名 | 服务类型 | 描述 |
|--------|----------|------|
| `/localization/load_map` | `std_srvs/SetString` | 加载地图 |
| `/localization/relocalize` | `std_srvs/Trigger` | 请求重定位 |
| `/localization/get_state` | `std_srvs/Trigger` | 获取状态 |

### 6.4 Launch 文件示例

```python
# launch/localization.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'prior_map_path',
            default_value='/path/to/map.pcd',
            description='Path to prior map'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='config/localization_params.yaml',
            description='Path to config file'
        ),
        
        # GLIM 核心节点
        Node(
            package='glim_ros2',
            executable='glim_ros2',
            name='glim',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
            }],
            remappings=[
                ('/points', '/lidar/points'),
                ('/imu', '/imu/data'),
            ],
        ),
        
        # 定位扩展节点
        Node(
            package='glim_localization',
            executable='localization_extension_node',
            name='localization_extension',
            parameters=[{
                'prior_map_path': LaunchConfiguration('prior_map_path'),
                'config_path': LaunchConfiguration('config_path'),
            }],
        ),
    ])
```

---

## 7. 开发计划

### 7.1 阶段划分

| 阶段 | 任务 | 交付物 | 预计工时 |
|------|------|--------|----------|
| **P0** | 环境搭建 | GLIM + gtsam_points 编译通过 | 2天 |
| **P1** | 先验地图模块 | PriorMapManager 类 | 3天 |
| **P2** | S2M 因子实现 | ScanToPriorMapFactor 类 | 5天 |
| **P3** | GLIM 集成 | LocalizationExtension 类 | 3天 |
| **P4** | 初始定位 | FPFH/Manual Localizer | 3天 |
| **P5** | 退化处理 | 退化检测与自适应权重 | 4天 |
| **P6** | ROS2 封装 | 节点、Launch、配置 | 2天 |
| **P7** | 测试调优 | 单元测试、实车测试 | 5天 |
| | **总计** | | **27天** |

### 7.2 里程碑

```
Week 1: P0 + P1 完成
        ├── GLIM 环境可用
        └── 地图加载功能验证

Week 2-3: P2 + P3 完成
        ├── S2M 因子单独测试通过
        └── 与 GLIM 因子图集成

Week 4: P4 + P5 完成
        ├── 初始定位功能可用
        └── 退化检测功能可用

Week 5: P6 + P7 完成
        ├── ROS2 节点发布
        └── 实车测试通过
```

---

## 8. 测试计划

### 8.1 单元测试

| 测试项 | 测试内容 | 通过标准 |
|--------|----------|----------|
| 地图加载 | PCD/PLY 格式加载 | 点数一致 |
| VoxelMap 构建 | GPU 体素图构建 | 无内存泄漏 |
| 重叠率计算 | 不同重叠场景 | 误差 < 5% |
| S2M 因子 | 误差和雅可比验证 | 数值梯度一致 |
| 退化检测 | 长廊/平面场景 | 正确识别退化方向 |

### 8.2 集成测试

| 测试场景 | 测试内容 | 通过标准 |
|----------|----------|----------|
| 完整地图覆盖 | 全程在地图范围内 | APE < 0.1m |
| 地图边界 | 进出地图边界 | 无跳变，平滑过渡 |
| 地图缺失区域 | 穿越未建图区域 | 自动退化为LIO，回来后收敛 |
| 动态障碍物 | 地图中有变化 | 不被错误匹配影响 |
| 快速运动 | 1m/s+ 运动 | 不丢失 |

### 8.3 性能测试

| 指标 | 目标值 | 测试方法 |
|------|--------|----------|
| 处理频率 | ≥ 10Hz (100ms/帧) | 计时统计 |
| GPU 显存 | < 2GB | nvidia-smi 监控 |
| CPU 占用 | < 200% (双核) | top 监控 |
| 内存占用 | < 4GB | top 监控 |

---

## 9. 风险与对策

| 风险 | 影响 | 概率 | 对策 |
|------|------|------|------|
| GLIM 回调机制限制 | 无法注入自定义因子 | 低 | 研究源码，必要时 fork 修改 |
| GPU 显存不足 | 大地图无法加载 | 中 | 实现分块加载机制 |
| 退化检测不准 | 错误降权导致漂移 | 中 | 保守阈值 + 多帧平滑 |
| 初始定位失败率高 | 用户体验差 | 中 | 多方法融合 + 人工兜底 |
| 性能不达标 | 无法实时运行 | 低 | GPU优化 + 降采样 |

---

## 10. 参考资料

### 10.1 论文

1. Koide et al., "GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors", RAS 2024
2. Koide et al., "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA 2022
3. Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA 2021
4. Koide et al., "Tightly Coupled Range Inertial Localization on a 3D Prior Map Based on Sliding Window Factor Graph Optimization", ICRA 2024 (GLIL)

### 10.2 开源项目

- GLIM: https://github.com/koide3/glim
- gtsam_points: https://github.com/koide3/gtsam_points
- GTSAM: https://github.com/borglab/gtsam
- GLIM 文档: https://koide3.github.io/glim/

### 10.3 相关实现参考

- Lidar_IMU_Localization: https://github.com/chengwei0427/Lidar_IMU_Localization
- MOLA-LO: https://github.com/MOLAorg/mola_lidar_odometry
- FAST_LIO_LOCALIZATION: https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION

---

## 附录 A: GLIM 回调机制说明

GLIM 提供以下回调点用于扩展：

```cpp
// glim/odometry/callbacks.hpp
namespace glim {

struct OdometryEstimationCallbacks {
  // 新帧创建时触发
  static CallbackSlot<void(const EstimationFrame::ConstPtr&)> on_new_frame;
  
  // 滑动窗口更新前触发 (可添加因子)
  static CallbackSlot<void(gtsam::NonlinearFactorGraph&, gtsam::Values&)> 
    on_smoother_update;
  
  // 帧被边缘化时触发
  static CallbackSlot<void(const std::vector<EstimationFrame::ConstPtr>&)> 
    on_marginalized_frames;
  
  // 优化完成后触发
  static CallbackSlot<void(const gtsam::Values&)> on_smoother_result;
};

} // namespace glim
```

使用示例：

```cpp
#include <glim/odometry/callbacks.hpp>

void setup_callbacks() {
  // 注册新帧回调
  glim::OdometryEstimationCallbacks::on_new_frame.add(
    [](const glim::EstimationFrame::ConstPtr& frame) {
      // 处理新帧
    }
  );
  
  // 注册滑动窗口更新回调
  glim::OdometryEstimationCallbacks::on_smoother_update.add(
    [](gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
      // 添加自定义因子到 graph
      graph.add(my_custom_factor);
    }
  );
}
```

---

## 附录 B: 术语表

| 术语 | 全称 | 说明 |
|------|------|------|
| S2S | Scan-to-Scan | 帧间点云配准 |
| S2M | Scan-to-Map | 点云到地图配准 |
| FGO | Factor Graph Optimization | 因子图优化 |
| VGICP | Voxelized GICP | 体素化广义ICP |
| ESKF | Error State Kalman Filter | 误差状态卡尔曼滤波 |
| LIO | LiDAR-Inertial Odometry | 激光-惯性里程计 |
| APE | Absolute Pose Error | 绝对位姿误差 |
| FPFH | Fast Point Feature Histogram | 快速点特征直方图 |

---

*文档结束*
