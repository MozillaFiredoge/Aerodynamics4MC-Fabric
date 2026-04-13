# Aerodynamics4MC

[![Build Status](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/build/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![Native Matrix](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/native-matrix/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

[English](#english) | [中文](#中文)

---

## 中文

### 简介

**Aerodynamics4MC** 是一个为 Minecraft Fabric 开发的实时空气动力学模拟模组。它使用基于格子玻尔兹曼方法（LBM）的 CFD（计算流体力学）求解器，在游戏内实现真实的风场模拟。

#### 核心特性

- 🌪️ **实时风场模拟**：基于 D3Q27 LBM 模型，每 tick 更新风场状态
- 🌀 **风扇与整流系统**：添加可交互的风扇方块和整流罩方块
- 🌡️ **热力学耦合**：支持热浮力（Boussinesq 效应）模拟
- 🏗️ **多尺度模拟**：L0（背景）和 L1（中尺度）双层网格系统
- 🔧 **Native 加速**：通过 JNI 调用本地库实现高性能计算
- 🖥️ **客户端可视化**：实时风速矢量场渲染

### 环境要求

| 组件 | 版本要求 |
|------|----------|
| Minecraft | 1.21.11 |
| Fabric Loader | >= 0.18.2 |
| Java | >= 21 |
| Fabric API | 0.141.2+ |

**支持平台**：
- Linux x86_64 / ARM64
- Windows x86_64
- macOS ARM64

### 安装指南

#### 方式一：下载预编译版本

1. 前往 [GitHub Actions](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions) 下载最新构建的 `mod-jar` 工件
2. 将 `.jar` 文件放入 Minecraft 安装目录的 `mods` 文件夹

#### 方式二：从源码构建

```bash
# 克隆仓库
git clone https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric.git
cd Aerodynamics4MC-Fabric

# 构建开发版本
./gradlew build

# 构建发布版本（包含 Native 库）
./gradlew remapJar
```

**注意**：完整的 native 构建需要 CMake 和平台特定的 OpenCL SDK，详见 [native/README.md](native/README.md)。

### 使用方法

#### 游戏内命令

所有命令需要管理员权限（OP 权限等级 2+）：

| 命令 | 说明 |
|------|------|
| `/aero start` | 启动风场实时模拟 |
| `/aero stop` | 停止模拟并清理运行时状态 |
| `/aero status` | 显示详细运行状态统计 |
| `/aero dumpdata` | 导出 L0/L1 网格诊断数据到世界目录 |
| `/aero dump_l1` | 导出 L1（中尺度）网格快照 |

#### 方块说明

##### 风扇（Fan）

- **功能**：产生定向气流，是风场的主要驱动力
- **放置**：根据玩家朝向自动确定风向
- **交互**：放置后自动作为风源参与模拟

##### 整流罩（Duct）

- **功能**：引导气流方向，优化风场分布
- **用途**：构建风道系统，减少涡流损失

#### 多尺度网格系统

模组使用三层嵌套网格系统实现不同尺度的气象模拟：

| 层级 | 名称 | 网格尺寸 | 分辨率 | 覆盖范围 |
|------|------|----------|--------|----------|
| L0 | 背景气象网格 | 41×41 cells | 256 方块/cell | ~10km×10km |
| L1 | 中尺度网格 | 33×33×8 cells | 64×64×40 方块/cell | ~2km×2km×320m |
| CFD | LBM 仿真网格 | 64×64×64 voxels | 1 方块/voxel | 64×64×64 方块 |

- **L0 背景网格**：粗粒度大范围气候模拟（半径 20 cells）
- **L1 中尺度网格**：中等分辨率区域气象（半径 16 cells，8 层高度）
- **CFD 仿真网格**：细粒度实时 LBM 流体仿真（64³ 体素）

#### 模拟区域

- 每个活跃 CFD 区域为 **64×64×64** 体素（64米边长）
- 空间分辨率：1 体素 = 1 方块
- 时间步长：0.05 秒（20 ticks/秒）

### 技术架构

```
┌─────────────────────────────────────────────────────────────┐
│                     客户端 (Client)                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ 风场可视化    │  │ 粒子渲染      │  │ AeroVisualizer│      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
                         网络同步
                              │
┌─────────────────────────────────────────────────────────────┐
│                     服务端 (Server)                          │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            AeroServerRuntime (主运行时)               │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐│  │
│  │  │ L0 背景网格   │  │ L1 中尺度网格 │  │ 边界耦合器    ││  │
│  │  │ 41×41 cells  │  │ 33×33×8 cells│  │              ││  │
│  │  │ 256m/cell    │  │ 64×64×40m    │  │              ││  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘│  │
│  └──────────────────────────────────────────────────────┘  │
│                         │                                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              CFD 仿真层 (LBM)                         │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  D3Q27 LBM 求解器                               │  │  │
│  │  │  - 64×64×64 voxels 每区域                        │  │  │
│  │  │  - Cumulant 碰撞模型                            │  │  │
│  │  │  - SGS 亚格子模型                               │  │  │
│  │  │  - OpenCL/CPU 双后端                            │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 配置与调优

#### 性能参数

在 `AeroServerRuntime.java` 中可调整以下常量：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **CFD 仿真网格** |||
| `GRID_SIZE` | 64 | CFD 区域尺寸（64×64×64 体素） |
| **L0 背景气象网格** |||
| `BACKGROUND_MET_CELL_SIZE_BLOCKS` | 256 | L0 网格单元大小（方块） |
| `BACKGROUND_MET_RADIUS_CELLS` | 20 | L0 网格半径（cells） |
| `BACKGROUND_MET_REFRESH_TICKS` | 240 | L0 刷新间隔（ticks） |
| **L1 中尺度网格** |||
| `MESOSCALE_MET_CELL_SIZE_BLOCKS` | 64 | L1 网格单元大小（方块） |
| `MESOSCALE_MET_RADIUS_CELLS` | 16 | L1 网格半径（cells） |
| `MESOSCALE_MET_LAYER_HEIGHT_BLOCKS` | 40 | L1 层高度（方块） |
| `MESOSCALE_MET_MAX_LAYERS` | 8 | L1 最大层数 |
| **物理参数** |||
| `MAX_SAFE_LATTICE_SPEED` | 0.28 | 最大安全格子速度 |
| `DEFAULT_FAN_INFLOW_SPEED` | 4.0 | 默认风扇入流速度 (m/s) |
| `FAN_RADIUS` | 1 | 风扇影响半径（方块） |
| `DUCT_SCAN_MAX` | 20 | 整流罩最大扫描深度 |

#### 热力学参数

在 `native/src/aero_lbm_jni.cpp` 中：

| 参数 | 说明 |
|------|------|
| `kThermalDiffusivity` | 热扩散系数 |
| `kThermalCooling` | 热冷却速率 |
| `kBoussinesqBeta` | 热膨胀系数 |
| `kBoussinesqForceMax` | 最大浮力 |

### 开发计划

- [ ] 可配置的热源模式（风扇衍生/方块定义/脚本区域）
- [ ] 增量上传优化（仅传输障碍物/风扇变化）
- [ ] 混合精度路径（FP16/Int16 存储）
- [ ] 更多边界条件变体（流入/流出/滑移）

### 构建 Native 库

#### Linux (本地构建)

```bash
cd native
sudo apt-get install -y ocl-icd-opencl-dev opencl-c-headers opencl-clhpp-headers
cmake -S . -B build
cmake --build build -j
```

#### 跨平台 Superbuild（一次性构建 4 个平台）

```bash
cd native
cmake -S . -B build-all \
  -DAERO_LBM_SUPERBUILD=ON \
  -DAERO_LBM_DIST_DIR=$PWD/dist/natives
cmake --build build-all -j
```

更多构建选项详见 [native/README.md](native/README.md)。

### 贡献指南

欢迎提交 Issue 和 PR！请确保：

1. 代码符合项目现有风格
2. 重大变更先开 Issue 讨论
3. 提交信息清晰描述变更内容

### 致谢

- 核心 CFD 求解器基于格子玻尔兹曼方法（LBM）
- 使用 [Fabric](https://fabricmc.net/) 模组框架
- Native 库使用 OpenCL 实现 GPU 加速

---

## English

### Overview

**Aerodynamics4MC** is a real-time aerodynamics simulation mod for Minecraft Fabric. It uses a CFD (Computational Fluid Dynamics) solver based on the Lattice Boltzmann Method (LBM) to achieve authentic wind field simulation within the game.

#### Core Features

- 🌪️ **Real-time Wind Simulation**: D3Q27 LBM model with per-tick wind state updates
- 🌀 **Fan & Duct System**: Interactive fan blocks and duct blocks for airflow control
- 🌡️ **Thermodynamic Coupling**: Supports thermal buoyancy (Boussinesq effect) simulation
- 🏗️ **Multi-scale Simulation**: Dual-layer grid system with L0 (background) and L1 (mesoscale)
- 🔧 **Native Acceleration**: High-performance computing via JNI native libraries
- 🖥️ **Client Visualization**: Real-time wind velocity vector field rendering

### Requirements

| Component | Version |
|-----------|---------|
| Minecraft | 1.21.11 |
| Fabric Loader | >= 0.18.2 |
| Java | >= 21 |
| Fabric API | 0.141.2+ |

**Supported Platforms**:
- Linux x86_64 / ARM64
- Windows x86_64
- macOS ARM64

### Installation

#### Option 1: Download Pre-built

1. Download the latest `mod-jar` artifact from [GitHub Actions](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
2. Place the `.jar` file in your Minecraft `mods` folder

#### Option 2: Build from Source

```bash
# Clone repository
git clone https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric.git
cd Aerodynamics4MC-Fabric

# Build development version
./gradlew build

# Build release (with native libraries)
./gradlew remapJar
```

**Note**: Full native build requires CMake and platform-specific OpenCL SDK. See [native/README.md](native/README.md) for details.

### Usage

#### In-game Commands

All commands require admin privileges (OP permission level 2+):

| Command | Description |
|---------|-------------|
| `/aero start` | Start real-time wind simulation |
| `/aero stop` | Stop simulation and clear runtime state |
| `/aero status` | Display detailed runtime statistics |
| `/aero dumpdata` | Export L0/L1 grid diagnostic data to world directory |
| `/aero dump_l1` | Export L1 (mesoscale) grid snapshot |

#### Blocks

##### Fan

- **Function**: Generates directional airflow, primary driver of wind field
- **Placement**: Orientation automatically aligned to player facing
- **Interaction**: Automatically participates in simulation as wind source when placed

##### Duct

- **Function**: Guides airflow direction, optimizes wind field distribution
- **Purpose**: Build duct systems to reduce vortex losses

#### Multi-scale Grid System

The mod uses a three-tier nested grid system for meteorological simulation at different scales:

| Tier | Name | Grid Size | Resolution | Coverage |
|------|------|-----------|------------|----------|
| L0 | Background Met Grid | 41×41 cells | 256 blocks/cell | ~10km×10km |
| L1 | Mesoscale Grid | 33×33×8 cells | 64×64×40 blocks/cell | ~2km×2km×320m |
| CFD | LBM Simulation Grid | 64×64×64 voxels | 1 block/voxel | 64×64×64 blocks |

- **L0 Background**: Coarse large-scale climate simulation (radius 20 cells)
- **L1 Mesoscale**: Medium-resolution regional meteorology (radius 16 cells, 8 vertical layers)
- **CFD Simulation Grid**: Fine-grained real-time LBM fluid simulation (64³ voxels)

#### Simulation Domain

- Each active CFD region is **64×64×64** voxels (64m cube)
- Spatial resolution: 1 voxel = 1 block
- Time step: 0.05 seconds (20 ticks/second)

### Technical Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Client Side                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Flow Viz     │  │ Particle     │  │ AeroVisualizer│      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
                         Network Sync
                              │
┌─────────────────────────────────────────────────────────────┐
│                     Server Side                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            AeroServerRuntime (Main Runtime)          │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐│  │
│  │  │ L0 Background│  │ L1 Mesoscale │  │ Boundary     ││  │
│  │  │ 41×41 cells  │  │ 33×33×8 cells│  │ Coupler      ││  │
│  │  │ 256m/cell    │  │ 64×64×40m    │  │              ││  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘│  │
│  └──────────────────────────────────────────────────────┘  │
│                         │                                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              CFD Layer (LBM)                          │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │  D3Q27 LBM Solver                               │  │  │
│  │  │  - 64×64×64 voxels per region                    │  │  │
│  │  │  - Cumulant collision model                     │  │  │
│  │  │  - SGS subgrid model                            │  │  │
│  │  │  - OpenCL/CPU dual backend                      │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Configuration & Tuning

#### Performance Parameters

Adjustable constants in `AeroServerRuntime.java`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| **CFD Simulation Grid** |||
| `GRID_SIZE` | 64 | CFD region size (64×64×64 voxels) |
| **L0 Background Met Grid** |||
| `BACKGROUND_MET_CELL_SIZE_BLOCKS` | 256 | L0 grid cell size (blocks) |
| `BACKGROUND_MET_RADIUS_CELLS` | 20 | L0 grid radius (cells) |
| `BACKGROUND_MET_REFRESH_TICKS` | 240 | L0 refresh interval (ticks) |
| **L1 Mesoscale Grid** |||
| `MESOSCALE_MET_CELL_SIZE_BLOCKS` | 64 | L1 grid cell size (blocks) |
| `MESOSCALE_MET_RADIUS_CELLS` | 16 | L1 grid radius (cells) |
| `MESOSCALE_MET_LAYER_HEIGHT_BLOCKS` | 40 | L1 layer height (blocks) |
| `MESOSCALE_MET_MAX_LAYERS` | 8 | L1 maximum layers |
| **Physics Parameters** |||
| `MAX_SAFE_LATTICE_SPEED` | 0.28 | Maximum safe lattice speed |
| `DEFAULT_FAN_INFLOW_SPEED` | 4.0 | Default fan inflow speed (m/s) |
| `FAN_RADIUS` | 1 | Fan influence radius (blocks) |
| `DUCT_SCAN_MAX` | 20 | Maximum duct scan depth |

#### Thermodynamic Parameters

In `native/src/aero_lbm_jni.cpp`:

| Parameter | Description |
|-----------|-------------|
| `kThermalDiffusivity` | Thermal diffusivity coefficient |
| `kThermalCooling` | Thermal cooling rate |
| `kBoussinesqBeta` | Thermal expansion coefficient |
| `kBoussinesqForceMax` | Maximum buoyancy force |

### Roadmap

- [ ] Configurable thermal source modes (fan-derived/block-defined/scripted regions)
- [ ] Incremental upload optimization (transfer only obstacle/fan changes)
- [ ] Mixed precision path (FP16/Int16 storage)
- [ ] More boundary condition variants (inflow/outflow/slip)

### Building Native Libraries

#### Linux (Local Build)

```bash
cd native
sudo apt-get install -y ocl-icd-opencl-dev opencl-c-headers opencl-clhpp-headers
cmake -S . -B build
cmake --build build -j
```

#### Cross-platform Superbuild (4 platforms at once)

```bash
cd native
cmake -S . -B build-all \
  -DAERO_LBM_SUPERBUILD=ON \
  -DAERO_LBM_DIST_DIR=$PWD/dist/natives
cmake --build build-all -j
```

For more build options, see [native/README.md](native/README.md).

### Contributing

Issues and PRs are welcome! Please ensure:

1. Code follows existing project style
2. Open an Issue first for significant changes
3. Clear commit messages describing the changes

### Acknowledgments

- Core CFD solver based on Lattice Boltzmann Method (LBM)
- Built with [Fabric](https://fabricmc.net/) mod framework
- Native library uses OpenCL for GPU acceleration

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Related Projects

- Core training/data generation/Python solver: [aerodynamics4mc](https://github.com/MozillaFiredoge/Aerodynamics4MC) (separate repository)
