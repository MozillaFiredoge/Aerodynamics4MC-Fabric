# Aerodynamics4MC Fabric Mod

## 中文版

### 简介
这是 `Aerodynamics4MC` 的 Minecraft Fabric 客户端模组仓库，用于在游戏内：
- 采集体素窗口（障碍物、风扇源、状态场）
- 驱动风场求解后端（Socket 或 Native）
- 将风场作用到实体并进行可视化
- 低分辨率背景环流场 + Boussinesq 近似浮力耦合（昼夜热异常）

> 说明：核心训练/数据生成/PhiFlow Python 后端在独立仓库 `aerodynamics4mc`（core）中维护。

### 环境要求
- Minecraft: `1.21.11`
- Fabric Loader: `>=0.18.2`
- Java: `21`
- Gradle: 使用仓库内 `./gradlew`

### 快速启动
```bash
cd fabric-mod
./gradlew runClient
```

### 主要命令（游戏内）
- `/aero start`：开始每 tick 风场更新
- `/aero stop`：停止更新并清理状态
- `/aero maxspeed <value>`：限制风速上限
- `/aero streamline <1|2|4|8>`：设置流线采样步长
- `/aero streamline roi set <minX> <maxX> <minY> <maxY> <minZ> <maxZ>`：设置局部流线 ROI（窗口局部坐标 0..127）
- `/aero streamline roi off`：关闭 ROI，恢复全窗口流线
- `/aero backend socket`：使用 TCP Python 后端
- `/aero backend native`：使用 JNI Native 后端
- `/aero backend status`：查看当前后端与 native timing
- `/aero debug`：切换调试渲染
- `/aero render background on|off|status`：控制/查看背景环流矢量调试叠加
- `/aero render thermal on|off|status`：控制/查看热异常调试叠加

### 后端模式
1. `socket`（默认）
- 模组通过 TCP 发送 `(N,N,N,C)` 数据包到 Python 后端。
- 适合快速迭代物理参数与算法。

2. `native`
- 模组通过 JNI 调用内置动态库（自动从 jar 解压后加载）。
- 支持按平台加载：`linux/windows/macos` + `x86_64/arm64`（Windows 仅 `x86_64`）。

### 低分辨率背景环流场
- 运行时会按维度维护低分辨率背景风场（默认约 `16` 方块一格），并在窗口范围扩大时自动提升 coarse cell 尺寸以扩大覆盖范围。
- 背景场不再使用写死解析风型，改为在粗网格上用 Native C-LBM 动态推进速度场。
- `thermal_anomaly` 由世界采样驱动（方块热源/亮度/高度/日照/生物群系/天气加权），并通过 Boussinesq 近似转化为粗网格浮力源项。
- 在窗口求解输入阶段，状态速度会以小权重向粗网格背景风场松弛；局部窗口不再额外注入单独的“写死浮力脉冲”。

### Native 多平台构建与打包
详见：`native/README.md`

关键点：
- CMake 支持一次性 superbuild 目标（4 个动态库）
- Gradle `prepareNativeResources` 会将 `native/dist/natives/**` 打包进 mod jar
- GitHub Actions workflow：`.github/workflows/native-matrix.yml`

### 常用构建命令
```bash
# 开发构建
./gradlew build

# 生成可发布 remapped jar
./gradlew remapJar

# 仅处理资源（包含 native 打包）
./gradlew processResources
```

### 项目结构（关键目录）
- `src/main/java/com/aerodynamics4mc/`：方块、方块实体、主入口
- `src/client/java/com/aerodynamics4mc/client/`：客户端风场逻辑、物理施加、渲染、JNI 桥接
- `native/`：JNI + Native solver + CMake + 多平台构建配置
- `.github/workflows/native-matrix.yml`：五平台动态库构建与打包 CI

---

## English

### Overview
This is the Fabric client mod repository for `Aerodynamics4MC`. It is responsible for:
- capturing voxel windows in-game (obstacles, fan sources, state fields)
- driving airflow solver backends (Socket or Native)
- applying wind forces to entities and rendering debug visuals
- low-resolution background circulation solved by coarse-grid Native C-LBM with Boussinesq buoyancy

> Note: dataset generation, model training, and Python solver live in a separate core repository (`aerodynamics4mc`).

### Requirements
- Minecraft: `1.21.11`
- Fabric Loader: `>=0.18.2`
- Java: `21`
- Gradle: use project wrapper `./gradlew`

### Quick Start
```bash
cd fabric-mod
./gradlew runClient
```

### In-game Commands
- `/aero start`: start per-tick airflow updates
- `/aero stop`: stop updates and clear runtime state
- `/aero maxspeed <value>`: clamp max wind speed
- `/aero streamline <1|2|4|8>`: set streamline sampling stride
- `/aero streamline roi set <minX> <maxX> <minY> <maxY> <minZ> <maxZ>`: set local streamline ROI (window-local coords 0..127)
- `/aero streamline roi off`: disable ROI and render streamlines for the full window
- `/aero backend socket`: use TCP Python backend
- `/aero backend native`: use JNI native backend
- `/aero backend status`: print backend + native timing
- `/aero debug`: toggle debug rendering
- `/aero render background on|off|status`: toggle/query background circulation vector debug overlay
- `/aero render thermal on|off|status`: toggle/query thermal anomaly debug overlay

### Backend Modes
1. `socket` (default)
- Sends `(N,N,N,C)` payloads to the Python TCP backend.
- Best for fast CFD iteration and parameter tuning.

2. `native`
- Calls bundled native libraries via JNI (auto-extracted from mod jar at runtime).
- Platform-aware loading: `linux/windows/macos` + `x86_64/arm64` (Windows only `x86_64`).

### Low-resolution background circulation field
- The runtime maintains a low-resolution background wind field per dimension (roughly `16` blocks per cell by default), and automatically increases coarse cell size when active windows spread out.
- The field no longer uses hardcoded analytic circulation; it advances velocity on a coarse grid using the Native C-LBM solver.
- `thermal_anomaly` is sampled from world factors (block heat source/luminance, altitude, sunlight, biome, weather), then mapped to Boussinesq buoyancy forcing on the coarse solver.
- During window capture, state velocities are lightly relaxed toward this coarse background flow without a separate hardcoded per-voxel buoyancy impulse.

### Native Multi-platform Build & Packaging
See: `native/README.md`

Highlights:
- CMake superbuild supports one-shot generation of 4 native binaries
- Gradle `prepareNativeResources` packs `native/dist/natives/**` into the mod jar
- GitHub Actions workflow: `.github/workflows/native-matrix.yml`

### Common Build Commands
```bash
# Development build
./gradlew build

# Release remapped jar
./gradlew remapJar

# Resource processing only (includes native packing)
./gradlew processResources
```

### Key Directories
- `src/main/java/com/aerodynamics4mc/`: block, block entity, main entry
- `src/client/java/com/aerodynamics4mc/client/`: client airflow logic, physics, rendering, JNI bridge
- `native/`: JNI + native solver + CMake + multi-platform build config
- `.github/workflows/native-matrix.yml`: CI for 4-platform native build + packaging
