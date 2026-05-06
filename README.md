# Aerodynamics4MC - 实时风场与天气
![Banner](docs/banner.png)

[![Build Status](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/build/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![Native Matrix](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/native-matrix/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **Aerodynamics4MC** 为 Minecraft 带来了多尺度实时风场与天气系统——服务端负责大尺度气象，客户端可选开启高分辨率气流可视化，其他模组则通过统一的公共 API 获取风场数据，而无需接触内部解算器。

---

## 🌬 简介

Aerodynamics4MC 提供了从全球环流到局部湍流的多层风场模拟。风暴、气旋、龙卷风自然生成并演变，地形对风的影响也被精细计算。无论是驱动飞行器、吹散烟雾粒子，还是为工业模组提供风力资源，它都能提供可靠且高性能的风数据。

---

## ✨ 核心特色

- **多尺度风系统** – 从行星波到建筑周围米级湍流，四层嵌套网格协同工作。
- **服务端权威天气** – 气旋、对流簇、龙卷风等由服务端驱动，保证多人游戏一致。
- **可选客户端高分辨率气流** – 客户端本地运行 LBM 解算器，呈现细腻的烟尘、粒子飘移。
- **公共 API** – 提供 `SERVER_COARSE_ONLY`, `CLIENT_LOCAL_PREFERRED` 等多种采样策略，方便其他模组集成。
- **原生加速** – C++ 编写的格子玻尔兹曼（LBM）核心，支持 Windows / Linux / macOS，支持 OpenCL GPU 加速。
- **轻量玩法组件** – 风扇、风速计等，让风成为可玩的物理元素。

---

## 🖼 效果展示

![效果图1：L0](docs/l0_minecraft_overworld_tick2946.png)

![效果图2：L1](docs/l1_minecraft_overworld_tick2946.png)

![效果图3：风扇](docs/fan.gif)
---

## 📦 安装

| 需求          | 版本                              |
|---------------|-----------------------------------|
| Minecraft     | 1.21.11                           |
| Fabric Loader | 0.18.4                            |
| Fabric API    | 0.141.2+1.21.11                   |
| Java          | 21+                               |

1. 下载模组 `.jar` 文件（可从 [Releases](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/releases) 或 CurseForge/Modrinth 获取）。
2. 放入 `.minecraft/mods/` 文件夹。
3. 确保已安装对应版本的 Fabric API。

---

## 🌐 风场系统速览

整个模组的风场由 **四层** 嵌套网格构成：

| 层级   | Java 类                         | 分辨率                       | 职责                                       | 可信度            |
|--------|---------------------------------|------------------------------|--------------------------------------------|-------------------|
| 驱动层 | `runtime.WorldScaleDriver`      | 384×384 气压单元（每单元 256b） | 气旋、对流簇、龙卷风、行星波               | 服务端权威        |
| **L0** | `runtime.BackgroundMetGrid`     | 41×41 单元 × 256 b/单元       | 全球气压、地转风、地形阻力与粗糙度         | 服务端权威        |
| **L1** | `runtime.MesoscaleGrid`         | 33×33×8 单元（64×64×40 b/单元） | 随地形风场、大气边界层切变、湍流诊断       | 服务端权威        |
| **L2** | `client.ClientL2Solver`（默认） | 客户端 32³ 砖块 | D3Q27 cumulant LBM + 亚格子模型 + Boussinesq | **客户端本地**    |

服务端 L2 默认关闭（`SERVER_AUTHORITATIVE_L2_ENABLED = false`），客户端 L2 用于可视化与粒子效果，**不可**用于游戏平衡判定。

---

## 🧩 游戏内组件

| 组件       | 说明                                                     |
|------------|----------------------------------------------------------|
| **风扇**   | 添加局部动量，用于管道气流实验或通风测试。               |
| **管道**   | 塑造气流路径。                                           |
| **风速计** | 右键点击可查看当前位置的 `GameplayWindSample`，包括有效风、阵风、湍流、遮蔽与置信度。 |
| **风力涡轮探针** | 使用 `GameplayWindSample` 计算示例功率，并输出 0-15 红石信号。 |
| **地形与方块** | 自动向 L1 层提供世界障碍信息，影响 L2 局部风场。    |
| **粒子 / Iris 桥接** | 通过公共 API 采样风，驱动树叶摆动、烟雾飘散等视觉效果。（需要搭配修改过的shaderpack进行使用） |

---

## 🕹 指令


服务端风场运行时会随服务器启动自动开启，不再需要手动执行启动/停止命令。

| 指令                           | 用途                                         |
|--------------------------------|----------------------------------------------|
| `/aero status`                 | 显示服务端运行时、L0/L1、协调器、原生库状态  |
| `/aero render`                 | 查看当前客户端渲染模式                       |
| `/aero render vectors on/off`  | 切换速度矢量渲染                             |
| `/aero render streamlines on/off` | 切换流线渲染                             |
| `/aero dumpdata`               | 导出运行时诊断数据(诊断功能)                           |
| `/aero dump_l1`                | 导出 L1 中尺度快照（诊断功能）                           |

*客户端指令：*

| 指令                     | 用途                                 |
|--------------------------|--------------------------------------|
| `/aero_client_l2`        | 查看客户端 L2 状态                   |
| `/aero_client_l2 on/off` | 开启/关闭实验性客户端本地 L2 解算器  |

---

## 🎨 可视化

| 模式       | 说明                                                         |
|------------|--------------------------------------------------------------|
| **矢量场** | 每个采样单元绘制一条线段，方向=速度，长度与颜色表示风速大小 |
| **流线**   | 通过种子点积分追踪流线，颜色反映速度                         |

默认使用 Viridis 色系的 CFD 风格线条渲染，旧式箭头样式仅作保留。

---

## 🔌 风采样 API（面向模组开发者）

其他模组可通过 `com.aerodynamics4mc.api` 获取风数据，无需依赖内部网格类或数据包格式。

**服务端采样：**

```java
import com.aerodynamics4mc.api.AeroWindApi;
import com.aerodynamics4mc.api.GameplayWindSample;
import com.aerodynamics4mc.api.SamplePolicy;

GameplayWindSample wind = AeroWindApi.sampleGameplay(player, position, SamplePolicy.GAMEPLAY_SERVER_ONLY);

if (wind.isTrustedForGameplay()) {
    Vec3d mean = wind.meanVelocity();
    Vec3d effective = wind.effectiveVelocity();
    float turbulence = wind.turbulenceIntensity();
}
```

**客户端采样（用于视觉）：**

```java
import com.aerodynamics4mc.api.AeroClientWindApi;

AeroWindSample sample = AeroClientWindApi.sample(
    clientWorld, position, SamplePolicy.CLIENT_LOCAL_PREFERRED);
Vec3d visualDrift = sample.effectiveVelocity();
```

**建议的集成策略：**

- 飞行器、船舶、涡轮机等权威玩法 → `AeroWindApi.sampleGameplay(...)`，配合 `isTrustedForGameplay()` 校验。
- 内置示例 → 放置 **风力涡轮探针**，观察 `GameplayWindSample` 如何转成红石信号。
- 客户端粒子（烟、火把、飘落树叶等） → `SamplePolicy.CLIENT_LOCAL_PREFERRED`。
- 工程可视化和调试 → `AeroWindApi.sample(...)` + `SamplePolicy.VISUAL_LOCAL_FIRST`。

完整 API 契约请参阅：[`docs/wind-sampling-api.md`](docs/wind-sampling-api.md)

---

## 🔧 构建与原生库

如果你想从源码构建模组：

```bash
cd fabric-mod/
./gradlew runClient        # 启动开发客户端
./gradlew build            # 构建 jar
./gradlew remapJar         # 生成发布版 jar
```

原生 LBM 库手工构建：

```bash
cd native
cmake -S . -B build
cmake --build build -j
```

多平台构建细节见 [`native/README.md`](native/README.md)。  
若不构建原生库，模组会尝试使用内嵌的预编译库。

---

## ⚠️ 已知边界

| 场景                 | 现状                                                       |
|----------------------|------------------------------------------------------------|
| 近音速飞行器         | 游戏内求解器不支持                                         |
| 动态螺旋桨几何       | 不在公共风洞 API 内                                        |
| 多人 L2 反馈         | 尚需未来验证与聚合设计                                     |
| 服务端权威 L2        | 非默认路径，诊断用途保留                                   |
| 体素到翼型设计       | 超出运行时 API 范围                                        |

对于 Create:Aeronautics 式的集成，待项目迁移到1.21.1 NeoForge后，推荐使用 `SERVER_COARSE_ONLY` 获取世界风，另行处理载具相对风速与翼型气动系数。

---

## 📚 文档地图

**风场系统（从这里开始）：**

| 文档                                                     | 说明                                     |
|----------------------------------------------------------|------------------------------------------|
| 📘 [`docs/wind-system-overview.md`](docs/wind-system-overview.md) | **权威风系统总览**（必读）              |
| [`docs/release-roadmap.md`](docs/release-roadmap.md) | 正式发布路线图与版本边界                  |
| [`docs/wind-sampling-api.md`](docs/wind-sampling-api.md) | 公共采样 API 契约                        |
| [`docs/world-scale-weather-design.md`](docs/world-scale-weather-design.md) | 驱动层天气现象学设计（旧命名）           |
| [`docs/wind-shear-weather-roadmap.md`](docs/wind-shear-weather-roadmap.md) | 大气边界层/风切变路线图                  |
| [`docs/player-facing-wind-design.md`](docs/player-facing-wind-design.md) | 面向玩家的设计理念                       |

**原生求解器：**

| 文档                                                            | 说明                 |
|-----------------------------------------------------------------|----------------------|
| [`native/README.md`](native/README.md)                          | 原生库构建细节       |
| [`docs/native-jni-interface-reference.md`](docs/native-jni-interface-reference.md) | JNI / 通道布局参考   |
| [`docs/native-physics-engine-todo.md`](docs/native-physics-engine-todo.md) | 原生求解器待办工作   |
| [`native/docs/wind_tunnel_solver_api.md`](native/docs/wind_tunnel_solver_api.md) | 独立 C ABI 风洞接口 |

**集成与遗留文档：**

| 文档                                                                   | 说明                    |
|------------------------------------------------------------------------|-------------------------|
| [`docs/shaderpack-wind-compat-design.md`](docs/shaderpack-wind-compat-design.md) | Iris / BSL 桥接设计     |
| [`docs/on-demand-l2-prefetch-design.md`](docs/on-demand-l2-prefetch-design.md) | 砖块预取设计（远期）    |
| [`docs/local-air-patch-design.md`](docs/local-air-patch-design.md)     | 旧版局部空气补丁（弃用）|
| [`docs/phase2-completion-log.md`](docs/phase2-completion-log.md)等     | 历史里程碑              |

---

## 📄 许可证

采用 MIT 许可证。详情见仓库 LICENSE 文件。

---

---

# Aerodynamics4MC - Real-time Wind & Weather (English)

[![Build Status](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/build/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![Native Matrix](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/native-matrix/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **Aerodynamics4MC** brings a multi‑scale, real‑time wind and weather system to Minecraft. Coarse weather is server‑authoritative, an optional high‑resolution CFD layer runs on the client for visualization, and external mods consume wind through a clean public API without touching internal solver buffers.

---

## 🌬 Introduction

Aerodynamics4MC simulates everything from planetary‑scale pressure systems to block‑level gusts. Cyclones, tornadoes, and convective storms emerge naturally; terrain shape and roughness modulate airflow. Whether you’re building airships, blowing smoke particles, or harvesting wind energy, this mod delivers consistent, high‑performance wind data.

---

## ✨ Highlights

- **Multi‑scale wind system** – four nested grids from planetary waves down to metre‑scale turbulence.
- **Server‑authoritative weather** – cyclones, convective clusters, and tornadoes are driven by the server for consistent multiplayer.
- **Optional client high‑res aerodynamics** – a local LBM solver runs on the client for stunning smoke, torch and leaves particle drift.
- **Public API** – multiple sampling policies (`SERVER_COARSE_ONLY`, `CLIENT_LOCAL_PREFERRED`, etc.) make integration easy.
- **Native acceleration** – C++‑based LBM core, optional OpenCL CPU fallback(GPU by default), pre‑built binaries for Windows, Linux, and macOS.
- **Lightweight gameplay blocks** – fan, duct, and wind meter let you experiment with airflow.

---

## 🖼 Gallery

![showcase 1：L0](docs/l0_minecraft_overworld_tick2946.png)

![showcase 2：L1](docs/l1_minecraft_overworld_tick2946.png)

![showcase 3：风扇](docs/fan.gif)

---

## 📦 Installation

| Requirement    | Version                           |
|----------------|-----------------------------------|
| Minecraft      | 1.21.11                           |
| Fabric Loader  | 0.18.4                            |
| Fabric API     | 0.141.2+1.21.11                   |
| Java           | 21+                               |

1. Download the mod `.jar` from [Releases](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/releases) or CurseForge/Modrinth.
2. Place it in `.minecraft/mods/`.
3. Ensure the required Fabric API is installed.

---

## 🌐 Wind System Overview

The mod’s wind field is computed by four nested grids:

| Layer  | Java Class                      | Resolution                        | Role                                                    | Trust              |
|--------|---------------------------------|-----------------------------------|---------------------------------------------------------|--------------------|
| Driver | `runtime.WorldScaleDriver`      | 384×384 pressure cells (256 b/cell) | Cyclones, convection, tornadoes, planetary waves      | Server‑authoritative |
| **L0** | `runtime.BackgroundMetGrid`     | 41×41 cells × 256 b/cell          | Global pressure, geostrophic wind, terrain drag        | Server‑authoritative |
| **L1** | `runtime.MesoscaleGrid`         | 33×33×8 cells (64×64×40 b/cell)   | Terrain‑following wind, ABL shear, turbulence          | Server‑authoritative |
| **L2** | `client.ClientL2Solver` (default)| Client 32³ bricks; | D3Q27 cumulant LBM + SGS + Boussinesq                | **Client‑local**   |

Server‑side L2 is disabled by default (`SERVER_AUTHORITATIVE_L2_ENABLED = false`). Client L2 is for visual effects only — never use it for gameplay balance.

---

## 🧩 In‑Game Components

| Component          | Description                                                       |
|--------------------|-------------------------------------------------------------------|
| **Fan**            | Adds local momentum for duct/ventilation experiments.            |
| **Duct**           | Shapes local airflow paths.                                      |
| **Wind Meter**     | Right‑click to inspect `GameplayWindSample`: effective wind, gust, turbulence, shelter, and confidence. |
| **Wind Turbine Probe** | Converts `GameplayWindSample` into example power and a 0-15 redstone signal. |
| **Terrain & Blocks** | Feed world obstacles into L1, modulating L2 forcing.           |
| **Particles / Iris bridge** | Sample wind through the public API for visual effects. (Modified compatible shaderpacks needed)  |

---

## 🕹 Commands

*Server commands require permission level 2+.*

The server wind runtime starts automatically with the server. Manual runtime start/stop commands are no longer needed.

| Command                        | Purpose                                      |
|--------------------------------|----------------------------------------------|
| `/aero status`                 | Show runtime, L0/L1, coordinator, native status. |
| `/aero render`                 | Print client render mode status.             |
| `/aero render vectors on/off`  | Toggle velocity‑vector rendering.            |
| `/aero render streamlines on/off` | Toggle streamline rendering.             |
| `/aero dumpdata`               | Dump runtime diagnostics.(Developer)                    |
| `/aero dump_l1`                | Dump L1 mesoscale snapshot.(Developer)                  |

*Client commands:*

| Command                      | Purpose                                   |
|------------------------------|-------------------------------------------|
| `/aero_client_l2`            | Print client L2 status.                   |
| `/aero_client_l2 on/off`     | Enable/disable experimental client‑local L2 solver. |

---

## 🎨 Visualization

| Mode        | Description                                                      |
|-------------|------------------------------------------------------------------|
| **Vectors** | One line segment per sampled cell; direction = velocity, colour & length encode speed. |
| **Streamlines** | Seeded integration through the field, coloured by speed.     |

The preferred rendering uses a CFD‑style viridis colour map; the old glyph style is legacy.

---

## 🔌 Wind Sampling API (for Mod Developers)

Other mods consume wind through `com.aerodynamics4mc.api`. No internal grid classes or packet formats are required.

**Server‑side sampling:**

```java
import com.aerodynamics4mc.api.AeroWindApi;
import com.aerodynamics4mc.api.GameplayWindSample;
import com.aerodynamics4mc.api.SamplePolicy;

GameplayWindSample wind = AeroWindApi.sampleGameplay(player, position, SamplePolicy.GAMEPLAY_SERVER_ONLY);

if (wind.isTrustedForGameplay()) {
    Vec3d mean = wind.meanVelocity();
    Vec3d effective = wind.effectiveVelocity();
    float turbulence = wind.turbulenceIntensity();
}
```

**Client‑side sampling (visuals):**

```java
import com.aerodynamics4mc.api.AeroClientWindApi;

AeroWindSample sample = AeroClientWindApi.sample(
    clientWorld, position, SamplePolicy.CLIENT_LOCAL_PREFERRED);
Vec3d visualDrift = sample.effectiveVelocity();
```

**Recommended integration patterns:**

- Aircraft, airships, turbines, gameplay logic → `AeroWindApi.sampleGameplay(...)` + `isTrustedForGameplay()` check.
- Built-in example → place a **Wind Turbine Probe** to see `GameplayWindSample` become redstone output.
- Client particles (smoke, steam, dust) → `SamplePolicy.CLIENT_LOCAL_PREFERRED`.
- Engineering overlays and diagnostics → `AeroWindApi.sample(...)` + `SamplePolicy.VISUAL_LOCAL_FIRST`.

Full API contract: [`docs/wind-sampling-api.md`](docs/wind-sampling-api.md)

---

## 🔧 Building & Native Code

To build from source:

```bash
cd fabric-mod/
./gradlew runClient        # launch dev client
./gradlew build            # build jar
./gradlew remapJar         # distributable jar
```

Manual native library build:

```bash
cd native
cmake -S . -B build
cmake --build build -j
```

See [`native/README.md`](native/README.md) for cross‑platform superbuild details. If you don’t build the native library, the mod will attempt to use an embedded pre‑built binary.

---

## ⚠️ Known Boundaries

| Scenario                       | Current State                                          |
|--------------------------------|--------------------------------------------------------|
| Near‑sonic aircraft            | Not supported by the in‑game solver.                   |
| Dynamic propeller geometry     | Not in the public wind‑tunnel API.                     |
| Multiplayer L2 feedback        | Requires future validation/aggregation design.         |
| Server‑authoritative L2        | Not the default path; diagnostic use only.             |
| Voxel‑to‑airfoil design        | Out of scope for the runtime API.                      |

For Create:Aeronautics‑style integration, use `SERVER_COARSE_ONLY` for world wind and a separate vehicle‑relative aerodynamic model for wing/propeller coefficients.

---

## 📚 Documentation Map

**Wind system (start here):**

| Document                                                              | Purpose                                    |
|-----------------------------------------------------------------------|--------------------------------------------|
| 📘 [`docs/wind-system-overview.md`](docs/wind-system-overview.md)     | **Authoritative wind‑system overview**     |
| [`docs/release-roadmap.md`](docs/release-roadmap.md)                  | Release roadmap and version boundaries     |
| [`docs/wind-sampling-api.md`](docs/wind-sampling-api.md)             | Public sampling API contract               |
| [`docs/world-scale-weather-design.md`](docs/world-scale-weather-design.md) | Driver phenomenology (older naming)   |
| [`docs/wind-shear-weather-roadmap.md`](docs/wind-shear-weather-roadmap.md) | ABL / wind‑shear roadmap             |
| [`docs/player-facing-wind-design.md`](docs/player-facing-wind-design.md) | Product philosophy                     |

**Native solver:**

| Document                                                                   | Purpose                      |
|----------------------------------------------------------------------------|------------------------------|
| [`native/README.md`](native/README.md)                                     | Native build details         |
| [`docs/native-jni-interface-reference.md`](docs/native-jni-interface-reference.md) | JNI / channel‑layout ref |
| [`docs/native-physics-engine-todo.md`](docs/native-physics-engine-todo.md) | Native solver to‑do          |
| [`native/docs/wind_tunnel_solver_api.md`](native/docs/wind_tunnel_solver_api.md) | Standalone C ABI        |

**Integration & legacy:**

| Document                                                                         | Purpose                        |
|----------------------------------------------------------------------------------|--------------------------------|
| [`docs/shaderpack-wind-compat-design.md`](docs/shaderpack-wind-compat-design.md) | Iris / BSL bridge design       |
| [`docs/on-demand-l2-prefetch-design.md`](docs/on-demand-l2-prefetch-design.md)   | Brick‑prefetch (aspirational)  |
| [`docs/local-air-patch-design.md`](docs/local-air-patch-design.md)               | Legacy patch concept (deprecated) |
| [`docs/phase2-completion-log.md`](docs/phase2-completion-log.md) etc.            | Historical milestones          |

---

## 📄 License

MIT. See repository license files for details.
