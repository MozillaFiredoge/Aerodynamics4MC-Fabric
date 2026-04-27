# Java/JNI 调用 DLL 使用说明

本文档面向需要在 Java 程序中调用 `aero_lbm.dll` / `libaero_lbm.so` / `libaero_lbm.dylib` 的合作者。

推荐优先使用 `aero_solver_capi.h` 中的 `aero_solver_*` C ABI。它是一个很薄的风洞求解接口，不依赖 Minecraft、Fabric、JNI 类名或 Create: Aeronautics 内部结构。

## 动态库文件

编译产物名称：

```text
Windows: aero_lbm.dll
Linux:   libaero_lbm.so
macOS:   libaero_lbm.dylib
```

Java 侧加载时，文件必须在以下任一位置：

```text
java.library.path
程序当前工作目录
手动指定的绝对路径
```

如果使用 `System.load(...)`，传入绝对路径：

```java
System.load("C:\\path\\to\\aero_lbm.dll");
```

如果使用 `System.loadLibrary(...)`，只传库名，不带前缀/后缀：

```java
System.loadLibrary("aero_lbm");
```

Windows 下如果 DLL 依赖 OpenCL runtime，需要目标机器已经安装显卡驱动/OpenCL runtime。

## 推荐方式：JNA 调用 C ABI

如果只是做验证工具，推荐用 JNA，不需要自己写 JNI glue code。

Gradle 依赖示例：

```kotlin
dependencies {
    implementation("net.java.dev.jna:jna:5.14.0")
}
```

Java 映射示例：

```java
import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.LongByReference;

import java.util.List;

public interface AeroSolverLibrary extends Library {
    AeroSolverLibrary INSTANCE = Native.load("aero_lbm", AeroSolverLibrary.class);

    int AERO_SOLVER_BOUNDARY_WIND_TUNNEL = 1;
    int AERO_SOLVER_FLOW_CHANNELS = 4;

    final class AeroGridDesc extends Structure {
        public int nx;
        public int ny;
        public int nz;
        public float dx;
        public float dt;

        @Override
        protected List<String> getFieldOrder() {
            return List.of("nx", "ny", "nz", "dx", "dt");
        }
    }

    final class AeroBoundaryDesc extends Structure {
        public int mode;
        public float inlet_vx;
        public float inlet_vy;
        public float inlet_vz;
        public float outlet_pressure;
        public float density;
        public float viscosity;

        @Override
        protected List<String> getFieldOrder() {
            return List.of(
                "mode",
                "inlet_vx",
                "inlet_vy",
                "inlet_vz",
                "outlet_pressure",
                "density",
                "viscosity"
            );
        }
    }

    void aero_solver_default_grid(AeroGridDesc outGrid);

    void aero_solver_default_boundary(AeroBoundaryDesc outBoundary);

    int aero_solver_create(
        int nx,
        int ny,
        int nz,
        float dx,
        float dt,
        LongByReference outHandle
    );

    int aero_solver_set_solid_mask(
        long handle,
        byte[] solidMask,
        int cellCount
    );

    int aero_solver_set_flow_state(
        long handle,
        float[] flow,
        int valueCount
    );

    int aero_solver_step_wind_tunnel(
        long handle,
        AeroBoundaryDesc boundary,
        int steps,
        float[] outFlow,
        int outValueCount
    );

    void aero_solver_destroy(long handle);

    Pointer aero_solver_last_error();

    Pointer aero_solver_runtime_info();
}
```

## 最小调用示例

```java
import com.sun.jna.Pointer;
import com.sun.jna.ptr.LongByReference;

public final class WindTunnelSmokeTest {
    public static void main(String[] args) {
        AeroSolverLibrary lib = AeroSolverLibrary.INSTANCE;

        int nx = 64;
        int ny = 64;
        int nz = 64;
        float dx = 1.0f;
        float dt = 0.05f;
        int cells = nx * ny * nz;

        LongByReference handleRef = new LongByReference();
        int created = lib.aero_solver_create(nx, ny, nz, dx, dt, handleRef);
        if (created == 0) {
            throw new IllegalStateException(errorString(lib));
        }

        long handle = handleRef.getValue();
        try {
            byte[] solidMask = new byte[cells];

            // 示例：在区域中心放一个 solid voxel
            int cx = nx / 2;
            int cy = ny / 2;
            int cz = nz / 2;
            solidMask[(cx * ny + cy) * nz + cz] = 1;

            if (lib.aero_solver_set_solid_mask(handle, solidMask, cells) == 0) {
                throw new IllegalStateException(errorString(lib));
            }

            AeroSolverLibrary.AeroBoundaryDesc boundary = new AeroSolverLibrary.AeroBoundaryDesc();
            lib.aero_solver_default_boundary(boundary);
            boundary.mode = AeroSolverLibrary.AERO_SOLVER_BOUNDARY_WIND_TUNNEL;
            boundary.inlet_vx = 5.0f;        // m/s
            boundary.inlet_vy = 0.0f;
            boundary.inlet_vz = 0.0f;
            boundary.outlet_pressure = 0.0f; // dimensionless pressure proxy
            boundary.density = 1.225f;
            boundary.viscosity = 1.5e-5f;
            boundary.write();

            float[] flowOut = new float[cells * AeroSolverLibrary.AERO_SOLVER_FLOW_CHANNELS];
            int steps = 256;
            int ok = lib.aero_solver_step_wind_tunnel(
                handle,
                boundary,
                steps,
                flowOut,
                flowOut.length
            );
            if (ok == 0) {
                throw new IllegalStateException(errorString(lib));
            }

            int sample = (cx * ny + cy) * nz + cz;
            int base = sample * 4;
            System.out.printf(
                "sample vx=%.4f vy=%.4f vz=%.4f p=%.6f%n",
                flowOut[base],
                flowOut[base + 1],
                flowOut[base + 2],
                flowOut[base + 3]
            );
        } finally {
            lib.aero_solver_destroy(handle);
        }
    }

    private static String errorString(AeroSolverLibrary lib) {
        Pointer ptr = lib.aero_solver_last_error();
        return ptr == null ? "unknown native solver error" : ptr.getString(0);
    }
}
```

## 输入/输出数组布局

体素索引：

```java
int cell = (x * ny + y) * nz + z;
```

`solidMask[cell]`：

```text
0 = 流体
非 0 = 固体障碍物
```

`flow[cell * 4 + channel]`：

```text
channel 0 = vx, m/s
channel 1 = vy, m/s
channel 2 = vz, m/s
channel 3 = pressure proxy，当前为 dimensionless rho_delta，不是 Pa
```

## 上一帧流场输入

如果需要用上一帧速度/压力场作为初始化，可以调用：

```java
lib.aero_solver_set_flow_state(handle, previousFlow, cells * 4);
```

注意：

- 这是初始化/重启输入，不建议每个小 step 都调用。
- 如果想连续推进 LBM 时间，应创建一次 handle，设置一次 solid mask，然后反复调用 `aero_solver_step_wind_tunnel`。
- 如果结构发生变化，应重新调用 `aero_solver_set_solid_mask`，内部会释放旧 LBM context，下次 step 重新初始化。

## 边界条件

当前暴露的 mode：

```text
1 = AERO_SOLVER_BOUNDARY_WIND_TUNNEL
2 = AERO_SOLVER_BOUNDARY_WIND_TUNNEL_PRESSURE_OUTLET
3 = AERO_SOLVER_BOUNDARY_CLOSED
4 = AERO_SOLVER_BOUNDARY_PERIODIC
```

推荐先用：

```java
boundary.mode = 1;
boundary.inlet_vx = 5.0f;
```

风洞模式含义：

```text
x-: velocity inlet
x+: convective outflow
y-/y+/z-/z+: symmetry / slip-like side boundary
```

## 内部边界条件处理说明

外部 API 只暴露简单的 `solidMask + boundary`，内部会转换成现有 LBM 求解器的 packet 和 benchmark boundary config。

### solid mask

`solidMask[cell] != 0` 的 voxel 会被写入内部 packet 的 obstacle channel。

内部求解时：

```text
solid cell 本身速度输出为 0
fluid cell 如果邻居是 solid，则在 fluid-solid link 上执行 bounce-back
```

也就是说，障碍物不是通过“强行把周围速度清零”实现的，而是通过 LBM streaming 阶段的反弹边界处理。

当前 MVP 仍是 voxel/block-aligned 边界：

```text
壁面位置近似在 fluid cell 和 solid cell 中间
```

这等价于简单 half-way bounce-back 语义，适合方块边界和风洞验证，但还不是曲面/SDF/interpolated bounce-back。

### 入口边界

`AERO_SOLVER_BOUNDARY_WIND_TUNNEL` 下，`x-` 面是 velocity inlet。

内部处理：

```text
boundary.inlet_vx/vy/vz
-> m/s 转 lattice velocity
-> 写入 benchmark config 的 x_min velocity
-> 出界 population 用 equilibrium distribution 重构
```

这属于速度 Dirichlet 风洞入口。

注意：

- 推荐让来流主要沿 `+x`。
- 如果要模拟不同攻角，优先由调用方把飞机模型旋转到局部风洞坐标系，而不是让入口速度在三个轴上任意变化。
- 入口速度过高时 LBM 会不稳定，MVP 先保持低马赫。

### 出口边界

默认 `AERO_SOLVER_BOUNDARY_WIND_TUNNEL` 下，`x+` 面是 convective outflow。

内部处理：

```text
从内部相邻 cell 外推 distribution
允许扰动向出口方向离开
```

这比封闭墙更适合风洞，但仍是简单出流近似，不是完整压力投影或工业级 non-reflecting outlet。

`AERO_SOLVER_BOUNDARY_WIND_TUNNEL_PRESSURE_OUTLET` 会把 `x+` 切换成 pressure Dirichlet：

```text
boundary.outlet_pressure
-> 内部 pressure proxy
-> 用 equilibrium distribution 重构出界 population
```

当前 pressure 是 `rho - 1` 的 dimensionless proxy，不是 Pa。

### 侧边界

风洞模式下，`y-/y+/z-/z+` 使用 symmetry / slip-like side boundary。

含义：

```text
侧面不作为实体墙
尽量减少侧壁剪切阻力
让区域更像开阔风洞
```

这适合测试飞机/方块结构绕流。如果需要管道/封闭盒，可以使用：

```text
AERO_SOLVER_BOUNDARY_CLOSED
```

该模式会把所有外边界设为 bounce-back wall。

### periodic 模式

`AERO_SOLVER_BOUNDARY_PERIODIC` 会把所有外边界设为 periodic。

该模式主要用于数值测试，不建议作为普通飞机风洞默认设置。

### 上一帧流场和初始条件

如果没有调用 `aero_solver_set_flow_state`，内部会用入口速度初始化整个 fluid 区域：

```text
fluid cell: inlet velocity
solid cell: zero velocity
```

如果调用了 `aero_solver_set_flow_state`，则：

```text
prev_flow 作为下一次 LBM context 初始化状态
solid cell 仍强制 zero velocity
```

之后连续调用 `aero_solver_step_wind_tunnel` 时，内部 LBM populations 会持续演化，不会每一步都从宏观场重建。

### 当前没有做的边界能力

MVP 暂时没有：

```text
curved wall / SDF wall distance
interpolated bounce-back
moving wall
rotating propeller boundary
inlet turbulence profile
non-reflecting farfield
force / moment integration
```

因此它适合做第一阶段风洞速度场验证，不适合直接宣称为高精度工业 CFD。

## 单位

Java API 输入/输出速度单位是 `m/s`。

内部 LBM 使用 lattice velocity：

```text
u_lattice = u_mps * dt / dx
u_mps = u_lattice * dx / dt
```

当前 pressure channel 是求解器内部的压力代理：

```text
pressure_proxy = rho - 1
```

它不是 Pa，不能直接当真实压力使用。

## 线程和生命周期

当前 MVP C API 内部用 mutex 串行化访问。

建议：

- 一个分析任务创建一个 solver handle。
- 同一个 handle 不要多线程同时调用。
- 用完必须调用 `aero_solver_destroy(handle)`。
- 当前底层 LBM runtime 是全局 grid config，暂时不要在同一进程里同时混用多个不同 grid size。

## 纯 JNI 方式

如果不想引入 JNA，也可以自己写 JNI wrapper。推荐 wrapper 只做很薄的一层：

```cpp
JNIEXPORT jlong JNICALL Java_xxx_NativeAero_create(JNIEnv*, jclass, jint nx, jint ny, jint nz, jfloat dx, jfloat dt) {
    long long handle = 0;
    return aero_solver_create(nx, ny, nz, dx, dt, &handle) ? (jlong) handle : 0;
}
```

然后分别包装：

```text
aero_solver_set_solid_mask
aero_solver_set_flow_state
aero_solver_step_wind_tunnel
aero_solver_destroy
aero_solver_last_error
```

不建议 Java 侧直接调用旧的 `aero_lbm_step_rect`，因为那需要理解内部 packet channel layout。对外验证请只使用 `aero_solver_*`。

## 常见错误

`UnsatisfiedLinkError`：

- DLL 不在 `java.library.path`。
- DLL 依赖的 OpenCL runtime 缺失。
- 32/64 位架构不匹配。
- Windows 下缺少 MSVC runtime。

输出全 0：

- `solidMask` 全部被设置成 solid。
- `steps` 太少。
- `inlet_vx/inlet_vy/inlet_vz` 都是 0。
- grid 太小，障碍物贴到边界。

速度发散或 NaN：

- `dt/dx` 导致 lattice velocity 过大。
- 入口速度过高。MVP 先保持低马赫，建议 `u_lattice < 0.1`。
- solid 几何太靠近 inlet/outlet。

可先用以下设置做 smoke test：

```text
nx=64, ny=64, nz=64
dx=1.0
dt=0.05
inlet_vx=2.0~5.0 m/s
steps=128~512
```

## Python DLL Benchmark

仓库里提供了一个最小 Python/ctypes benchmark 脚本：

```text
native/tools/benchmark_solver_dll.py
```

Windows 示例：

```powershell
py native\tools\benchmark_solver_dll.py `
  --dll path\to\aero_lbm.dll `
  --grid 128 `
  --steps-per-frame 1 `
  --frames 120 `
  --warmup 8
```

输出会包含：

```text
runtime 后端信息
avg/min/p50/p95/max ms per frame
LBM steps per second
NaN 检查
采样速度
native_timing(copy/solver/readback/total)
```

如果 `runtime` 以 `cpu|` 开头，说明没有跑 GPU，而是 CPU fallback。正常 GPU 路径应类似：

```text
opencl|cumulant-d3q27+sgs+bouss:<device name>
```

如果 `native_timing` 里 `copy` 或 `readback` 明显大于 `solver`，瓶颈在 CPU/GPU 数据传输，不在 kernel 计算。

如果要完整扫描 128³ 输出中的 NaN 和最大速度，可以加：

```powershell
--scan-output
```

注意：`--scan-output` 会额外消耗 CPU 时间，性能测试时默认只扫描前 4096 个 float。
