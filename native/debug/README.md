# Native LBM Debug Harness

该目录用于对 `native/src/aero_lbm_jni.cpp` 的核心求解流程做可重复的数值检查，重点覆盖：

- 守恒与稳定性（质量、速度、压力漂移）
- 固体边界无滑移（obstacle 内速度应接近 0）
- 风扇驱动注入是否生效（近场速度提升、正向动量增长）

## 文件说明

- `java/com/aerodynamics4mc/client/NativeLbmDebugHarness.java`
  - 直接调用 `NativeLbmBridge` 的 JNI 接口，走与 Mod 运行一致的 native 路径。
- `run_debug_checks.sh`
  - 自动选择当前平台 native 动态库，编译并运行 debug harness。

## 运行方式

在 `fabric-mod/` 下执行：

```bash
./native/debug/run_debug_checks.sh --grid 16 --steps 48
```

可选参数：

- `--grid`：网格尺寸（默认 `16`）
- `--steps`：每个场景 rollout 步数（默认 `48`）

## 当前测试项

- `equilibrium_invariant`
  - 初始全零状态下，最大速度/平均压力应保持极小，质量漂移应接近 0。
- `mass_conservation`
  - 无风扇、无障碍、随机小速度扰动下，跟踪质量相对漂移。
- `obstacle_no_slip`
  - 中央障碍体内速度应维持接近 0。
- `fan_injection`
  - 风扇附近截面速度明显高于远场，且 `x` 向总动量为正。

## 结果判读

- 全部 `PASS`：说明当前参数与实现下，基本物理约束成立，可作为回归基线。
- 任一 `FAIL`：优先检查最近对以下模块的修改：
  - 碰撞（TRT/MRT/BGK）参数
  - 流动与反弹边界逻辑
  - 输入 payload 通道映射（`obstacle/fan/ref state`）
