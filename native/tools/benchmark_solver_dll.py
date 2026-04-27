#!/usr/bin/env python3
"""Benchmark the external aero_solver C API from a built native DLL.

Example on Windows:

    py native\\tools\\benchmark_solver_dll.py --dll path\\to\\aero_lbm.dll --grid 128 --steps-per-frame 1 --frames 120

The script uses ctypes only. It does not require Minecraft, Fabric, Java, or JNI.
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
from pathlib import Path
from time import perf_counter


AERO_SOLVER_BOUNDARY_WIND_TUNNEL = 1
AERO_SOLVER_FLOW_CHANNELS = 4


class AeroBoundaryDesc(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_int),
        ("inlet_vx", ctypes.c_float),
        ("inlet_vy", ctypes.c_float),
        ("inlet_vz", ctypes.c_float),
        ("outlet_pressure", ctypes.c_float),
        ("density", ctypes.c_float),
        ("viscosity", ctypes.c_float),
    ]


def load_library(path: Path) -> ctypes.CDLL:
    if not path.exists():
        raise FileNotFoundError(f"DLL not found: {path}")
    if os.name == "nt":
        return ctypes.WinDLL(str(path))
    return ctypes.CDLL(str(path))


def configure_api(lib: ctypes.CDLL) -> None:
    lib.aero_solver_create.argtypes = [
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.POINTER(ctypes.c_longlong),
    ]
    lib.aero_solver_create.restype = ctypes.c_int

    lib.aero_solver_set_solid_mask.argtypes = [
        ctypes.c_longlong,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int,
    ]
    lib.aero_solver_set_solid_mask.restype = ctypes.c_int

    lib.aero_solver_set_flow_state.argtypes = [
        ctypes.c_longlong,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.aero_solver_set_flow_state.restype = ctypes.c_int

    lib.aero_solver_step_wind_tunnel.argtypes = [
        ctypes.c_longlong,
        ctypes.POINTER(AeroBoundaryDesc),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.aero_solver_step_wind_tunnel.restype = ctypes.c_int

    lib.aero_solver_destroy.argtypes = [ctypes.c_longlong]
    lib.aero_solver_destroy.restype = None

    lib.aero_solver_last_error.argtypes = []
    lib.aero_solver_last_error.restype = ctypes.c_char_p

    lib.aero_solver_runtime_info.argtypes = []
    lib.aero_solver_runtime_info.restype = ctypes.c_char_p

    try:
        lib.aero_lbm_reset_timing.argtypes = []
        lib.aero_lbm_reset_timing.restype = None
        lib.aero_lbm_timing_info.argtypes = []
        lib.aero_lbm_timing_info.restype = ctypes.c_char_p
    except AttributeError:
        pass


def native_error(lib: ctypes.CDLL) -> str:
    ptr = lib.aero_solver_last_error()
    if not ptr:
        return "unknown native error"
    return ptr.decode("utf-8", errors="replace")


def runtime_info(lib: ctypes.CDLL) -> str:
    ptr = lib.aero_solver_runtime_info()
    if not ptr:
        return "unknown runtime"
    return ptr.decode("utf-8", errors="replace")


def reset_native_timing(lib: ctypes.CDLL) -> None:
    reset = getattr(lib, "aero_lbm_reset_timing", None)
    if reset is not None:
        reset()


def native_timing_info(lib: ctypes.CDLL) -> str:
    timing = getattr(lib, "aero_lbm_timing_info", None)
    if timing is None:
        return "native timing unavailable"
    ptr = timing()
    if not ptr:
        return "native timing unavailable"
    return ptr.decode("utf-8", errors="replace")


def cell_index(x: int, y: int, z: int, ny: int, nz: int) -> int:
    return (x * ny + y) * nz + z


def build_obstacle_mask(
    nx: int,
    ny: int,
    nz: int,
    obstacle: str,
    radius_ratio: float,
) -> ctypes.Array[ctypes.c_uint8]:
    cells = nx * ny * nz
    mask = (ctypes.c_uint8 * cells)()
    if obstacle == "none":
        return mask

    cx = nx // 3
    cy = ny // 2
    cz = nz // 2
    radius = max(1.0, min(nx, ny, nz) * radius_ratio)

    if obstacle == "cube":
        half = max(1, int(round(radius)))
        for x in range(max(0, cx - half), min(nx, cx + half + 1)):
            for y in range(max(0, cy - half), min(ny, cy + half + 1)):
                for z in range(max(0, cz - half), min(nz, cz + half + 1)):
                    mask[cell_index(x, y, z, ny, nz)] = 1
        return mask

    if obstacle == "sphere":
        r2 = radius * radius
        xmin = max(0, int(math.floor(cx - radius)))
        xmax = min(nx - 1, int(math.ceil(cx + radius)))
        ymin = max(0, int(math.floor(cy - radius)))
        ymax = min(ny - 1, int(math.ceil(cy + radius)))
        zmin = max(0, int(math.floor(cz - radius)))
        zmax = min(nz - 1, int(math.ceil(cz + radius)))
        for x in range(xmin, xmax + 1):
            for y in range(ymin, ymax + 1):
                for z in range(zmin, zmax + 1):
                    dx = x - cx
                    dy = y - cy
                    dz = z - cz
                    if dx * dx + dy * dy + dz * dz <= r2:
                        mask[cell_index(x, y, z, ny, nz)] = 1
        return mask

    if obstacle == "cylinder":
        r2 = radius * radius
        for x in range(max(0, cx - 1), min(nx, cx + 2)):
            for y in range(ny):
                dy = y - cy
                for z in range(nz):
                    dz = z - cz
                    if dy * dy + dz * dz <= r2:
                        mask[cell_index(x, y, z, ny, nz)] = 1
        return mask

    raise ValueError(f"unknown obstacle: {obstacle}")


def validate_output(flow: ctypes.Array[ctypes.c_float], max_scan_values: int) -> tuple[float, int]:
    max_speed = 0.0
    non_finite = 0
    value_count = min(len(flow), max_scan_values)
    cell_count = value_count // AERO_SOLVER_FLOW_CHANNELS
    for cell in range(cell_count):
        base = cell * AERO_SOLVER_FLOW_CHANNELS
        vx = float(flow[base])
        vy = float(flow[base + 1])
        vz = float(flow[base + 2])
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz) and math.isfinite(float(flow[base + 3]))):
            non_finite += 1
            continue
        max_speed = max(max_speed, math.sqrt(vx * vx + vy * vy + vz * vz))
    return max_speed, non_finite


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark aero_lbm.dll wind-tunnel solver API")
    parser.add_argument("--dll", required=True, type=Path, help="Path to aero_lbm.dll / libaero_lbm.so")
    parser.add_argument("--grid", type=int, default=128, help="Cubic grid resolution, e.g. 128")
    parser.add_argument("--nx", type=int, default=0, help="Override x resolution")
    parser.add_argument("--ny", type=int, default=0, help="Override y resolution")
    parser.add_argument("--nz", type=int, default=0, help="Override z resolution")
    parser.add_argument("--dx", type=float, default=1.0, help="Cell size in meters")
    parser.add_argument("--dt", type=float, default=0.05, help="Time step in seconds")
    parser.add_argument("--velocity", type=float, default=2.0, help="Inlet x velocity in m/s")
    parser.add_argument("--steps-per-frame", type=int, default=1, help="LBM steps per measured frame")
    parser.add_argument("--frames", type=int, default=120, help="Measured frames")
    parser.add_argument("--warmup", type=int, default=8, help="Warmup frames before measurement")
    parser.add_argument(
        "--obstacle",
        choices=("none", "cube", "sphere", "cylinder"),
        default="sphere",
        help="Simple obstacle inserted near the inlet",
    )
    parser.add_argument("--obstacle-radius-ratio", type=float, default=0.08, help="Obstacle radius as fraction of grid")
    parser.add_argument("--scan-output", action="store_true", help="Scan all output values for NaN and max speed each frame")
    args = parser.parse_args()

    nx = args.nx or args.grid
    ny = args.ny or args.grid
    nz = args.nz or args.grid
    if min(nx, ny, nz) <= 0:
        raise SystemExit("grid dimensions must be positive")
    if args.steps_per_frame <= 0 or args.frames <= 0 or args.warmup < 0:
        raise SystemExit("steps-per-frame/frames/warmup must be positive")

    cells = nx * ny * nz
    value_count = cells * AERO_SOLVER_FLOW_CHANNELS
    print(f"[bench] dll={args.dll}")
    print(f"[bench] grid={nx}x{ny}x{nz} cells={cells:,} flow_values={value_count:,}")
    print(f"[bench] dx={args.dx:g}m dt={args.dt:g}s inlet_vx={args.velocity:g}m/s steps/frame={args.steps_per_frame}")

    lib = load_library(args.dll)
    configure_api(lib)

    handle = ctypes.c_longlong()
    if not lib.aero_solver_create(nx, ny, nz, ctypes.c_float(args.dx), ctypes.c_float(args.dt), ctypes.byref(handle)):
        raise SystemExit(f"create failed: {native_error(lib)}")
    print(f"[bench] runtime={runtime_info(lib)}")

    try:
        solid = build_obstacle_mask(nx, ny, nz, args.obstacle, args.obstacle_radius_ratio)
        if not lib.aero_solver_set_solid_mask(handle.value, solid, cells):
            raise SystemExit(f"set_solid_mask failed: {native_error(lib)}")

        boundary = AeroBoundaryDesc(
            AERO_SOLVER_BOUNDARY_WIND_TUNNEL,
            ctypes.c_float(args.velocity),
            ctypes.c_float(0.0),
            ctypes.c_float(0.0),
            ctypes.c_float(0.0),
            ctypes.c_float(1.225),
            ctypes.c_float(1.5e-5),
        )
        out_flow = (ctypes.c_float * value_count)()

        print(f"[bench] warmup_frames={args.warmup}")
        for _ in range(args.warmup):
            ok = lib.aero_solver_step_wind_tunnel(
                handle.value,
                ctypes.byref(boundary),
                args.steps_per_frame,
                out_flow,
                value_count,
            )
            if not ok:
                raise SystemExit(f"warmup step failed: {native_error(lib)}")

        reset_native_timing(lib)
        print(f"[bench] measured_frames={args.frames}")
        times_ms: list[float] = []
        max_speed = 0.0
        non_finite = 0
        scan_values = value_count if args.scan_output else min(value_count, 4096)
        for frame in range(args.frames):
            start = perf_counter()
            ok = lib.aero_solver_step_wind_tunnel(
                handle.value,
                ctypes.byref(boundary),
                args.steps_per_frame,
                out_flow,
                value_count,
            )
            elapsed_ms = (perf_counter() - start) * 1000.0
            if not ok:
                raise SystemExit(f"frame {frame} step failed: {native_error(lib)}")
            times_ms.append(elapsed_ms)
            frame_max, frame_bad = validate_output(out_flow, scan_values)
            max_speed = max(max_speed, frame_max)
            non_finite += frame_bad

        avg_ms = sum(times_ms) / len(times_ms)
        print("[bench] result")
        print(f"  avg_ms_per_frame={avg_ms:.3f}")
        print(f"  min_ms={min(times_ms):.3f}")
        print(f"  p50_ms={percentile(times_ms, 0.50):.3f}")
        print(f"  p95_ms={percentile(times_ms, 0.95):.3f}")
        print(f"  max_ms={max(times_ms):.3f}")
        print(f"  avg_lbm_steps_per_second={args.steps_per_frame * 1000.0 / avg_ms:.2f}")
        print(f"  max_speed_scanned_mps={max_speed:.6f}")
        print(f"  non_finite_values_scanned={non_finite}")
        print(f"  first_cell=({out_flow[0]:.6f}, {out_flow[1]:.6f}, {out_flow[2]:.6f}, {out_flow[3]:.6f})")
        print(f"  native_timing={native_timing_info(lib)}")
    finally:
        lib.aero_solver_destroy(handle.value)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
