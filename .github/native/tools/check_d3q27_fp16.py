#!/usr/bin/env python3
"""Small correctness checks for the experimental D3Q27 FP16 path.

This is intentionally not a CFD validation suite. It catches the easy mistakes
that previously hid behind good benchmark numbers: parity flips, boundary
reflections, non-finite output, and non-zero solid output.
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
from pathlib import Path


AERO_SOLVER_BOUNDARY_WIND_TUNNEL = 1
FLOW_CHANNELS = 4


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


def cell_index(x: int, y: int, z: int, ny: int, nz: int) -> int:
    return (x * ny + y) * nz + z


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
    lib.aero_lbm_timing_info.argtypes = []
    lib.aero_lbm_timing_info.restype = ctypes.c_char_p


def native_error(lib: ctypes.CDLL) -> str:
    ptr = lib.aero_solver_last_error()
    return ptr.decode("utf-8", errors="replace") if ptr else "unknown native error"


def runtime_info(lib: ctypes.CDLL) -> str:
    ptr = lib.aero_solver_runtime_info()
    return ptr.decode("utf-8", errors="replace") if ptr else "unknown runtime"


def timing_info(lib: ctypes.CDLL) -> str:
    ptr = lib.aero_lbm_timing_info()
    return ptr.decode("utf-8", errors="replace") if ptr else "unknown timing"


def make_boundary(velocity: float = 2.0) -> AeroBoundaryDesc:
    return AeroBoundaryDesc(
        AERO_SOLVER_BOUNDARY_WIND_TUNNEL,
        ctypes.c_float(velocity),
        ctypes.c_float(0.0),
        ctypes.c_float(0.0),
        ctypes.c_float(0.0),
        ctypes.c_float(1.225),
        ctypes.c_float(1.5e-5),
    )


def make_empty_mask(cells: int) -> ctypes.Array[ctypes.c_uint8]:
    return (ctypes.c_uint8 * cells)()


def make_sphere_mask(nx: int, ny: int, nz: int) -> ctypes.Array[ctypes.c_uint8]:
    cells = nx * ny * nz
    mask = (ctypes.c_uint8 * cells)()
    cx = nx // 3
    cy = ny // 2
    cz = nz // 2
    radius = max(1.0, min(nx, ny, nz) * 0.12)
    r2 = radius * radius
    for x in range(nx):
        for y in range(ny):
            for z in range(nz):
                dx = x - cx
                dy = y - cy
                dz = z - cz
                if dx * dx + dy * dy + dz * dz <= r2:
                    mask[cell_index(x, y, z, ny, nz)] = 1
    return mask


def step_case(
    lib: ctypes.CDLL,
    nx: int,
    ny: int,
    nz: int,
    mask: ctypes.Array[ctypes.c_uint8],
    frames: int,
) -> list[list[float]]:
    cells = nx * ny * nz
    values = cells * FLOW_CHANNELS
    handle = ctypes.c_longlong()
    if not lib.aero_solver_create(nx, ny, nz, ctypes.c_float(1.0), ctypes.c_float(0.05), ctypes.byref(handle)):
        raise RuntimeError(f"create failed: {native_error(lib)}")
    try:
        if not lib.aero_solver_set_solid_mask(handle.value, mask, cells):
            raise RuntimeError(f"set_solid_mask failed: {native_error(lib)}")
        boundary = make_boundary()
        out = (ctypes.c_float * values)()
        snapshots: list[list[float]] = []
        for frame in range(frames):
            if not lib.aero_solver_step_wind_tunnel(handle.value, ctypes.byref(boundary), 1, out, values):
                raise RuntimeError(f"frame {frame} failed: {native_error(lib)}")
            snapshots.append([float(out[i]) for i in range(values)])
        return snapshots
    finally:
        lib.aero_solver_destroy(handle.value)


def assert_finite(name: str, values: list[float]) -> None:
    bad = sum(1 for value in values if not math.isfinite(value))
    if bad:
        raise AssertionError(f"{name}: found {bad} non-finite values")


def check_uniform(lib: ctypes.CDLL) -> None:
    nx = ny = nz = 16
    cells = nx * ny * nz
    snapshots = step_case(lib, nx, ny, nz, make_empty_mask(cells), frames=8)
    for index, values in enumerate(snapshots, start=1):
        assert_finite(f"uniform frame {index}", values)
        vxs = values[0::FLOW_CHANNELS]
        vys = values[1::FLOW_CHANNELS]
        vzs = values[2::FLOW_CHANNELS]
        if min(vxs) < 1.85 or max(vxs) > 2.20:
            raise AssertionError(
                f"uniform frame {index}: vx range drifted to [{min(vxs):.6f}, {max(vxs):.6f}]"
            )
        if max(abs(value) for value in vys + vzs) > 0.08:
            raise AssertionError(f"uniform frame {index}: transverse velocity drifted")
    last = snapshots[-1]
    prev = snapshots[-2]
    max_delta = max(abs(a - b) for a, b in zip(last, prev))
    if max_delta > 0.08:
        raise AssertionError(f"uniform parity: adjacent-frame max delta {max_delta:.6f} exceeds tolerance")
    print("[check] uniform/parity ok")


def check_sphere(lib: ctypes.CDLL) -> None:
    nx = ny = nz = 16
    cells = nx * ny * nz
    mask = make_sphere_mask(nx, ny, nz)
    snapshots = step_case(lib, nx, ny, nz, mask, frames=10)
    values = snapshots[-1]
    assert_finite("sphere", values)
    speeds = []
    solid_bad = 0
    for cell in range(cells):
        base = cell * FLOW_CHANNELS
        vx = values[base]
        vy = values[base + 1]
        vz = values[base + 2]
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        speeds.append(speed)
        if mask[cell] and speed > 1.0e-4:
            solid_bad += 1
    if solid_bad:
        raise AssertionError(f"sphere: {solid_bad} solid cells emitted non-zero flow")
    if max(speeds) > 8.0:
        raise AssertionError(f"sphere: max speed {max(speeds):.6f} is implausibly high")
    avg_vx = sum(values[0::FLOW_CHANNELS]) / cells
    if avg_vx <= 0.2:
        raise AssertionError(f"sphere: average vx {avg_vx:.6f} is too low")
    print("[check] sphere obstacle ok")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check D3Q27 FP16 native correctness invariants")
    parser.add_argument("--dll", required=True, type=Path, help="Path to aero_lbm.dll / libaero_lbm.so")
    args = parser.parse_args()

    os.environ.setdefault("AERO_LBM_D3Q27_FP16_INPLACE", "1")
    lib = ctypes.CDLL(str(args.dll))
    configure_api(lib)
    check_uniform(lib)
    check_sphere(lib)
    print(f"[check] runtime={runtime_info(lib)}")
    print(f"[check] timing={timing_info(lib)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
