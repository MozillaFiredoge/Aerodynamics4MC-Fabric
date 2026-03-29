#include <jni.h>

#include "aero_lbm_capi.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(AERO_LBM_OPENCL) && !defined(CL_TARGET_OPENCL_VERSION)
#define CL_TARGET_OPENCL_VERSION 120
#endif

#if defined(AERO_LBM_OPENCL)
#include <CL/cl.h>
#endif

namespace {

constexpr int kQ = 27;
constexpr std::array<int, 3> kVel1D = {-1, 0, 1};
constexpr std::array<float, 3> kW1D = {1.0f / 6.0f, 2.0f / 3.0f, 1.0f / 6.0f};
constexpr std::array<std::array<float, 3>, 3> kMomentInv1D = {{
    {{0.0f, -0.5f, 0.5f}},  // c = -1
    {{1.0f, 0.0f, -1.0f}},  // c =  0
    {{0.0f, 0.5f, 0.5f}}    // c = +1
}};

constexpr int lattice_q(int ix, int iy, int iz) {
    return (ix * 3 + iy) * 3 + iz;
}

constexpr std::array<int, kQ> make_cx() {
    std::array<int, kQ> out{};
    int q = 0;
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 3; ++iz) {
                (void)iy;
                (void)iz;
                out[q++] = kVel1D[ix];
            }
        }
    }
    return out;
}

constexpr std::array<int, kQ> make_cy() {
    std::array<int, kQ> out{};
    int q = 0;
    for (int ix = 0; ix < 3; ++ix) {
        (void)ix;
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 3; ++iz) {
                (void)iz;
                out[q++] = kVel1D[iy];
            }
        }
    }
    return out;
}

constexpr std::array<int, kQ> make_cz() {
    std::array<int, kQ> out{};
    int q = 0;
    for (int ix = 0; ix < 3; ++ix) {
        (void)ix;
        for (int iy = 0; iy < 3; ++iy) {
            (void)iy;
            for (int iz = 0; iz < 3; ++iz) {
                out[q++] = kVel1D[iz];
            }
        }
    }
    return out;
}

constexpr std::array<int, kQ> make_opp() {
    std::array<int, kQ> out{};
    int q = 0;
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 3; ++iz) {
                out[q++] = lattice_q(2 - ix, 2 - iy, 2 - iz);
            }
        }
    }
    return out;
}

constexpr std::array<float, kQ> make_w() {
    std::array<float, kQ> out{};
    int q = 0;
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 3; ++iz) {
                out[q++] = kW1D[ix] * kW1D[iy] * kW1D[iz];
            }
        }
    }
    return out;
}

constexpr std::array<int, kQ> kCx = make_cx();
constexpr std::array<int, kQ> kCy = make_cy();
constexpr std::array<int, kQ> kCz = make_cz();
constexpr std::array<int, kQ> kOpp = make_opp();
constexpr std::array<float, kQ> kW = make_w();

constexpr int kChannelObstacle = 0;
constexpr int kChannelFanMask = 1;
constexpr int kChannelFanVx = 2;
constexpr int kChannelFanVy = 3;
constexpr int kChannelFanVz = 4;
constexpr int kChannelStateVx = 5;
constexpr int kChannelStateVy = 6;
constexpr int kChannelStateVz = 7;
constexpr int kChannelStateP = 8;
constexpr int kChannelThermalSource = 9;

constexpr float kLatticeSoundSpeed = 0.57735026919f;
constexpr float kMaxMach = 0.60f;
constexpr float kHardMaxLatticeSpeed = kLatticeSoundSpeed * kMaxMach;
constexpr float kCs2 = 1.0f / 3.0f;

constexpr float kRhoMin = 0.97f;
constexpr float kRhoMax = 1.03f;
constexpr float kPressureMin = -0.03f;
constexpr float kPressureMax = 0.03f;

// D3Q27 cumulant closure with low-viscosity baseline tau.
constexpr float kTauShear = 0.502f;
constexpr float kTauShearMin = 0.5005;
constexpr float kTauShearMax = 0.95f;
constexpr float kTauNormal = 0.502f;
constexpr float kTauNormalMin = 0.5005f;
constexpr float kTauNormalMax = 0.95f;
constexpr bool kEnableSgs = true;
constexpr float kSgsC = 0.025f;
constexpr float kSgsC2 = kSgsC * kSgsC;
constexpr float kSgsNutToNu0Max = 5.0f;
constexpr float kSgsBulkCoupling = 0.30f;

constexpr int kSpongeLayers = 4;
constexpr float kSpongeStrength = 0.03f;
constexpr float kBoundaryConvectiveBeta = 0.15f;

constexpr float kObstacleBounceBlend = 0.30f;
constexpr float kFanBeta = 0.07f;
constexpr float kFanTargetScale = 1.0f / 30.0f;
constexpr float kFanTargetMax = 0.34f;
constexpr float kFanNoiseAmp = 0.02f;
constexpr float kFanSpeedSoftCap = 0.30f;
constexpr float kFanSpeedDampWidth = 0.06f;
constexpr float kFanPerpDamp = 1.0f;
constexpr float kStateNudge = 0.0f;
constexpr float kMaxSpeed = kHardMaxLatticeSpeed;

// Boussinesq approximation with an internal thermal scalar field.
constexpr bool kEnableBoussinesq = true;
constexpr float kThermalDiffusivity = 0.035f;
constexpr float kThermalCooling = 0.020f;
constexpr float kThermalSourceScale = 0.0012f;
constexpr float kThermalSourceMax = 0.006f;
constexpr float kThermalMin = -0.20f;
constexpr float kThermalMax = 0.20f;
constexpr int kThermalUpdateStride = 2;
constexpr float kBoussinesqBeta = 0.12f;
constexpr float kBoussinesqForceMax = 0.02f;

constexpr float kCylinderBenchmarkLength = 2.2f;
constexpr float kCylinderBenchmarkHeight = 0.41f;
constexpr float kCylinderBenchmarkDiameter = 0.1f;
constexpr float kCylinderBenchmarkCenterX = 0.2f;
constexpr float kCylinderBenchmarkCenterY = 0.2f;
constexpr float kCylinderBenchmarkUmean = 0.08f;
constexpr float kCylinderBenchmarkUmax = 1.5f * kCylinderBenchmarkUmean;

inline float clampf(float v, float lo, float hi);
inline float finite_or(float v, float fallback);

constexpr std::uint32_t kBenchmarkKnownFlags =
    AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_FORCING
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_NOISE
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_SPONGE
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_CONVECTIVE_OUTFLOW
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_OBSTACLE_BOUNCE_BLEND
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_SGS
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE
    | AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY;

struct CylinderBenchmarkGeometry {
    float y_min;
    float y_max;
    float height;
    float center_x;
    float center_y;
    float radius;
    float diameter;
};

inline CylinderBenchmarkGeometry cylinder_benchmark_geometry(int nx, int ny) {
    const float length_cells = static_cast<float>(std::max(1, nx - 1));
    const float channel_height = static_cast<float>(std::max(1, ny - 1));
    float diameter = std::max(2.5f, kCylinderBenchmarkDiameter / kCylinderBenchmarkLength * length_cells);
    diameter = std::min(diameter, std::max(2.5f, channel_height - 4.0f));
    const float radius = 0.5f * diameter;
    const float y_min = 0.0f;
    const float y_max = y_min + channel_height;
    const float center_x = kCylinderBenchmarkCenterX / kCylinderBenchmarkLength * length_cells;
    const float center_y = y_min + kCylinderBenchmarkCenterY / kCylinderBenchmarkHeight * channel_height;
    return {y_min, y_max, channel_height, center_x, center_y, radius, 2.0f * radius};
}

inline float cylinder_benchmark_inlet_ux(int nx, int ny, int y, float u_max) {
    const CylinderBenchmarkGeometry geom = cylinder_benchmark_geometry(nx, ny);
    const float denom = std::max(geom.y_max - geom.y_min, 1.0e-6f);
    const float s = (static_cast<float>(y) - geom.y_min) / denom;
    if (s < 0.0f || s > 1.0f) return 0.0f;
    return 4.0f * u_max * s * (1.0f - s);
}

inline bool cylinder_benchmark_solid_cell(int nx, int ny, int x, int y) {
    const CylinderBenchmarkGeometry geom = cylinder_benchmark_geometry(nx, ny);
    const float dx = static_cast<float>(x) - geom.center_x;
    const float dy = static_cast<float>(y) - geom.center_y;
    return dx * dx + dy * dy <= geom.radius * geom.radius;
}

inline AeroLbmBoundaryFaceConfig make_face_config(int hydro_kind, int thermal_kind) {
    AeroLbmBoundaryFaceConfig face{};
    face.hydrodynamic_kind = hydro_kind;
    face.thermal_kind = thermal_kind;
    return face;
}

bool valid_benchmark_preset(int preset) {
    switch (preset) {
        case AERO_LBM_BENCHMARK_PRESET_NONE:
        case AERO_LBM_BENCHMARK_PRESET_TAYLOR_GREEN_3D:
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D:
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_3D:
        case AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D:
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D:
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D:
            return true;
        default:
            return false;
    }
}

bool valid_hydrodynamic_boundary_kind(int kind) {
    switch (kind) {
        case AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME:
        case AERO_LBM_HYDRO_BOUNDARY_PERIODIC:
        case AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK:
        case AERO_LBM_HYDRO_BOUNDARY_MOVING_WALL:
        case AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET:
        case AERO_LBM_HYDRO_BOUNDARY_PRESSURE_DIRICHLET:
        case AERO_LBM_HYDRO_BOUNDARY_CONVECTIVE_OUTFLOW:
        case AERO_LBM_HYDRO_BOUNDARY_SYMMETRY:
            return true;
        default:
            return false;
    }
}

bool valid_thermal_boundary_kind(int kind) {
    switch (kind) {
        case AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME:
        case AERO_LBM_THERMAL_BOUNDARY_DISABLED:
        case AERO_LBM_THERMAL_BOUNDARY_PERIODIC:
        case AERO_LBM_THERMAL_BOUNDARY_ADIABATIC:
        case AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET:
        case AERO_LBM_THERMAL_BOUNDARY_HEAT_FLUX_NEUMANN:
            return true;
        default:
            return false;
    }
}

inline void sanitize_face_config(AeroLbmBoundaryFaceConfig& face) {
    if (!valid_hydrodynamic_boundary_kind(face.hydrodynamic_kind)) {
        face.hydrodynamic_kind = AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME;
    }
    if (!valid_thermal_boundary_kind(face.thermal_kind)) {
        face.thermal_kind = AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME;
    }
    face.velocity[0] = finite_or(face.velocity[0], 0.0f);
    face.velocity[1] = finite_or(face.velocity[1], 0.0f);
    face.velocity[2] = finite_or(face.velocity[2], 0.0f);
    face.pressure = finite_or(face.pressure, 0.0f);
    face.temperature = finite_or(face.temperature, 0.0f);
    face.heat_flux = finite_or(face.heat_flux, 0.0f);
}

void set_all_faces(
    AeroLbmBenchmarkConfig& cfg, int hydro_kind, int thermal_kind
) {
    cfg.x_min = make_face_config(hydro_kind, thermal_kind);
    cfg.x_max = make_face_config(hydro_kind, thermal_kind);
    cfg.y_min = make_face_config(hydro_kind, thermal_kind);
    cfg.y_max = make_face_config(hydro_kind, thermal_kind);
    cfg.z_min = make_face_config(hydro_kind, thermal_kind);
    cfg.z_max = make_face_config(hydro_kind, thermal_kind);
}

inline AeroLbmBenchmarkConfig make_default_benchmark_config() {
    AeroLbmBenchmarkConfig cfg{};
    cfg.abi_version = AERO_LBM_BENCHMARK_ABI_VERSION;
    cfg.struct_size = sizeof(AeroLbmBenchmarkConfig);
    cfg.flags =
        AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_FORCING
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_NOISE
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_SPONGE
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_OBSTACLE_BOUNCE_BLEND
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_SGS;
    cfg.enabled = 0;
    cfg.preset = AERO_LBM_BENCHMARK_PRESET_NONE;
    cfg.reynolds_number = 100.0f;
    cfg.rayleigh_number = 1.0e5f;
    cfg.prandtl_number = 0.71f;
    cfg.mach_number = 0.05f;
    cfg.reference_density = 1.0f;
    cfg.reference_temperature = 0.0f;
    cfg.reference_length = 1.0f;
    cfg.body_force[0] = 0.0f;
    cfg.body_force[1] = 0.0f;
    cfg.body_force[2] = 0.0f;
    cfg.gravity[0] = 0.0f;
    cfg.gravity[1] = -1.0f;
    cfg.gravity[2] = 0.0f;
    cfg.initial_velocity[0] = 0.0f;
    cfg.initial_velocity[1] = 0.0f;
    cfg.initial_velocity[2] = 0.0f;
    cfg.initial_pressure = 0.0f;
    cfg.initial_temperature = 0.0f;
    set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME, AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME);
    return cfg;
}

void apply_benchmark_preset_defaults(AeroLbmBenchmarkConfig& cfg, int preset) {
    cfg = make_default_benchmark_config();
    cfg.enabled = 1;
    cfg.preset = preset;

    switch (preset) {
        case AERO_LBM_BENCHMARK_PRESET_TAYLOR_GREEN_3D:
            cfg.flags |=
                AERO_LBM_BENCHMARK_FLAG_DISABLE_CONVECTIVE_OUTFLOW
                | AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE
                | AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY;
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            break;

        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D:
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_3D:
            cfg.reynolds_number = 1000.0f;
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK, AERO_LBM_THERMAL_BOUNDARY_ADIABATIC);
            cfg.y_max = make_face_config(AERO_LBM_HYDRO_BOUNDARY_MOVING_WALL, AERO_LBM_THERMAL_BOUNDARY_ADIABATIC);
            cfg.y_max.velocity[0] = 0.10f;
            cfg.flags |=
                AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE
                | AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY;
            if (preset == AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D) {
                cfg.z_min = make_face_config(AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_PERIODIC);
                cfg.z_max = make_face_config(AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_PERIODIC);
            }
            break;

        case AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D:
            cfg.reynolds_number = 100.0f;
            cfg.flags |=
                AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE
                | AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY;
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.x_min = make_face_config(AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.initial_velocity[0] = kCylinderBenchmarkUmean;
            cfg.x_min.velocity[0] = kCylinderBenchmarkUmax;
            cfg.x_max = make_face_config(AERO_LBM_HYDRO_BOUNDARY_CONVECTIVE_OUTFLOW, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.z_min = make_face_config(AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_PERIODIC);
            cfg.z_max = make_face_config(AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_PERIODIC);
            break;

        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D:
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D:
            cfg.rayleigh_number = 1.0e5f;
            cfg.prandtl_number = 0.71f;
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK, AERO_LBM_THERMAL_BOUNDARY_ADIABATIC);
            cfg.x_min = make_face_config(AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK, AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET);
            cfg.x_min.temperature = 0.5f;
            cfg.x_max = make_face_config(AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK, AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET);
            cfg.x_max.temperature = -0.5f;
            if (preset == AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D) {
                cfg.z_min = make_face_config(AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_PERIODIC);
                cfg.z_max = make_face_config(AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_PERIODIC);
            }
            break;

        case AERO_LBM_BENCHMARK_PRESET_NONE:
        default:
            cfg.enabled = 0;
            cfg.preset = AERO_LBM_BENCHMARK_PRESET_NONE;
            break;
    }
}

void sanitize_benchmark_config(AeroLbmBenchmarkConfig& cfg) {
    cfg.abi_version = AERO_LBM_BENCHMARK_ABI_VERSION;
    cfg.struct_size = sizeof(AeroLbmBenchmarkConfig);
    cfg.flags &= kBenchmarkKnownFlags;
    cfg.enabled = cfg.enabled != 0 ? 1 : 0;
    if (!valid_benchmark_preset(cfg.preset)) {
        cfg.preset = AERO_LBM_BENCHMARK_PRESET_NONE;
    }
    cfg.reynolds_number = finite_or(cfg.reynolds_number, 100.0f);
    cfg.rayleigh_number = finite_or(cfg.rayleigh_number, 1.0e5f);
    cfg.prandtl_number = finite_or(cfg.prandtl_number, 0.71f);
    cfg.mach_number = clampf(finite_or(cfg.mach_number, 0.05f), 1.0e-4f, 0.30f);
    cfg.reference_density = finite_or(cfg.reference_density, 1.0f);
    cfg.reference_temperature = finite_or(cfg.reference_temperature, 0.0f);
    cfg.reference_length = std::max(1.0e-6f, finite_or(cfg.reference_length, 1.0f));
    cfg.body_force[0] = finite_or(cfg.body_force[0], 0.0f);
    cfg.body_force[1] = finite_or(cfg.body_force[1], 0.0f);
    cfg.body_force[2] = finite_or(cfg.body_force[2], 0.0f);
    cfg.gravity[0] = finite_or(cfg.gravity[0], 0.0f);
    cfg.gravity[1] = finite_or(cfg.gravity[1], -1.0f);
    cfg.gravity[2] = finite_or(cfg.gravity[2], 0.0f);
    cfg.initial_velocity[0] = finite_or(cfg.initial_velocity[0], 0.0f);
    cfg.initial_velocity[1] = finite_or(cfg.initial_velocity[1], 0.0f);
    cfg.initial_velocity[2] = finite_or(cfg.initial_velocity[2], 0.0f);
    cfg.initial_pressure = finite_or(cfg.initial_pressure, 0.0f);
    cfg.initial_temperature = finite_or(cfg.initial_temperature, 0.0f);
    sanitize_face_config(cfg.x_min);
    sanitize_face_config(cfg.x_max);
    sanitize_face_config(cfg.y_min);
    sanitize_face_config(cfg.y_max);
    sanitize_face_config(cfg.z_min);
    sanitize_face_config(cfg.z_max);
}

const char* benchmark_preset_name(int preset) {
    switch (preset) {
        case AERO_LBM_BENCHMARK_PRESET_TAYLOR_GREEN_3D: return "taylor_green_3d";
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D: return "lid_driven_cavity_2d";
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_3D: return "lid_driven_cavity_3d";
        case AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D: return "cylinder_crossflow_2d";
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D: return "heated_cavity_2d";
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D: return "heated_cavity_3d";
        case AERO_LBM_BENCHMARK_PRESET_NONE:
        default:
            return "none";
    }
}

const char* hydrodynamic_boundary_name(int kind) {
    switch (kind) {
        case AERO_LBM_HYDRO_BOUNDARY_PERIODIC: return "periodic";
        case AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK: return "bounce_back";
        case AERO_LBM_HYDRO_BOUNDARY_MOVING_WALL: return "moving_wall";
        case AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET: return "velocity";
        case AERO_LBM_HYDRO_BOUNDARY_PRESSURE_DIRICHLET: return "pressure";
        case AERO_LBM_HYDRO_BOUNDARY_CONVECTIVE_OUTFLOW: return "convective_outflow";
        case AERO_LBM_HYDRO_BOUNDARY_SYMMETRY: return "symmetry";
        case AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME:
        default:
            return "inherit_game";
    }
}

std::string benchmark_info_string(const AeroLbmBenchmarkConfig& cfg) {
    std::ostringstream oss;
    oss << "enabled=" << cfg.enabled
        << " preset=" << benchmark_preset_name(cfg.preset)
        << " flags=0x" << std::hex << cfg.flags << std::dec
        << " Re=" << cfg.reynolds_number
        << " Ra=" << cfg.rayleigh_number
        << " Pr=" << cfg.prandtl_number
        << " Ma=" << cfg.mach_number
        << " boundaries={x-:" << hydrodynamic_boundary_name(cfg.x_min.hydrodynamic_kind)
        << ",x+:" << hydrodynamic_boundary_name(cfg.x_max.hydrodynamic_kind)
        << ",y-:" << hydrodynamic_boundary_name(cfg.y_min.hydrodynamic_kind)
        << ",y+:" << hydrodynamic_boundary_name(cfg.y_max.hydrodynamic_kind)
        << ",z-:" << hydrodynamic_boundary_name(cfg.z_min.hydrodynamic_kind)
        << ",z+:" << hydrodynamic_boundary_name(cfg.z_max.hydrodynamic_kind)
        << "}";
    return oss.str();
}

struct Config {
    int grid_size = 0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    int input_channels = 0;
    int output_channels = 0;
    bool initialized = false;
    bool opencl_enabled = false;
    std::string runtime_info;
};

struct ContextState {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::size_t cells = 0;
    bool cpu_initialized = false;

    std::vector<float> f;
    std::vector<float> f_post;
    std::vector<float> rho;
    std::vector<float> ux;
    std::vector<float> uy;
    std::vector<float> uz;

    std::vector<float> ref_ux;
    std::vector<float> ref_uy;
    std::vector<float> ref_uz;
    std::vector<float> ref_pressure;

    std::vector<float> packet; // reused host-side payload buffer (floats)
    std::vector<float> fan_mask;
    std::vector<float> fan_ux;
    std::vector<float> fan_uy;
    std::vector<float> fan_uz;
    std::vector<float> thermal_source;
    std::vector<uint8_t> obstacle;
    std::vector<float> temperature;
    std::vector<float> temperature_next;
    float last_force[3] = {0.0f, 0.0f, 0.0f};
    std::uint64_t step_counter = 0;

#if defined(AERO_LBM_OPENCL)
    bool gpu_buffers_ready = false;
    bool gpu_initialized = false;
    cl_mem d_payload = nullptr;
    cl_mem d_f = nullptr;
    cl_mem d_f_post = nullptr;
    cl_mem d_output = nullptr;
    cl_mem d_temp = nullptr;
    cl_mem d_temp_next = nullptr;
#endif
};

Config g_cfg;
std::unordered_map<jlong, ContextState> g_contexts;

struct StepTiming {
    double payload_copy_ms = 0.0;
    double solver_ms = 0.0;
    double readback_ms = 0.0;
    double total_ms = 0.0;
};

struct TimingStats {
    std::uint64_t ticks = 0;
    double payload_copy_ms_sum = 0.0;
    double solver_ms_sum = 0.0;
    double readback_ms_sum = 0.0;
    double total_ms_sum = 0.0;
    StepTiming last;
};

TimingStats g_timing;
AeroLbmBenchmarkConfig g_benchmark_cfg = make_default_benchmark_config();
using Clock = std::chrono::steady_clock;

enum BoundaryFaceIndex {
    kFaceXMin = 0,
    kFaceXMax = 1,
    kFaceYMin = 2,
    kFaceYMax = 3,
    kFaceZMin = 4,
    kFaceZMax = 5,
};

inline bool benchmark_mode_active() {
    return g_benchmark_cfg.enabled != 0;
}

inline bool benchmark_flag_enabled(std::uint32_t flag) {
    return benchmark_mode_active() && (g_benchmark_cfg.flags & flag) != 0;
}

inline const AeroLbmBoundaryFaceConfig& boundary_face_config(int face_index) {
    switch (face_index) {
        case kFaceXMin: return g_benchmark_cfg.x_min;
        case kFaceXMax: return g_benchmark_cfg.x_max;
        case kFaceYMin: return g_benchmark_cfg.y_min;
        case kFaceYMax: return g_benchmark_cfg.y_max;
        case kFaceZMin: return g_benchmark_cfg.z_min;
        case kFaceZMax: return g_benchmark_cfg.z_max;
        default: return g_benchmark_cfg.x_min;
    }
}

inline bool wrap_axis_periodic(int& coord, int dim, int min_face, int max_face, bool thermal) {
    if (coord >= 0 && coord < dim) return false;
    const int face = coord < 0 ? min_face : max_face;
    const AeroLbmBoundaryFaceConfig& cfg = boundary_face_config(face);
    const int kind = thermal ? cfg.thermal_kind : cfg.hydrodynamic_kind;
    const int periodic_kind = thermal ? static_cast<int>(AERO_LBM_THERMAL_BOUNDARY_PERIODIC)
                                      : static_cast<int>(AERO_LBM_HYDRO_BOUNDARY_PERIODIC);
    if (kind != periodic_kind) return false;
    coord %= dim;
    if (coord < 0) coord += dim;
    return true;
}

inline const AeroLbmBoundaryFaceConfig* hydrodynamic_face_for_oob(int x, int y, int z, int nx, int ny, int nz) {
    if (x < 0) return &boundary_face_config(kFaceXMin);
    if (x >= nx) return &boundary_face_config(kFaceXMax);
    if (y < 0) return &boundary_face_config(kFaceYMin);
    if (y >= ny) return &boundary_face_config(kFaceYMax);
    if (z < 0) return &boundary_face_config(kFaceZMin);
    if (z >= nz) return &boundary_face_config(kFaceZMax);
    return nullptr;
}

inline const AeroLbmBoundaryFaceConfig* thermal_face_for_oob(int x, int y, int z, int nx, int ny, int nz) {
    if (x < 0) return &boundary_face_config(kFaceXMin);
    if (x >= nx) return &boundary_face_config(kFaceXMax);
    if (y < 0) return &boundary_face_config(kFaceYMin);
    if (y >= ny) return &boundary_face_config(kFaceYMax);
    if (z < 0) return &boundary_face_config(kFaceZMin);
    if (z >= nz) return &boundary_face_config(kFaceZMax);
    return nullptr;
}

inline bool remap_hydrodynamic_coords(int& x, int& y, int& z, int nx, int ny, int nz) {
    bool changed = false;
    changed |= wrap_axis_periodic(x, nx, kFaceXMin, kFaceXMax, false);
    changed |= wrap_axis_periodic(y, ny, kFaceYMin, kFaceYMax, false);
    changed |= wrap_axis_periodic(z, nz, kFaceZMin, kFaceZMax, false);
    return changed;
}

inline bool remap_thermal_coords(int& x, int& y, int& z, int nx, int ny, int nz) {
    bool changed = false;
    changed |= wrap_axis_periodic(x, nx, kFaceXMin, kFaceXMax, true);
    changed |= wrap_axis_periodic(y, ny, kFaceYMin, kFaceYMax, true);
    changed |= wrap_axis_periodic(z, nz, kFaceZMin, kFaceZMax, true);
    return changed;
}

inline float effective_sponge_alpha(int nx, int ny, int nz, int x, int y, int z) {
    if (benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_SPONGE)) return 0.0f;
    if (kSpongeLayers <= 0) return 0.0f;
    const int d = std::min({x, y, z, nx - 1 - x, ny - 1 - y, nz - 1 - z});
    if (d >= kSpongeLayers) return 0.0f;
    const float eta = static_cast<float>(kSpongeLayers - d) / static_cast<float>(kSpongeLayers);
    return clampf(kSpongeStrength * eta * eta, 0.0f, 0.95f);
}

inline float effective_obstacle_bounce_blend() {
    return benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_OBSTACLE_BOUNCE_BLEND) ? 0.0f : kObstacleBounceBlend;
}

inline bool effective_enable_sgs() {
    return kEnableSgs && !benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_SGS);
}

inline bool effective_enable_buoyancy() {
    return kEnableBoussinesq && !benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY);
}

inline bool effective_enable_internal_thermal_source() {
    return !benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE);
}

inline bool effective_enable_fan_forcing() {
    return !benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_FORCING);
}

inline float effective_fan_noise_amp() {
    return benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_NOISE) ? 0.0f : kFanNoiseAmp;
}

inline float effective_fan_beta() {
    return effective_enable_fan_forcing() ? kFanBeta : 0.0f;
}

inline float benchmark_reference_speed() {
    if (!benchmark_mode_active()) return 0.0f;
    if (g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D) {
        const float initial_speed = std::sqrt(
            g_benchmark_cfg.initial_velocity[0] * g_benchmark_cfg.initial_velocity[0]
            + g_benchmark_cfg.initial_velocity[1] * g_benchmark_cfg.initial_velocity[1]
            + g_benchmark_cfg.initial_velocity[2] * g_benchmark_cfg.initial_velocity[2]
        );
        if (initial_speed > 1.0e-8f) return initial_speed;
        const float face_speed = std::sqrt(
            g_benchmark_cfg.x_min.velocity[0] * g_benchmark_cfg.x_min.velocity[0]
            + g_benchmark_cfg.x_min.velocity[1] * g_benchmark_cfg.x_min.velocity[1]
            + g_benchmark_cfg.x_min.velocity[2] * g_benchmark_cfg.x_min.velocity[2]
        );
        return (2.0f / 3.0f) * face_speed;
    }
    float speed = std::sqrt(
        g_benchmark_cfg.initial_velocity[0] * g_benchmark_cfg.initial_velocity[0]
        + g_benchmark_cfg.initial_velocity[1] * g_benchmark_cfg.initial_velocity[1]
        + g_benchmark_cfg.initial_velocity[2] * g_benchmark_cfg.initial_velocity[2]
    );
    const AeroLbmBoundaryFaceConfig faces[6] = {
        g_benchmark_cfg.x_min, g_benchmark_cfg.x_max,
        g_benchmark_cfg.y_min, g_benchmark_cfg.y_max,
        g_benchmark_cfg.z_min, g_benchmark_cfg.z_max,
    };
    for (const AeroLbmBoundaryFaceConfig& face : faces) {
        const float face_speed = std::sqrt(
            face.velocity[0] * face.velocity[0]
            + face.velocity[1] * face.velocity[1]
            + face.velocity[2] * face.velocity[2]
        );
        speed = std::max(speed, face_speed);
    }
    return speed;
}

inline float effective_benchmark_tau_shear() {
    if (!benchmark_mode_active()) return kTauShear;
    const float re = finite_or(g_benchmark_cfg.reynolds_number, 0.0f);
    const float ref_speed = benchmark_reference_speed();
    const float ref_length = std::max(1.0e-6f, finite_or(g_benchmark_cfg.reference_length, 1.0f));
    if (re <= 1.0e-6f || ref_speed <= 1.0e-8f) return kTauShear;
    const float nu = ref_speed * ref_length / re;
    return clampf(0.5f + 3.0f * nu, kTauShearMin, kTauShearMax);
}

inline float effective_benchmark_tau_normal(float tau_shear_eff) {
    if (!benchmark_mode_active()) return clampf(kTauNormal, kTauNormalMin, kTauNormalMax);
    return clampf(tau_shear_eff, kTauNormalMin, kTauNormalMax);
}

inline int opencl_benchmark_flags() {
    if (!benchmark_mode_active()) return 0;
    return static_cast<int>(g_benchmark_cfg.flags & kBenchmarkKnownFlags);
}

inline int opencl_hydrodynamic_periodic_mask() {
    if (!benchmark_mode_active()) return 0;
    int mask = 0;
    if (g_benchmark_cfg.x_min.hydrodynamic_kind == AERO_LBM_HYDRO_BOUNDARY_PERIODIC
        && g_benchmark_cfg.x_max.hydrodynamic_kind == AERO_LBM_HYDRO_BOUNDARY_PERIODIC) {
        mask |= 1;
    }
    if (g_benchmark_cfg.y_min.hydrodynamic_kind == AERO_LBM_HYDRO_BOUNDARY_PERIODIC
        && g_benchmark_cfg.y_max.hydrodynamic_kind == AERO_LBM_HYDRO_BOUNDARY_PERIODIC) {
        mask |= 2;
    }
    if (g_benchmark_cfg.z_min.hydrodynamic_kind == AERO_LBM_HYDRO_BOUNDARY_PERIODIC
        && g_benchmark_cfg.z_max.hydrodynamic_kind == AERO_LBM_HYDRO_BOUNDARY_PERIODIC) {
        mask |= 4;
    }
    return mask;
}

inline bool benchmark_opencl_supported() {
    if (!benchmark_mode_active()) return true;
    switch (g_benchmark_cfg.preset) {
        case AERO_LBM_BENCHMARK_PRESET_TAYLOR_GREEN_3D:
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D:
        case AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_3D:
        case AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D:
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D:
        case AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D:
            return true;
        default:
            return false;
    }
}

using OpenClFaceData = std::array<float, 4>;

inline std::array<int, 6> opencl_hydrodynamic_face_kinds() {
    return {
        g_benchmark_cfg.x_min.hydrodynamic_kind,
        g_benchmark_cfg.x_max.hydrodynamic_kind,
        g_benchmark_cfg.y_min.hydrodynamic_kind,
        g_benchmark_cfg.y_max.hydrodynamic_kind,
        g_benchmark_cfg.z_min.hydrodynamic_kind,
        g_benchmark_cfg.z_max.hydrodynamic_kind,
    };
}

inline std::array<OpenClFaceData, 6> opencl_hydrodynamic_face_data() {
    return {{
        {{g_benchmark_cfg.x_min.velocity[0], g_benchmark_cfg.x_min.velocity[1], g_benchmark_cfg.x_min.velocity[2], g_benchmark_cfg.x_min.pressure}},
        {{g_benchmark_cfg.x_max.velocity[0], g_benchmark_cfg.x_max.velocity[1], g_benchmark_cfg.x_max.velocity[2], g_benchmark_cfg.x_max.pressure}},
        {{g_benchmark_cfg.y_min.velocity[0], g_benchmark_cfg.y_min.velocity[1], g_benchmark_cfg.y_min.velocity[2], g_benchmark_cfg.y_min.pressure}},
        {{g_benchmark_cfg.y_max.velocity[0], g_benchmark_cfg.y_max.velocity[1], g_benchmark_cfg.y_max.velocity[2], g_benchmark_cfg.y_max.pressure}},
        {{g_benchmark_cfg.z_min.velocity[0], g_benchmark_cfg.z_min.velocity[1], g_benchmark_cfg.z_min.velocity[2], g_benchmark_cfg.z_min.pressure}},
        {{g_benchmark_cfg.z_max.velocity[0], g_benchmark_cfg.z_max.velocity[1], g_benchmark_cfg.z_max.velocity[2], g_benchmark_cfg.z_max.pressure}},
    }};
}

inline int opencl_thermal_periodic_mask() {
    if (!benchmark_mode_active()) return 0;
    int mask = 0;
    if (g_benchmark_cfg.x_min.thermal_kind == AERO_LBM_THERMAL_BOUNDARY_PERIODIC
        && g_benchmark_cfg.x_max.thermal_kind == AERO_LBM_THERMAL_BOUNDARY_PERIODIC) {
        mask |= 1;
    }
    if (g_benchmark_cfg.y_min.thermal_kind == AERO_LBM_THERMAL_BOUNDARY_PERIODIC
        && g_benchmark_cfg.y_max.thermal_kind == AERO_LBM_THERMAL_BOUNDARY_PERIODIC) {
        mask |= 2;
    }
    if (g_benchmark_cfg.z_min.thermal_kind == AERO_LBM_THERMAL_BOUNDARY_PERIODIC
        && g_benchmark_cfg.z_max.thermal_kind == AERO_LBM_THERMAL_BOUNDARY_PERIODIC) {
        mask |= 4;
    }
    return mask;
}

inline std::array<int, 6> opencl_thermal_face_kinds() {
    return {
        g_benchmark_cfg.x_min.thermal_kind,
        g_benchmark_cfg.x_max.thermal_kind,
        g_benchmark_cfg.y_min.thermal_kind,
        g_benchmark_cfg.y_max.thermal_kind,
        g_benchmark_cfg.z_min.thermal_kind,
        g_benchmark_cfg.z_max.thermal_kind,
    };
}

inline std::array<OpenClFaceData, 6> opencl_thermal_face_data() {
    return {{
        {{g_benchmark_cfg.x_min.temperature, g_benchmark_cfg.x_min.heat_flux, 0.0f, 0.0f}},
        {{g_benchmark_cfg.x_max.temperature, g_benchmark_cfg.x_max.heat_flux, 0.0f, 0.0f}},
        {{g_benchmark_cfg.y_min.temperature, g_benchmark_cfg.y_min.heat_flux, 0.0f, 0.0f}},
        {{g_benchmark_cfg.y_max.temperature, g_benchmark_cfg.y_max.heat_flux, 0.0f, 0.0f}},
        {{g_benchmark_cfg.z_min.temperature, g_benchmark_cfg.z_min.heat_flux, 0.0f, 0.0f}},
        {{g_benchmark_cfg.z_max.temperature, g_benchmark_cfg.z_max.heat_flux, 0.0f, 0.0f}},
    }};
}

inline std::array<float, 2> opencl_effective_tau_pair() {
    const float tau_shear = effective_benchmark_tau_shear();
    const float tau_normal = effective_benchmark_tau_normal(tau_shear);
    return {tau_shear, tau_normal};
}

double elapsed_ms(const Clock::time_point& begin, const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

void reset_timing_stats() { g_timing = TimingStats{}; }

void record_timing(const StepTiming& timing) {
    g_timing.ticks += 1;
    g_timing.payload_copy_ms_sum += timing.payload_copy_ms;
    g_timing.solver_ms_sum += timing.solver_ms;
    g_timing.readback_ms_sum += timing.readback_ms;
    g_timing.total_ms_sum += timing.total_ms;
    g_timing.last = timing;
}

std::string timing_info_string() {
    if (g_timing.ticks == 0) return "ticks=0";
    const double inv = 1.0 / static_cast<double>(g_timing.ticks);
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(3) << "ticks=" << g_timing.ticks
        << " last_ms(copy=" << g_timing.last.payload_copy_ms
        << ",solver=" << g_timing.last.solver_ms
        << ",readback=" << g_timing.last.readback_ms
        << ",total=" << g_timing.last.total_ms << ")"
        << " avg_ms(copy=" << g_timing.payload_copy_ms_sum * inv
        << ",solver=" << g_timing.solver_ms_sum * inv
        << ",readback=" << g_timing.readback_ms_sum * inv
        << ",total=" << g_timing.total_ms_sum * inv << ")";
    return oss.str();
}

inline std::size_t cell_index(int x, int y, int z, int nx, int ny, int nz) {
    return (static_cast<std::size_t>(x) * ny + y) * nz + z;
}

// 极为关键的一步：CPU 侧同样采用 SoA (Structure of Arrays) 布局！
inline std::size_t dist_index(std::size_t cell, int q, std::size_t cells) {
    return static_cast<std::size_t>(q) * cells + cell;
}

inline float clampf(float v, float lo, float hi) {
    return std::min(hi, std::max(lo, v));
}

inline bool finitef(float v) {
    return std::isfinite(v);
}

inline float finite_or(float v, float fallback) {
    return finitef(v) ? v : fallback;
}

inline float feq(int q, float rho, float ux, float uy, float uz) {
    const float cu = 3.0f * (kCx[q] * ux + kCy[q] * uy + kCz[q] * uz);
    const float uu = ux * ux + uy * uy + uz * uz;
    return kW[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

inline void decode_cell(std::size_t cell, int nx, int ny, int nz, int& x, int& y, int& z) {
    (void)nx;
    const int yz = ny * nz;
    x = static_cast<int>(cell / static_cast<std::size_t>(yz));
    const int rem = static_cast<int>(cell - static_cast<std::size_t>(x) * yz);
    y = rem / nz;
    z = rem - y * nz;
}

inline bool solid_or_oob(const ContextState& ctx, int x, int y, int z) {
    remap_hydrodynamic_coords(x, y, z, ctx.nx, ctx.ny, ctx.nz);
    if (x < 0 || y < 0 || z < 0 || x >= ctx.nx || y >= ctx.ny || z >= ctx.nz) return true;
    return ctx.obstacle[cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz)] != 0;
}

inline float hash_signed_noise(std::uint64_t cell, std::uint64_t tick) {
    std::uint64_t x = cell * 0x9E3779B97F4A7C15ULL ^ tick * 0xD1B54A32D192ED03ULL;
    x ^= x >> 30;
    x *= 0xBF58476D1CE4E5B9ULL;
    x ^= x >> 27;
    x *= 0x94D049BB133111EBULL;
    x ^= x >> 31;
    const float u = static_cast<float>(x & 0x00FFFFFFULL) * (1.0f / 16777215.0f);
    return 2.0f * u - 1.0f;
}

inline int binom(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (n <= 1 || k == 0 || k == n) return 1;
    return 2;  // only used by n=2,k=1
}

inline void compute_raw_moments(const std::array<float, kQ>& f_local, float raw[3][3][3]) {
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int c = 0; c < 3; ++c) {
                raw[a][b][c] = 0.0f;
            }
        }
    }

    for (int q = 0; q < kQ; ++q) {
        const float fq = f_local[q];
        const float px[3] = {1.0f, static_cast<float>(kCx[q]), static_cast<float>(kCx[q] * kCx[q])};
        const float py[3] = {1.0f, static_cast<float>(kCy[q]), static_cast<float>(kCy[q] * kCy[q])};
        const float pz[3] = {1.0f, static_cast<float>(kCz[q]), static_cast<float>(kCz[q] * kCz[q])};
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) {
                for (int c = 0; c < 3; ++c) {
                    raw[a][b][c] += fq * px[a] * py[b] * pz[c];
                }
            }
        }
    }
}

inline void compute_central_moments(
    const float raw[3][3][3], float ux, float uy, float uz, float central[3][3][3]
) {
    const float sx[3] = {1.0f, -ux, ux * ux};
    const float sy[3] = {1.0f, -uy, uy * uy};
    const float sz[3] = {1.0f, -uz, uz * uz};

    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int c = 0; c < 3; ++c) {
                float sum = 0.0f;
                for (int p = 0; p <= a; ++p) {
                    for (int q = 0; q <= b; ++q) {
                        for (int r = 0; r <= c; ++r) {
                            const float coeff = static_cast<float>(binom(a, p) * binom(b, q) * binom(c, r));
                            sum += coeff * sx[a - p] * sy[b - q] * sz[c - r] * raw[p][q][r];
                        }
                    }
                }
                central[a][b][c] = sum;
            }
        }
    }
}

inline void cumulant_relax_closure(
    float rho, float central_in[3][3][3], float omega_diag, float omega_offdiag, float central_out[3][3][3]
) {
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int c = 0; c < 3; ++c) {
                central_out[a][b][c] = 0.0f;
            }
        }
    }

    central_out[0][0][0] = rho;

    const float eq_second = rho * kCs2;
    const float s_diag = clampf(omega_diag, 0.0f, 1.95f);
    const float s_offdiag = clampf(omega_offdiag, 0.0f, 1.95f);

    // Relax second-order cumulants (equal to second-order central moments).
    central_out[2][0][0] = central_in[2][0][0] + s_diag * (eq_second - central_in[2][0][0]);
    central_out[0][2][0] = central_in[0][2][0] + s_diag * (eq_second - central_in[0][2][0]);
    central_out[0][0][2] = central_in[0][0][2] + s_diag * (eq_second - central_in[0][0][2]);
    central_out[1][1][0] = (1.0f - s_offdiag) * central_in[1][1][0];
    central_out[1][0][1] = (1.0f - s_offdiag) * central_in[1][0][1];
    central_out[0][1][1] = (1.0f - s_offdiag) * central_in[0][1][1];

    // Higher-order cumulants are set to zero, then reconstructed via Gaussian closure.
    const float c200 = central_out[2][0][0];
    const float c020 = central_out[0][2][0];
    const float c002 = central_out[0][0][2];
    const float c110 = central_out[1][1][0];
    const float c101 = central_out[1][0][1];
    const float c011 = central_out[0][1][1];
    const float inv_rho = 1.0f / std::max(1e-6f, rho);

    central_out[2][2][0] = c200 * c020 * inv_rho + 2.0f * c110 * c110 * inv_rho;
    central_out[2][0][2] = c200 * c002 * inv_rho + 2.0f * c101 * c101 * inv_rho;
    central_out[0][2][2] = c020 * c002 * inv_rho + 2.0f * c011 * c011 * inv_rho;
    central_out[2][1][1] = (c200 * c011 + 2.0f * c110 * c101) * inv_rho;
    central_out[1][2][1] = (c020 * c101 + 2.0f * c110 * c011) * inv_rho;
    central_out[1][1][2] = (c002 * c110 + 2.0f * c101 * c011) * inv_rho;
    central_out[2][2][2] = (c200 * c020 * c002
                             + 2.0f * c110 * c110 * c002
                             + 2.0f * c101 * c101 * c020
                             + 2.0f * c011 * c011 * c200
                             + 8.0f * c110 * c101 * c011)
                            * inv_rho * inv_rho;
}

inline void central_to_raw(
    const float central[3][3][3], float ux, float uy, float uz, float raw_out[3][3][3]
) {
    const float ux_pow[3] = {1.0f, ux, ux * ux};
    const float uy_pow[3] = {1.0f, uy, uy * uy};
    const float uz_pow[3] = {1.0f, uz, uz * uz};

    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int c = 0; c < 3; ++c) {
                float sum = 0.0f;
                for (int p = 0; p <= a; ++p) {
                    for (int q = 0; q <= b; ++q) {
                        for (int r = 0; r <= c; ++r) {
                            const float coeff = static_cast<float>(binom(a, p) * binom(b, q) * binom(c, r));
                            sum += coeff * ux_pow[a - p] * uy_pow[b - q] * uz_pow[c - r] * central[p][q][r];
                        }
                    }
                }
                raw_out[a][b][c] = sum;
            }
        }
    }
}

inline void raw_to_populations(const float raw[3][3][3], std::array<float, kQ>& f_out) {
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 3; ++iz) {
                float sum = 0.0f;
                for (int a = 0; a < 3; ++a) {
                    for (int b = 0; b < 3; ++b) {
                        for (int c = 0; c < 3; ++c) {
                            sum += kMomentInv1D[ix][a] * kMomentInv1D[iy][b] * kMomentInv1D[iz][c] * raw[a][b][c];
                        }
                    }
                }
                f_out[lattice_q(ix, iy, iz)] = sum;
            }
        }
    }
}

inline float obstacle_bounce_value(
    const ContextState& ctx, std::size_t cell, int x, int y, int z, int q
) {
    const float bounced = ctx.f_post[dist_index(cell, kOpp[q], ctx.cells)];
    const float blend = effective_obstacle_bounce_blend();
    if (blend <= 0.0f) return bounced;

    const int s2x = x - 2 * kCx[q];
    const int s2y = y - 2 * kCy[q];
    const int s2z = z - 2 * kCz[q];
    if (s2x < 0 || s2y < 0 || s2z < 0 || s2x >= ctx.nx || s2y >= ctx.ny || s2z >= ctx.nz) return bounced;

    const std::size_t src2 = cell_index(s2x, s2y, s2z, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.obstacle[src2]) return bounced;

    const float upstream = ctx.f_post[dist_index(src2, q, ctx.cells)];
    return bounced + blend * (upstream - bounced);
}

inline float boundary_convective_value(
    const ContextState& ctx, std::size_t cell, int x, int y, int z, int q, int sx, int sy, int sz
) {
    const int dx = (sx < 0 || sx >= ctx.nx) ? kCx[q] : 0;
    const int dy = (sy < 0 || sy >= ctx.ny) ? kCy[q] : 0;
    const int dz = (sz < 0 || sz >= ctx.nz) ? kCz[q] : 0;

    const int i1x = std::clamp(x + dx, 0, ctx.nx - 1);
    const int i1y = std::clamp(y + dy, 0, ctx.ny - 1);
    const int i1z = std::clamp(z + dz, 0, ctx.nz - 1);
    const std::size_t src1 = cell_index(i1x, i1y, i1z, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.obstacle[src1]) return obstacle_bounce_value(ctx, cell, x, y, z, q);

    const float f1 = ctx.f_post[dist_index(src1, q, ctx.cells)];
    const int i2x = std::clamp(x + 2 * dx, 0, ctx.nx - 1);
    const int i2y = std::clamp(y + 2 * dy, 0, ctx.ny - 1);
    const int i2z = std::clamp(z + 2 * dz, 0, ctx.nz - 1);
    const std::size_t src2 = cell_index(i2x, i2y, i2z, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.obstacle[src2]) return f1;

    const float f2 = ctx.f_post[dist_index(src2, q, ctx.cells)];
    return f1 + kBoundaryConvectiveBeta * (f1 - f2);
}

inline float boundary_equilibrium_value(
    const ContextState& ctx,
    std::size_t cell,
    int x,
    int y,
    int z,
    int q,
    int sx,
    int sy,
    int sz,
    const AeroLbmBoundaryFaceConfig& face
) {
    float rho = 1.0f + clampf(ctx.ref_pressure[cell], kPressureMin, kPressureMax);
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    switch (face.hydrodynamic_kind) {
        case AERO_LBM_HYDRO_BOUNDARY_MOVING_WALL:
        case AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET:
            ux = face.velocity[0];
            uy = face.velocity[1];
            uz = face.velocity[2];
            if (g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D && sx < 0) {
                ux = cylinder_benchmark_inlet_ux(ctx.nx, ctx.ny, y, face.velocity[0]);
                uy = 0.0f;
                uz = 0.0f;
            }
            break;
        case AERO_LBM_HYDRO_BOUNDARY_PRESSURE_DIRICHLET:
            rho = 1.0f + clampf(face.pressure, kPressureMin, kPressureMax);
            break;
        default:
            break;
    }
    rho = clampf(rho, kRhoMin, kRhoMax);
    float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > kMaxSpeed * kMaxSpeed && speed2 > 0.0f) {
        const float scale = kMaxSpeed / std::sqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
    }
    return feq(q, rho, ux, uy, uz);
}

inline float benchmark_hydrodynamic_boundary_value(
    const ContextState& ctx,
    std::size_t cell,
    int x,
    int y,
    int z,
    int q,
    int sx,
    int sy,
    int sz
) {
    const AeroLbmBoundaryFaceConfig* face = hydrodynamic_face_for_oob(sx, sy, sz, ctx.nx, ctx.ny, ctx.nz);
    const int kind = face ? face->hydrodynamic_kind : AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME;
    switch (kind) {
        case AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK:
            return obstacle_bounce_value(ctx, cell, x, y, z, q);
        case AERO_LBM_HYDRO_BOUNDARY_MOVING_WALL:
        case AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET:
        case AERO_LBM_HYDRO_BOUNDARY_PRESSURE_DIRICHLET:
            return boundary_equilibrium_value(ctx, cell, x, y, z, q, sx, sy, sz, *face);
        case AERO_LBM_HYDRO_BOUNDARY_SYMMETRY: {
            const int ix = std::clamp(sx, 0, ctx.nx - 1);
            const int iy = std::clamp(sy, 0, ctx.ny - 1);
            const int iz = std::clamp(sz, 0, ctx.nz - 1);
            const std::size_t src = cell_index(ix, iy, iz, ctx.nx, ctx.ny, ctx.nz);
            return ctx.obstacle[src] ? obstacle_bounce_value(ctx, cell, x, y, z, q) : ctx.f_post[dist_index(src, q, ctx.cells)];
        }
        case AERO_LBM_HYDRO_BOUNDARY_CONVECTIVE_OUTFLOW:
            return boundary_convective_value(ctx, cell, x, y, z, q, sx, sy, sz);
        case AERO_LBM_HYDRO_BOUNDARY_PERIODIC:
            break;
        case AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME:
        default:
            if (benchmark_flag_enabled(AERO_LBM_BENCHMARK_FLAG_DISABLE_CONVECTIVE_OUTFLOW)) {
                return obstacle_bounce_value(ctx, cell, x, y, z, q);
            }
            return boundary_convective_value(ctx, cell, x, y, z, q, sx, sy, sz);
    }
    return boundary_convective_value(ctx, cell, x, y, z, q, sx, sy, sz);
}

void allocate_cpu_context(ContextState& ctx, int nx, int ny, int nz) {
    ctx.nx = nx;
    ctx.ny = ny;
    ctx.nz = nz;
    ctx.cells = static_cast<std::size_t>(nx) * ny * nz;
    ctx.cpu_initialized = false;

    ctx.f.assign(ctx.cells * kQ, 0.0f);
    ctx.f_post.assign(ctx.cells * kQ, 0.0f);
    ctx.rho.assign(ctx.cells, 1.0f);
    ctx.ux.assign(ctx.cells, 0.0f);
    ctx.uy.assign(ctx.cells, 0.0f);
    ctx.uz.assign(ctx.cells, 0.0f);

    ctx.ref_ux.assign(ctx.cells, 0.0f);
    ctx.ref_uy.assign(ctx.cells, 0.0f);
    ctx.ref_uz.assign(ctx.cells, 0.0f);
    ctx.ref_pressure.assign(ctx.cells, 0.0f);
    ctx.packet.assign(ctx.cells * g_cfg.input_channels, 0.0f);

    ctx.fan_mask.assign(ctx.cells, 0.0f);
    ctx.fan_ux.assign(ctx.cells, 0.0f);
    ctx.fan_uy.assign(ctx.cells, 0.0f);
    ctx.fan_uz.assign(ctx.cells, 0.0f);
    ctx.thermal_source.assign(ctx.cells, 0.0f);
    ctx.obstacle.assign(ctx.cells, 0);
    ctx.temperature.assign(ctx.cells, 0.0f);
    ctx.temperature_next.assign(ctx.cells, 0.0f);
    ctx.last_force[0] = 0.0f;
    ctx.last_force[1] = 0.0f;
    ctx.last_force[2] = 0.0f;
}

void ingest_payload(ContextState& ctx, const float* payload, int in_channels) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t base = cell * in_channels;
        ctx.obstacle[cell] = payload[base + kChannelObstacle] > 0.5f ? 1 : 0;
        ctx.fan_mask[cell] = clampf(payload[base + kChannelFanMask], 0.0f, 1.0f);
        ctx.fan_ux[cell] = finite_or(payload[base + kChannelFanVx], 0.0f);
        ctx.fan_uy[cell] = finite_or(payload[base + kChannelFanVy], 0.0f);
        ctx.fan_uz[cell] = finite_or(payload[base + kChannelFanVz], 0.0f);
        ctx.ref_ux[cell] = finite_or(payload[base + kChannelStateVx], 0.0f);
        ctx.ref_uy[cell] = finite_or(payload[base + kChannelStateVy], 0.0f);
        ctx.ref_uz[cell] = finite_or(payload[base + kChannelStateVz], 0.0f);
        ctx.ref_pressure[cell] = clampf(payload[base + kChannelStateP], kPressureMin, kPressureMax);
        float thermal_src = 0.0f;
        if (in_channels > kChannelThermalSource) {
            thermal_src = payload[base + kChannelThermalSource];
        }
        ctx.thermal_source[cell] = clampf(thermal_src, -kThermalSourceMax, kThermalSourceMax);
    }
}

void reseed_cell_from_reference(ContextState& ctx, std::size_t cell) {
    const float pressure = clampf(finite_or(ctx.ref_pressure[cell], 0.0f), kPressureMin, kPressureMax);
    const float rho = clampf(1.0f + pressure, kRhoMin, kRhoMax);
    float ux = finite_or(ctx.ref_ux[cell], 0.0f);
    float uy = finite_or(ctx.ref_uy[cell], 0.0f);
    float uz = finite_or(ctx.ref_uz[cell], 0.0f);
    float speed2 = ux * ux + uy * uy + uz * uz;
    if (!finitef(speed2) || speed2 > kMaxSpeed * kMaxSpeed) {
        if (!finitef(speed2) || speed2 <= 0.0f) {
            ux = 0.0f;
            uy = 0.0f;
            uz = 0.0f;
        } else {
            const float scale = kMaxSpeed / std::sqrt(speed2);
            ux *= scale;
            uy *= scale;
            uz *= scale;
        }
    }
    ctx.rho[cell] = rho;
    ctx.ux[cell] = ux;
    ctx.uy[cell] = uy;
    ctx.uz[cell] = uz;
    for (int q = 0; q < kQ; ++q) {
        ctx.f_post[dist_index(cell, q, ctx.cells)] = feq(q, rho, ux, uy, uz);
    }
}

void initialize_distributions(ContextState& ctx) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        if (ctx.obstacle[cell]) {
            ctx.rho[cell] = 1.0f;
            ctx.ux[cell] = ctx.uy[cell] = ctx.uz[cell] = 0.0f;
        } else {
            ctx.rho[cell] = clampf(1.0f + ctx.ref_pressure[cell], kRhoMin, kRhoMax);
            ctx.ux[cell] = ctx.ref_ux[cell];
            ctx.uy[cell] = ctx.ref_uy[cell];
            ctx.uz[cell] = ctx.ref_uz[cell];
        }

        const float rho = ctx.rho[cell];
        const float ux = ctx.ux[cell], uy = ctx.uy[cell], uz = ctx.uz[cell];
        for (int q = 0; q < kQ; ++q) {
            float eq = feq(q, rho, ux, uy, uz);
            ctx.f[dist_index(cell, q, ctx.cells)] = eq;
            ctx.f_post[dist_index(cell, q, ctx.cells)] = eq;
        }
        if (benchmark_mode_active() && std::fabs(ctx.temperature[cell]) < 1e-8f) {
            ctx.temperature[cell] = clampf(g_benchmark_cfg.initial_temperature, kThermalMin, kThermalMax);
        }
        ctx.temperature[cell] = clampf(ctx.temperature[cell], kThermalMin, kThermalMax);
        ctx.temperature_next[cell] = ctx.temperature[cell];
    }
    ctx.cpu_initialized = true;
}

inline float thermal_source_term(const ContextState& ctx, std::size_t cell) {
    if (!effective_enable_internal_thermal_source()) return 0.0f;
    if (g_cfg.input_channels > kChannelThermalSource) {
        return clampf(ctx.thermal_source[cell], -kThermalSourceMax, kThermalSourceMax);
    }
    if (ctx.fan_mask[cell] <= 0.0f) return 0.0f;
    const float fan_norm = std::sqrt(
        ctx.fan_ux[cell] * ctx.fan_ux[cell]
            + ctx.fan_uy[cell] * ctx.fan_uy[cell]
            + ctx.fan_uz[cell] * ctx.fan_uz[cell]
    );
    if (fan_norm <= 1e-8f) return 0.0f;
    const float capped = clampf(fan_norm * kThermalSourceScale, 0.0f, kThermalSourceMax);
    return ctx.fan_mask[cell] * capped;
}

inline float thermal_neighbor_or_self(const ContextState& ctx, int x, int y, int z, float self_value) {
    remap_thermal_coords(x, y, z, ctx.nx, ctx.ny, ctx.nz);
    if (x < 0 || y < 0 || z < 0 || x >= ctx.nx || y >= ctx.ny || z >= ctx.nz) {
        const AeroLbmBoundaryFaceConfig* face = thermal_face_for_oob(x, y, z, ctx.nx, ctx.ny, ctx.nz);
        const int kind = face ? face->thermal_kind : AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME;
        switch (kind) {
            case AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET:
                return clampf(face->temperature, kThermalMin, kThermalMax);
            case AERO_LBM_THERMAL_BOUNDARY_HEAT_FLUX_NEUMANN:
                return clampf(self_value + face->heat_flux, kThermalMin, kThermalMax);
            case AERO_LBM_THERMAL_BOUNDARY_DISABLED:
            case AERO_LBM_THERMAL_BOUNDARY_ADIABATIC:
            case AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME:
            default:
                return self_value;
        }
    }
    const std::size_t cell = cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.obstacle[cell]) return self_value;
    return ctx.temperature[cell];
}

inline float sample_temperature_trilinear(
    const ContextState& ctx, float x, float y, float z, float fallback
) {
    x = clampf(x, 0.0f, static_cast<float>(ctx.nx - 1));
    y = clampf(y, 0.0f, static_cast<float>(ctx.ny - 1));
    z = clampf(z, 0.0f, static_cast<float>(ctx.nz - 1));

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int z0 = static_cast<int>(std::floor(z));
    const int x1 = std::min(ctx.nx - 1, x0 + 1);
    const int y1 = std::min(ctx.ny - 1, y0 + 1);
    const int z1 = std::min(ctx.nz - 1, z0 + 1);

    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);
    const float fz = z - static_cast<float>(z0);

    const float c000 = thermal_neighbor_or_self(ctx, x0, y0, z0, fallback);
    const float c100 = thermal_neighbor_or_self(ctx, x1, y0, z0, fallback);
    const float c010 = thermal_neighbor_or_self(ctx, x0, y1, z0, fallback);
    const float c110 = thermal_neighbor_or_self(ctx, x1, y1, z0, fallback);
    const float c001 = thermal_neighbor_or_self(ctx, x0, y0, z1, fallback);
    const float c101 = thermal_neighbor_or_self(ctx, x1, y0, z1, fallback);
    const float c011 = thermal_neighbor_or_self(ctx, x0, y1, z1, fallback);
    const float c111 = thermal_neighbor_or_self(ctx, x1, y1, z1, fallback);

    const float c00 = c000 + (c100 - c000) * fx;
    const float c10 = c010 + (c110 - c010) * fx;
    const float c01 = c001 + (c101 - c001) * fx;
    const float c11 = c011 + (c111 - c011) * fx;
    const float c0 = c00 + (c10 - c00) * fy;
    const float c1 = c01 + (c11 - c01) * fy;
    return c0 + (c1 - c0) * fz;
}

void update_temperature_field(ContextState& ctx) {
    if (!kEnableBoussinesq) return;
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        if (ctx.obstacle[cell]) {
            ctx.temperature_next[cell] = 0.0f;
            continue;
        }

        int x = 0, y = 0, z = 0;
        decode_cell(cell, ctx.nx, ctx.ny, ctx.nz, x, y, z);

        const float t_center = ctx.temperature[cell];
        const float advected = sample_temperature_trilinear(
            ctx,
            static_cast<float>(x) - ctx.ux[cell],
            static_cast<float>(y) - ctx.uy[cell],
            static_cast<float>(z) - ctx.uz[cell],
            t_center
        );
        const float t_sum =
            thermal_neighbor_or_self(ctx, x + 1, y, z, t_center)
            + thermal_neighbor_or_self(ctx, x - 1, y, z, t_center)
            + thermal_neighbor_or_self(ctx, x, y + 1, z, t_center)
            + thermal_neighbor_or_self(ctx, x, y - 1, z, t_center)
            + thermal_neighbor_or_self(ctx, x, y, z + 1, t_center)
            + thermal_neighbor_or_self(ctx, x, y, z - 1, t_center);

        const float laplacian = t_sum - 6.0f * t_center;
        const float source = thermal_source_term(ctx, cell);
        const float t_next = advected
                             + kThermalDiffusivity * laplacian
                             + source
                             - kThermalCooling * advected;
        ctx.temperature_next[cell] = clampf(t_next, kThermalMin, kThermalMax);
    }
    ctx.temperature.swap(ctx.temperature_next);
}

inline bool should_update_temperature(std::uint64_t step_counter) {
    if (!kEnableBoussinesq) return false;
    if (kThermalUpdateStride <= 1) return true;
    return (step_counter % static_cast<std::uint64_t>(kThermalUpdateStride)) == 0;
}

void collide(ContextState& ctx) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        if (ctx.obstacle[cell]) {
            ctx.rho[cell] = 1.0f;
            ctx.ux[cell] = ctx.uy[cell] = ctx.uz[cell] = 0.0f;
            for (int q = 0; q < kQ; ++q) {
                ctx.f_post[dist_index(cell, q, ctx.cells)] = ctx.f[dist_index(cell, q, ctx.cells)];
            }
            continue;
        }

        std::array<float, kQ> f_local{};
        float rho = 0.0f;
        float mom_x = 0.0f;
        float mom_y = 0.0f;
        float mom_z = 0.0f;
        bool non_finite_distribution = false;
        for (int q = 0; q < kQ; ++q) {
            const float fq = ctx.f[dist_index(cell, q, ctx.cells)];
            if (!finitef(fq)) {
                non_finite_distribution = true;
                break;
            }
            f_local[q] = fq;
            rho += fq;
            mom_x += fq * static_cast<float>(kCx[q]);
            mom_y += fq * static_cast<float>(kCy[q]);
            mom_z += fq * static_cast<float>(kCz[q]);
        }
        if (non_finite_distribution || !finitef(rho) || !finitef(mom_x) || !finitef(mom_y) || !finitef(mom_z)) {
            reseed_cell_from_reference(ctx, cell);
            continue;
        }

        const float rho_safe = std::max(1e-6f, rho);
        const float inv_rho = 1.0f / rho_safe;
        float ux = mom_x * inv_rho;
        float uy = mom_y * inv_rho;
        float uz = mom_z * inv_rho;
        if (!finitef(rho_safe) || !finitef(inv_rho) || !finitef(ux) || !finitef(uy) || !finitef(uz)) {
            reseed_cell_from_reference(ctx, cell);
            continue;
        }
        const float speed_pre = std::sqrt(ux * ux + uy * uy + uz * uz);

        float fx = 0.0f;
        float fy = 0.0f;
        float fz = 0.0f;
        if (effective_enable_fan_forcing() && ctx.fan_mask[cell] > 0.0f) {
            const float fan_norm = std::sqrt(ctx.fan_ux[cell] * ctx.fan_ux[cell] + ctx.fan_uy[cell] * ctx.fan_uy[cell] + ctx.fan_uz[cell] * ctx.fan_uz[cell]);
            if (fan_norm > 1e-8f) {
                const float inv_norm = 1.0f / fan_norm;
                const float noise = 1.0f + effective_fan_noise_amp() * hash_signed_noise(cell, ctx.step_counter);
                const float target_speed = clampf(
                    fan_norm * kFanTargetScale * std::max(0.0f, noise),
                    0.0f,
                    kFanTargetMax
                );
                const float nx = ctx.fan_ux[cell] * inv_norm;
                const float ny = ctx.fan_uy[cell] * inv_norm;
                const float nz = ctx.fan_uz[cell] * inv_norm;
                const float u_para = ux * nx + uy * ny + uz * nz;
                const float u_perp_x = ux - u_para * nx;
                const float u_perp_y = uy - u_para * ny;
                const float u_perp_z = uz - u_para * nz;
                const float axial_push = std::max(0.0f, target_speed - u_para);
                float speed_damp = 1.0f;
                if (speed_pre > kFanSpeedSoftCap) {
                    const float r = (speed_pre - kFanSpeedSoftCap) / std::max(1e-4f, kFanSpeedDampWidth);
                    speed_damp = 1.0f / (1.0f + r * r);
                }
                const float beta = ctx.fan_mask[cell] * effective_fan_beta() * speed_damp;
                fx = beta * rho_safe * (axial_push * nx - kFanPerpDamp * u_perp_x);
                fy = beta * rho_safe * (axial_push * ny - kFanPerpDamp * u_perp_y);
                fz = beta * rho_safe * (axial_push * nz - kFanPerpDamp * u_perp_z);
            }
        }

        if (effective_enable_buoyancy()) {
            const float buoyancy = clampf(
                kBoussinesqBeta * ctx.temperature[cell],
                -kBoussinesqForceMax,
                kBoussinesqForceMax
            );
            fy += rho_safe * buoyancy;
        }

        const float dux = fx * inv_rho;
        const float duy = fy * inv_rho;
        const float duz = fz * inv_rho;

        // Midpoint velocity for second-order force integration.
        ux += 0.5f * dux;
        uy += 0.5f * duy;
        uz += 0.5f * duz;

        ux = (1.0f - kStateNudge) * ux + kStateNudge * ctx.ref_ux[cell];
        uy = (1.0f - kStateNudge) * uy + kStateNudge * ctx.ref_uy[cell];
        uz = (1.0f - kStateNudge) * uz + kStateNudge * ctx.ref_uz[cell];
        if (!finitef(ux) || !finitef(uy) || !finitef(uz)) {
            reseed_cell_from_reference(ctx, cell);
            continue;
        }

        float speed2 = ux * ux + uy * uy + uz * uz;
        if (!finitef(speed2)) {
            reseed_cell_from_reference(ctx, cell);
            continue;
        }
        if (speed2 > kMaxSpeed * kMaxSpeed) {
            float scale = kMaxSpeed / std::sqrt(speed2);
            ux *= scale; uy *= scale; uz *= scale;
        }

        ctx.rho[cell] = rho;
        ctx.ux[cell] = ux;
        ctx.uy[cell] = uy;
        ctx.uz[cell] = uz;

        float raw[3][3][3];
        float central_pre[3][3][3];
        float central_post[3][3][3];
        float raw_post[3][3][3];
        compute_raw_moments(f_local, raw);
        compute_central_moments(raw, ux, uy, uz, central_pre);

        float tau_shear_local = effective_benchmark_tau_shear();
        float tau_normal_local = effective_benchmark_tau_normal(tau_shear_local);
        if (effective_enable_sgs()) {
            const float nu0 = std::max(1e-6f, (kTauShear - 0.5f) / 3.0f);
            const float neq_xx = central_pre[2][0][0] - rho_safe * kCs2;
            const float neq_yy = central_pre[0][2][0] - rho_safe * kCs2;
            const float neq_zz = central_pre[0][0][2] - rho_safe * kCs2;
            const float neq_xy = central_pre[1][1][0];
            const float neq_xz = central_pre[1][0][1];
            const float neq_yz = central_pre[0][1][1];
            const float q_norm2 = neq_xx * neq_xx + neq_yy * neq_yy + neq_zz * neq_zz
                                  + 2.0f * (neq_xy * neq_xy + neq_xz * neq_xz + neq_yz * neq_yz);
            const float q_mag = std::sqrt(std::max(0.0f, q_norm2));
            const float s_mag = q_mag / std::max(1e-6f, 2.0f * rho_safe * nu0);
            float nu_t = kSgsC2 * s_mag;
            nu_t = std::min(nu_t, kSgsNutToNu0Max * nu0);
            tau_shear_local = clampf(0.5f + 3.0f * (nu0 + nu_t), kTauShearMin, kTauShearMax);
            tau_normal_local = clampf(
                kTauNormal + (tau_shear_local - kTauShear) * kSgsBulkCoupling,
                kTauNormalMin,
                kTauNormalMax
            );
        }

        const float omega_diag = 1.0f / tau_normal_local;
        const float omega_offdiag = 1.0f / tau_shear_local;
        cumulant_relax_closure(rho_safe, central_pre, omega_diag, omega_offdiag, central_post);
        central_to_raw(central_post, ux, uy, uz, raw_post);

        std::array<float, kQ> f_cumulant{};
        raw_to_populations(raw_post, f_cumulant);

        // Enforce exact low-order conservation after moment inversion.
        float rho_corr = 0.0f;
        float mx_corr = 0.0f;
        float my_corr = 0.0f;
        float mz_corr = 0.0f;
        for (int q = 0; q < kQ; ++q) {
            const float fq = f_cumulant[q];
            rho_corr += fq;
            mx_corr += fq * static_cast<float>(kCx[q]);
            my_corr += fq * static_cast<float>(kCy[q]);
            mz_corr += fq * static_cast<float>(kCz[q]);
        }
        const float rho_corr_safe = std::max(1e-6f, rho_corr);
        const float ux_corr = mx_corr / rho_corr_safe;
        const float uy_corr = my_corr / rho_corr_safe;
        const float uz_corr = mz_corr / rho_corr_safe;
        for (int q = 0; q < kQ; ++q) {
            f_cumulant[q] += feq(q, rho_safe, ux, uy, uz) - feq(q, rho_corr_safe, ux_corr, uy_corr, uz_corr);
        }

        // Exact Difference Method (EDM) forcing in population space.
        const float ux_minus = ux - 0.5f * dux;
        const float uy_minus = uy - 0.5f * duy;
        const float uz_minus = uz - 0.5f * duz;
        const float ux_plus = ux + 0.5f * dux;
        const float uy_plus = uy + 0.5f * duy;
        const float uz_plus = uz + 0.5f * duz;

        for (int q = 0; q < kQ; ++q) {
            const float edm_source = feq(q, rho_safe, ux_plus, uy_plus, uz_plus) - feq(q, rho_safe, ux_minus, uy_minus, uz_minus);
            ctx.f_post[dist_index(cell, q, ctx.cells)] = f_cumulant[q] + edm_source;
        }

        int cx = 0, cy = 0, cz = 0;
        decode_cell(cell, ctx.nx, ctx.ny, ctx.nz, cx, cy, cz);
        const float alpha = effective_sponge_alpha(ctx.nx, ctx.ny, ctx.nz, cx, cy, cz);
        if (alpha > 0.0f) {
            const float keep = 1.0f - alpha;
            for (int q = 0; q < kQ; ++q) {
                const float feq_far = feq(q, 1.0f, 0.0f, 0.0f, 0.0f);
                auto& fref = ctx.f_post[dist_index(cell, q, ctx.cells)];
                fref = keep * fref + alpha * feq_far;
            }
        }
    }
}

void stream_and_bounce(ContextState& ctx) {
    for (int x = 0; x < ctx.nx; ++x) {
        for (int y = 0; y < ctx.ny; ++y) {
            for (int z = 0; z < ctx.nz; ++z) {
                const std::size_t cell = cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz);
                std::array<float, kQ> f_local{};
                if (ctx.obstacle[cell]) {
                    for (int q = 0; q < kQ; ++q) {
                        f_local[q] = ctx.f_post[dist_index(cell, kOpp[q], ctx.cells)];
                    }
                } else {
                    for (int q = 0; q < kQ; ++q) {
                        int sx = x - kCx[q];
                        int sy = y - kCy[q];
                        int sz = z - kCz[q];
                        remap_hydrodynamic_coords(sx, sy, sz, ctx.nx, ctx.ny, ctx.nz);
                        if (sx < 0 || sy < 0 || sz < 0 || sx >= ctx.nx || sy >= ctx.ny || sz >= ctx.nz) {
                            f_local[q] = benchmark_hydrodynamic_boundary_value(ctx, cell, x, y, z, q, sx, sy, sz);
                            continue;
                        }

                        const std::size_t src = cell_index(sx, sy, sz, ctx.nx, ctx.ny, ctx.nz);
                        if (ctx.obstacle[src]) {
                            f_local[q] = obstacle_bounce_value(ctx, cell, x, y, z, q);
                        } else {
                            f_local[q] = ctx.f_post[dist_index(src, q, ctx.cells)];
                        }
                    }

                }

                for (int q = 0; q < kQ; ++q) {
                    ctx.f[dist_index(cell, q, ctx.cells)] = f_local[q];
                }
            }
        }
    }
}

void compute_benchmark_force(ContextState& ctx) {
    ctx.last_force[0] = 0.0f;
    ctx.last_force[1] = 0.0f;
    ctx.last_force[2] = 0.0f;
    if (!benchmark_mode_active() || g_benchmark_cfg.preset != AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D) {
        return;
    }

    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    for (int x = 0; x < ctx.nx; ++x) {
        for (int y = 0; y < ctx.ny; ++y) {
            for (int z = 0; z < ctx.nz; ++z) {
                const std::size_t cell = cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz);
                if (ctx.obstacle[cell]) continue;
                for (int q = 1; q < kQ; ++q) {
                    if (kCz[q] != 0) continue;
                    const int nx = x + kCx[q];
                    const int ny = y + kCy[q];
                    const int nz = z + kCz[q];
                    if (nx < 0 || ny < 0 || nz < 0 || nx >= ctx.nx || ny >= ctx.ny || nz >= ctx.nz) continue;
                    const std::size_t neighbor = cell_index(nx, ny, nz, ctx.nx, ctx.ny, ctx.nz);
                    if (!ctx.obstacle[neighbor]) continue;
                    if (!cylinder_benchmark_solid_cell(ctx.nx, ctx.ny, nx, ny)) continue;
                    const float fq = ctx.f_post[dist_index(cell, q, ctx.cells)];
                    fx += 2.0 * static_cast<double>(fq) * static_cast<double>(kCx[q]);
                    fy += 2.0 * static_cast<double>(fq) * static_cast<double>(kCy[q]);
                    fz += 2.0 * static_cast<double>(fq) * static_cast<double>(kCz[q]);
                }
            }
        }
    }

    const double inv_depth = ctx.nz > 0 ? (1.0 / static_cast<double>(ctx.nz)) : 1.0;
    ctx.last_force[0] = static_cast<float>(fx * inv_depth);
    ctx.last_force[1] = static_cast<float>(fy * inv_depth);
    ctx.last_force[2] = static_cast<float>(fz * inv_depth);
}

void compute_benchmark_force_from_distributions(ContextState& ctx, const float* dist_data) {
    ctx.last_force[0] = 0.0f;
    ctx.last_force[1] = 0.0f;
    ctx.last_force[2] = 0.0f;
    if (!dist_data) return;
    if (!benchmark_mode_active() || g_benchmark_cfg.preset != AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D) {
        return;
    }

    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    for (int x = 0; x < ctx.nx; ++x) {
        for (int y = 0; y < ctx.ny; ++y) {
            for (int z = 0; z < ctx.nz; ++z) {
                const std::size_t cell = cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz);
                if (ctx.obstacle[cell]) continue;
                for (int q = 1; q < kQ; ++q) {
                    if (kCz[q] != 0) continue;
                    const int nx = x + kCx[q];
                    const int ny = y + kCy[q];
                    const int nz = z + kCz[q];
                    if (nx < 0 || ny < 0 || nz < 0 || nx >= ctx.nx || ny >= ctx.ny || nz >= ctx.nz) continue;
                    const std::size_t neighbor = cell_index(nx, ny, nz, ctx.nx, ctx.ny, ctx.nz);
                    if (!ctx.obstacle[neighbor]) continue;
                    if (!cylinder_benchmark_solid_cell(ctx.nx, ctx.ny, nx, ny)) continue;
                    const float fq = dist_data[dist_index(cell, q, ctx.cells)];
                    fx += 2.0 * static_cast<double>(fq) * static_cast<double>(kCx[q]);
                    fy += 2.0 * static_cast<double>(fq) * static_cast<double>(kCy[q]);
                    fz += 2.0 * static_cast<double>(fq) * static_cast<double>(kCz[q]);
                }
            }
        }
    }

    const double inv_depth = ctx.nz > 0 ? (1.0 / static_cast<double>(ctx.nz)) : 1.0;
    ctx.last_force[0] = static_cast<float>(fx * inv_depth);
    ctx.last_force[1] = static_cast<float>(fy * inv_depth);
    ctx.last_force[2] = static_cast<float>(fz * inv_depth);
}

void write_output(const ContextState& ctx, float* out, int out_channels) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t out_base = cell * out_channels;
        if (ctx.obstacle[cell]) {
            for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
            continue;
        }

        float rho = 0.0f, ux = 0.0f, uy = 0.0f, uz = 0.0f;
        for (int q = 0; q < kQ; ++q) {
            const float fq = ctx.f[dist_index(cell, q, ctx.cells)];
            if (!finitef(fq)) {
                rho = 1.0f;
                ux = uy = uz = 0.0f;
                for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
                goto next_cell;
            }
            rho += fq;
            ux += fq * kCx[q]; uy += fq * kCy[q]; uz += fq * kCz[q];
        }
        if (!finitef(rho) || !finitef(ux) || !finitef(uy) || !finitef(uz)) {
            for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
            goto next_cell;
        }
        rho = clampf(rho, kRhoMin, kRhoMax);
        if (!finitef(rho) || rho <= 1e-6f) {
            for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
            goto next_cell;
        }
        ux /= rho; uy /= rho; uz /= rho;
        if (!finitef(ux) || !finitef(uy) || !finitef(uz)) {
            ux = uy = uz = 0.0f;
        }
        if (std::fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;
        if (std::fabs(ux) < 1e-7f) ux = 0.0f;
        if (std::fabs(uy) < 1e-7f) uy = 0.0f;
        if (std::fabs(uz) < 1e-7f) uz = 0.0f;

        out[out_base + 0] = ux;
        out[out_base + 1] = uy;
        out[out_base + 2] = uz;
        out[out_base + 3] = clampf(rho - 1.0f, kPressureMin, kPressureMax);
        for (int c = 4; c < out_channels; ++c) out[out_base + c] = 0.0f;
next_cell:
        (void)0;
    }
}

#if defined(AERO_LBM_OPENCL)

struct OpenClRuntime {
    bool available = false;
    std::string error;
    std::string device_name;
    cl_platform_id platform = nullptr;
    cl_device_id device = nullptr;
    cl_context context = nullptr;
    cl_command_queue queue = nullptr;
    cl_program program = nullptr;
    cl_kernel k_init = nullptr;
    cl_kernel k_stream_collide = nullptr;
    cl_kernel k_output = nullptr;
};

OpenClRuntime g_opencl;

const char* kOpenClSource =
R"CLC(
#define KQ 27
#define MI(a, b, c) (((a) * 3 + (b)) * 3 + (c))

__constant int CX[KQ] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1,
     0,  0,  0,  0,  0,  0,  0,  0,  0,
     1,  1,  1,  1,  1,  1,  1,  1,  1
};
__constant int CY[KQ] = {
    -1, -1, -1,  0,  0,  0,  1,  1,  1,
    -1, -1, -1,  0,  0,  0,  1,  1,  1,
    -1, -1, -1,  0,  0,  0,  1,  1,  1
};
__constant int CZ[KQ] = {
    -1,  0,  1, -1,  0,  1, -1,  0,  1,
    -1,  0,  1, -1,  0,  1, -1,  0,  1,
    -1,  0,  1, -1,  0,  1, -1,  0,  1
};
__constant int OPP[KQ] = {
    26, 25, 24, 23, 22, 21, 20, 19, 18,
    17, 16, 15, 14, 13, 12, 11, 10, 9,
     8,  7,  6,  5,  4,  3,  2,  1, 0
};
__constant float W[KQ] = {
    0.004629630f, 0.018518519f, 0.004629630f,
    0.018518519f, 0.074074074f, 0.018518519f,
    0.004629630f, 0.018518519f, 0.004629630f,
    0.018518519f, 0.074074074f, 0.018518519f,
    0.074074074f, 0.296296296f, 0.074074074f,
    0.018518519f, 0.074074074f, 0.018518519f,
    0.004629630f, 0.018518519f, 0.004629630f,
    0.018518519f, 0.074074074f, 0.018518519f,
    0.004629630f, 0.018518519f, 0.004629630f
};
__constant float TINV[3][3] = {
    {0.0f, -0.5f, 0.5f},
    {1.0f, 0.0f, -1.0f},
    {0.0f, 0.5f, 0.5f}
};

__constant float TAU_SHEAR = 0.502f;
__constant float TAU_SHEAR_MIN = 0.5005f;
__constant float TAU_SHEAR_MAX = 0.95f;
__constant float TAU_NORMAL = 0.502f;
__constant float TAU_NORMAL_MIN = 0.5005f;
__constant float TAU_NORMAL_MAX = 0.95f;
__constant int SGS_ENABLED = 1;
__constant float SGS_C2 = 0.0049f;
__constant float SGS_NUT_TO_NU0_MAX = 0.4f;
__constant float SGS_BULK_COUPLING = 0.30f;
__constant int SPONGE_LAYERS = 4;
__constant float SPONGE_STRENGTH = 0.03f;
__constant float BOUNDARY_CONVECTIVE_BETA = 0.15f;
__constant float OBSTACLE_BOUNCE_BLEND = 0.30f;
__constant float FAN_BETA = 0.07f;
__constant float FAN_TARGET_SCALE = 0.033333335f;
__constant float FAN_TARGET_MAX = 0.34f;
__constant float FAN_NOISE_AMP = 0.02f;
__constant float FAN_SPEED_SOFT_CAP = 0.30f;
__constant float FAN_SPEED_DAMP_WIDTH = 0.06f;
__constant float FAN_PERP_DAMP = 1.0f;
__constant float STATE_NUDGE = 0.0f;
__constant float MAX_SPEED = 0.34641016f;
__constant float RHO_MIN = 0.97f;
__constant float RHO_MAX = 1.03f;
__constant float CS2 = 0.33333334f;
__constant float P_MIN = -0.03f;
__constant float P_MAX = 0.03f;
__constant int BOUSSINESQ_ENABLED = 1;
__constant float THERMAL_DIFFUSIVITY = 0.035f;
__constant float THERMAL_COOLING = 0.020f;
__constant float THERMAL_SOURCE_SCALE = 0.0012f;
__constant float THERMAL_SOURCE_MAX = 0.006f;
__constant float THERMAL_MIN = -0.20f;
__constant float THERMAL_MAX = 0.20f;
__constant int THERMAL_UPDATE_STRIDE = 2;
__constant float BOUSSINESQ_BETA = 0.12f;
__constant float BOUSSINESQ_FORCE_MAX = 0.02f;

#define BENCH_DISABLE_FAN_FORCING (1 << 0)
#define BENCH_DISABLE_FAN_NOISE (1 << 1)
#define BENCH_DISABLE_SPONGE (1 << 2)
#define BENCH_DISABLE_CONVECTIVE_OUTFLOW (1 << 3)
#define BENCH_DISABLE_OBSTACLE_BOUNCE_BLEND (1 << 4)
#define BENCH_DISABLE_SGS (1 << 5)
#define BENCH_DISABLE_INTERNAL_THERMAL_SOURCE (1 << 6)
#define BENCH_DISABLE_BUOYANCY (1 << 7)

#define BENCH_PRESET_CYLINDER_CROSSFLOW_2D 4

#define PERIODIC_AXIS_X 1
#define PERIODIC_AXIS_Y 2
#define PERIODIC_AXIS_Z 4

#define CYLINDER_BENCHMARK_LENGTH 2.2f
#define CYLINDER_BENCHMARK_HEIGHT 0.41f

inline float clampf(float v, float lo, float hi) { return fmin(hi, fmax(lo, v)); }
inline int clampi(int v, int lo, int hi) { return min(hi, max(lo, v)); }
inline int binom(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (n <= 1 || k == 0 || k == n) return 1;
    return 2;
}
inline float feq(int q, float rho, float ux, float uy, float uz) {
    float cu = 3.0f * (CX[q] * ux + CY[q] * uy + CZ[q] * uz);
    float uu = ux * ux + uy * uy + uz * uz;
    return W[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

inline float obstacle_bounce_value(
    __global const float* f_read, __global const float* payload,
    int in_ch, int nx, int ny, int nz, int cells, int cell, int x, int y, int z, int q, int opp, int benchmark_flags
);

inline int wrap_axis_if_periodic(int coord, int dim, int axis_bit, int periodic_mask) {
    if (coord >= 0 && coord < dim) return coord;
    if ((periodic_mask & axis_bit) == 0) return coord;
    coord %= dim;
    if (coord < 0) coord += dim;
    return coord;
}

inline float cylinder_benchmark_inlet_ux(int nx, int ny, int y, float u_max) {
    float height_cells = (float)max(1, ny - 1);
    (void)nx;
    float s = (float)y / fmax(height_cells, 1.0e-6f);
    if (s < 0.0f || s > 1.0f) return 0.0f;
    return 4.0f * u_max * s * (1.0f - s);
}

inline int face_kind_for_oob(
    int sx, int sy, int sz, int nx, int ny, int nz,
    int x_min_kind, int x_max_kind,
    int y_min_kind, int y_max_kind,
    int z_min_kind, int z_max_kind
) {
    if (sx < 0) return x_min_kind;
    if (sx >= nx) return x_max_kind;
    if (sy < 0) return y_min_kind;
    if (sy >= ny) return y_max_kind;
    if (sz < 0) return z_min_kind;
    if (sz >= nz) return z_max_kind;
    return 0;
}

inline float4 face_data_for_oob(
    int sx, int sy, int sz, int nx, int ny, int nz,
    float4 x_min_data, float4 x_max_data,
    float4 y_min_data, float4 y_max_data,
    float4 z_min_data, float4 z_max_data
) {
    if (sx < 0) return x_min_data;
    if (sx >= nx) return x_max_data;
    if (sy < 0) return y_min_data;
    if (sy >= ny) return y_max_data;
    if (sz < 0) return z_min_data;
    if (sz >= nz) return z_max_data;
    return (float4)(0.0f, 0.0f, 0.0f, 0.0f);
}

inline uint hash_u32(uint x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

inline float signed_noise(uint cell, uint tick) {
    uint h = hash_u32(cell * 747796405U ^ tick * 2891336453U);
    float u = (float)(h & 0x00FFFFFFU) * (1.0f / 16777215.0f);
    return 2.0f * u - 1.0f;
}

inline float sponge_alpha(int nx, int ny, int nz, int x, int y, int z) {
    if (SPONGE_LAYERS <= 0) return 0.0f;
    int d = min(min(min(x, y), min(z, nz - 1 - z)), min(nx - 1 - x, ny - 1 - y));
    if (d >= SPONGE_LAYERS) return 0.0f;
    float eta = (float)(SPONGE_LAYERS - d) / (float)SPONGE_LAYERS;
    return clampf(SPONGE_STRENGTH * eta * eta, 0.0f, 0.95f);
}

inline float boundary_convective_value(
    __global const float* f_read, __global const float* payload,
    int in_ch, int nx, int ny, int nz, int cells, int cell, int x, int y, int z, int q, int sx, int sy, int sz, int benchmark_flags
) {
    int dx = (sx < 0 || sx >= nx) ? CX[q] : 0;
    int dy = (sy < 0 || sy >= ny) ? CY[q] : 0;
    int dz = (sz < 0 || sz >= nz) ? CZ[q] : 0;

    int i1x = clampi(x + dx, 0, nx - 1);
    int i1y = clampi(y + dy, 0, ny - 1);
    int i1z = clampi(z + dz, 0, nz - 1);
    int src1 = (i1x * ny + i1y) * nz + i1z;
    if (payload[src1 * in_ch + 0] > 0.5f) {
        return obstacle_bounce_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, OPP[q], benchmark_flags);
    }

    float f1 = f_read[q * cells + src1];
    int i2x = clampi(x + 2 * dx, 0, nx - 1);
    int i2y = clampi(y + 2 * dy, 0, ny - 1);
    int i2z = clampi(z + 2 * dz, 0, nz - 1);
    int src2 = (i2x * ny + i2y) * nz + i2z;
    if (payload[src2 * in_ch + 0] > 0.5f) return f1;

    float f2 = f_read[q * cells + src2];
    return f1 + BOUNDARY_CONVECTIVE_BETA * (f1 - f2);
}

inline float boundary_equilibrium_value(
    int q,
    float4 face_data,
    float ref_pressure,
    int use_face_pressure,
    int benchmark_preset,
    int nx,
    int ny,
    int y,
    int sx
) {
    float rho = 1.0f + clampf(ref_pressure, P_MIN, P_MAX);
    float ux = face_data.x;
    float uy = face_data.y;
    float uz = face_data.z;
    float pressure = face_data.w;
    if (use_face_pressure != 0) {
        rho = 1.0f + clampf(pressure, P_MIN, P_MAX);
    }
    if (use_face_pressure == 0 && benchmark_preset == BENCH_PRESET_CYLINDER_CROSSFLOW_2D && sx < 0) {
        ux = cylinder_benchmark_inlet_ux(nx, ny, y, face_data.x);
        uy = 0.0f;
        uz = 0.0f;
    }
    rho = clampf(rho, RHO_MIN, RHO_MAX);
    float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > MAX_SPEED * MAX_SPEED && speed2 > 0.0f) {
        float scale = MAX_SPEED * rsqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
    }
    return feq(q, rho, ux, uy, uz);
}

inline float benchmark_boundary_value(
    __global const float* f_read,
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
    int cell,
    int x,
    int y,
    int z,
    int q,
    int sx,
    int sy,
    int sz,
    int benchmark_flags,
    int x_min_kind,
    int x_max_kind,
    int y_min_kind,
    int y_max_kind,
    int z_min_kind,
    int z_max_kind,
    float4 x_min_data,
    float4 x_max_data,
    float4 y_min_data,
    float4 y_max_data,
    float4 z_min_data,
    float4 z_max_data,
    int benchmark_preset
) {
    int kind = face_kind_for_oob(sx, sy, sz, nx, ny, nz, x_min_kind, x_max_kind, y_min_kind, y_max_kind, z_min_kind, z_max_kind);
    float4 face_data = face_data_for_oob(sx, sy, sz, nx, ny, nz, x_min_data, x_max_data, y_min_data, y_max_data, z_min_data, z_max_data);
    switch (kind) {
        case 2:
            return obstacle_bounce_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, OPP[q], benchmark_flags);
        case 3:
        case 4:
            return boundary_equilibrium_value(q, face_data, payload[cell * in_ch + 8], 0, benchmark_preset, nx, ny, y, sx);
        case 5: {
            float4 pressure_face = (float4)(0.0f, 0.0f, 0.0f, face_data.w);
            return boundary_equilibrium_value(q, pressure_face, payload[cell * in_ch + 8], 1, benchmark_preset, nx, ny, y, sx);
        }
        case 7: {
            int ix = clampi(sx, 0, nx - 1);
            int iy = clampi(sy, 0, ny - 1);
            int iz = clampi(sz, 0, nz - 1);
            int src = (ix * ny + iy) * nz + iz;
            if (payload[src * in_ch + 0] > 0.5f) {
                return obstacle_bounce_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, OPP[q], benchmark_flags);
            }
            return f_read[q * cells + src];
        }
        case 6:
            return boundary_convective_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, sx, sy, sz, benchmark_flags);
        case 0:
        default:
            if ((benchmark_flags & BENCH_DISABLE_CONVECTIVE_OUTFLOW) != 0) {
                return obstacle_bounce_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, OPP[q], benchmark_flags);
            }
            return boundary_convective_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, sx, sy, sz, benchmark_flags);
    }
}

inline float temperature_or_self(
    __global const float* temp,
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int thermal_periodic_mask,
    int tx_min_kind,
    int tx_max_kind,
    int ty_min_kind,
    int ty_max_kind,
    int tz_min_kind,
    int tz_max_kind,
    float4 tx_min_data,
    float4 tx_max_data,
    float4 ty_min_data,
    float4 ty_max_data,
    float4 tz_min_data,
    float4 tz_max_data,
    int x,
    int y,
    int z,
    float self_value
) {
    x = wrap_axis_if_periodic(x, nx, PERIODIC_AXIS_X, thermal_periodic_mask);
    y = wrap_axis_if_periodic(y, ny, PERIODIC_AXIS_Y, thermal_periodic_mask);
    z = wrap_axis_if_periodic(z, nz, PERIODIC_AXIS_Z, thermal_periodic_mask);
    if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) {
        int kind = face_kind_for_oob(x, y, z, nx, ny, nz, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind);
        float4 face_data = face_data_for_oob(x, y, z, nx, ny, nz, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data);
        switch (kind) {
            case 4:
                return clampf(face_data.x, THERMAL_MIN, THERMAL_MAX);
            case 5:
                return clampf(self_value + face_data.y, THERMAL_MIN, THERMAL_MAX);
            case 1:
            case 2:
            case 3:
            case 0:
            default:
                return self_value;
        }
    }
    int cell = (x * ny + y) * nz + z;
    if (payload[cell * in_ch + 0] > 0.5f) return self_value;
    return temp[cell];
}

inline float sample_temperature_trilinear(
    __global const float* temp,
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int thermal_periodic_mask,
    int tx_min_kind,
    int tx_max_kind,
    int ty_min_kind,
    int ty_max_kind,
    int tz_min_kind,
    int tz_max_kind,
    float4 tx_min_data,
    float4 tx_max_data,
    float4 ty_min_data,
    float4 ty_max_data,
    float4 tz_min_data,
    float4 tz_max_data,
    float px,
    float py,
    float pz,
    float fallback
) {
    int x0 = (int)floor(px);
    int y0 = (int)floor(py);
    int z0 = (int)floor(pz);
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    int z1 = z0 + 1;

    float fx = px - (float)x0;
    float fy = py - (float)y0;
    float fz = pz - (float)z0;

    float c000 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x0, y0, z0, fallback);
    float c100 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x1, y0, z0, fallback);
    float c010 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x0, y1, z0, fallback);
    float c110 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x1, y1, z0, fallback);
    float c001 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x0, y0, z1, fallback);
    float c101 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x1, y0, z1, fallback);
    float c011 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x0, y1, z1, fallback);
    float c111 = temperature_or_self(temp, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x1, y1, z1, fallback);

    float c00 = c000 + (c100 - c000) * fx;
    float c10 = c010 + (c110 - c010) * fx;
    float c01 = c001 + (c101 - c001) * fx;
    float c11 = c011 + (c111 - c011) * fx;
    float c0 = c00 + (c10 - c00) * fy;
    float c1 = c01 + (c11 - c01) * fy;
    return c0 + (c1 - c0) * fz;
}

inline float obstacle_bounce_value(
    __global const float* f_read, __global const float* payload,
    int in_ch, int nx, int ny, int nz, int cells, int cell, int x, int y, int z, int q, int opp, int benchmark_flags
) {
    float bounced = f_read[opp * cells + cell];
    float blend = (benchmark_flags & BENCH_DISABLE_OBSTACLE_BOUNCE_BLEND) ? 0.0f : OBSTACLE_BOUNCE_BLEND;
    if (blend <= 0.0f) return bounced;

    int s2x = x - 2 * CX[q];
    int s2y = y - 2 * CY[q];
    int s2z = z - 2 * CZ[q];
    if (s2x < 0 || s2y < 0 || s2z < 0 || s2x >= nx || s2y >= ny || s2z >= nz) return bounced;

    int src2 = (s2x * ny + s2y) * nz + s2z;
    if (payload[src2 * in_ch + 0] > 0.5f) return bounced;

    float upstream = f_read[q * cells + src2];
    return bounced + blend * (upstream - bounced);
}

kernel void init_distributions(
    __global const float* payload, int in_ch, int cells,
    __global float* f, __global float* f_post
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;

    int base = cell * in_ch;
    float rho = 1.0f, ux = 0.0f, uy = 0.0f, uz = 0.0f;
    if (payload[base + 0] < 0.5f) {
        rho = clampf(1.0f + clampf(payload[base + 8], P_MIN, P_MAX), RHO_MIN, RHO_MAX);
        ux = payload[base + 5]; uy = payload[base + 6]; uz = payload[base + 7];
    }

    for (int q = 0; q < KQ; ++q) {
        float eq = feq(q, rho, ux, uy, uz);
        f[q * cells + cell] = eq;
        f_post[q * cells + cell] = eq;
    }
}

)CLC"
R"CLC(
kernel void stream_collide_step(
    __global const float* f_read,
    __global const float* payload,
    __global const float* temp_read,
    int in_ch, int nx, int ny, int nz, int cells, int tick, int benchmark_flags, int hydro_periodic_mask, int thermal_periodic_mask,
    float tau_shear_eff, float tau_normal_eff,
    int x_min_kind, int x_max_kind, int y_min_kind, int y_max_kind, int z_min_kind, int z_max_kind,
    float4 x_min_data, float4 x_max_data, float4 y_min_data, float4 y_max_data, float4 z_min_data, float4 z_max_data,
    int tx_min_kind, int tx_max_kind, int ty_min_kind, int ty_max_kind, int tz_min_kind, int tz_max_kind,
    float4 tx_min_data, float4 tx_max_data, float4 ty_min_data, float4 ty_max_data, float4 tz_min_data, float4 tz_max_data,
    int benchmark_preset,
    __global float* f_write,
    __global float* temp_write
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;

    int yz = ny * nz;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / nz;
    int z = rem - y * nz;
    
    int base = cell * in_ch;
    int is_solid = payload[base + 0] > 0.5f;

    float f_local[KQ];

    for (int q = 0; q < KQ; ++q) {
        int opp = OPP[q];
        if (is_solid) {
            f_local[q] = f_read[opp * cells + cell]; 
            continue;
        }

        int sx = x - CX[q], sy = y - CY[q], sz = z - CZ[q];
        sx = wrap_axis_if_periodic(sx, nx, PERIODIC_AXIS_X, hydro_periodic_mask);
        sy = wrap_axis_if_periodic(sy, ny, PERIODIC_AXIS_Y, hydro_periodic_mask);
        sz = wrap_axis_if_periodic(sz, nz, PERIODIC_AXIS_Z, hydro_periodic_mask);
        if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz) {
            f_local[q] = benchmark_boundary_value(
                f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, sx, sy, sz, benchmark_flags,
                x_min_kind, x_max_kind, y_min_kind, y_max_kind, z_min_kind, z_max_kind,
                x_min_data, x_max_data, y_min_data, y_max_data, z_min_data, z_max_data, benchmark_preset
            );
        } else {
            int src = (sx * ny + sy) * nz + sz;
            if (payload[src * in_ch + 0] > 0.5f) {
                f_local[q] = obstacle_bounce_value(f_read, payload, in_ch, nx, ny, nz, cells, cell, x, y, z, q, opp, benchmark_flags);
            } else {
                f_local[q] = f_read[q * cells + src];
            }
        }
    }

    if (is_solid) {
        temp_write[cell] = 0.0f;
        for (int q = 0; q < KQ; ++q) f_write[q * cells + cell] = f_local[q];
        return;
    }

    float rho = 0.0f, mx = 0.0f, my = 0.0f, mz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_local[q];
        rho += fq;
        mx += fq * (float)CX[q]; my += fq * (float)CY[q]; mz += fq * (float)CZ[q];
    }

    float rho_safe = fmax(1e-6f, rho);
    float inv_rho = 1.0f / rho_safe;
    float ux = mx * inv_rho;
    float uy = my * inv_rho;
    float uz = mz * inv_rho;
    float speed_pre = sqrt(ux * ux + uy * uy + uz * uz);

    float fan = (benchmark_flags & BENCH_DISABLE_FAN_FORCING) ? 0.0f : clampf(payload[base + 1], 0.0f, 1.0f);
    float fan_ux = payload[base + 2];
    float fan_uy = payload[base + 3];
    float fan_uz = payload[base + 4];
    float fan_norm = sqrt(fan_ux * fan_ux + fan_uy * fan_uy + fan_uz * fan_uz);

    float temp_center = temp_read[cell];
    float temp_next = temp_center;
    int update_temperature = BOUSSINESQ_ENABLED
        && (THERMAL_UPDATE_STRIDE <= 1 || (tick % THERMAL_UPDATE_STRIDE) == 0);
    if (update_temperature) {
        float temp_advected = sample_temperature_trilinear(
            temp_read,
            payload,
            in_ch,
            nx,
            ny,
            nz,
            thermal_periodic_mask,
            tx_min_kind,
            tx_max_kind,
            ty_min_kind,
            ty_max_kind,
            tz_min_kind,
            tz_max_kind,
            tx_min_data,
            tx_max_data,
            ty_min_data,
            ty_max_data,
            tz_min_data,
            tz_max_data,
            (float)x - ux,
            (float)y - uy,
            (float)z - uz,
            temp_center
        );
        float txp = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x + 1, y, z, temp_center);
        float txm = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x - 1, y, z, temp_center);
        float typ = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y + 1, z, temp_center);
        float tym = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y - 1, z, temp_center);
        float tzp = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y, z + 1, temp_center);
        float tzm = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y, z - 1, temp_center);

        float laplacian_t = (txp + txm + typ + tym + tzp + tzm) - 6.0f * temp_center;
        float thermal_source = 0.0f;
        if ((benchmark_flags & BENCH_DISABLE_INTERNAL_THERMAL_SOURCE) != 0) {
            thermal_source = 0.0f;
        } else if (in_ch > 9) {
            thermal_source = clampf(payload[base + 9], -THERMAL_SOURCE_MAX, THERMAL_SOURCE_MAX);
        } else if (fan > 0.0f && fan_norm > 1e-8f) {
            thermal_source = fan * clampf(fan_norm * THERMAL_SOURCE_SCALE, 0.0f, THERMAL_SOURCE_MAX);
        }
        temp_next = clampf(
            temp_advected + THERMAL_DIFFUSIVITY * laplacian_t + thermal_source - THERMAL_COOLING * temp_advected,
            THERMAL_MIN,
            THERMAL_MAX
        );
    }
    temp_write[cell] = temp_next;

    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
    if (fan > 0.0f) {
        if (fan_norm > 1e-8f) {
            float inv_norm = 1.0f / fan_norm;
            float noise_amp = (benchmark_flags & BENCH_DISABLE_FAN_NOISE) ? 0.0f : FAN_NOISE_AMP;
            float noise = 1.0f + noise_amp * signed_noise((uint)cell, (uint)tick);
            float target_speed = clampf(
                fan_norm * FAN_TARGET_SCALE * fmax(0.0f, noise),
                0.0f,
                FAN_TARGET_MAX
            );
            float nx = fan_ux * inv_norm;
            float ny = fan_uy * inv_norm;
            float nz = fan_uz * inv_norm;
            float u_para = ux * nx + uy * ny + uz * nz;
            float u_perp_x = ux - u_para * nx;
            float u_perp_y = uy - u_para * ny;
            float u_perp_z = uz - u_para * nz;
            float axial_push = fmax(0.0f, target_speed - u_para);
            float speed_damp = 1.0f;
            if (speed_pre > FAN_SPEED_SOFT_CAP) {
                float r = (speed_pre - FAN_SPEED_SOFT_CAP) / fmax(1e-4f, FAN_SPEED_DAMP_WIDTH);
                speed_damp = 1.0f / (1.0f + r * r);
            }
            float beta = fan * FAN_BETA * speed_damp;
            fx = beta * rho_safe * (axial_push * nx - FAN_PERP_DAMP * u_perp_x);
            fy = beta * rho_safe * (axial_push * ny - FAN_PERP_DAMP * u_perp_y);
            fz = beta * rho_safe * (axial_push * nz - FAN_PERP_DAMP * u_perp_z);
        }
    }

    if (BOUSSINESQ_ENABLED && (benchmark_flags & BENCH_DISABLE_BUOYANCY) == 0) {
        float buoyancy = clampf(BOUSSINESQ_BETA * temp_next, -BOUSSINESQ_FORCE_MAX, BOUSSINESQ_FORCE_MAX);
        fy += rho_safe * buoyancy;
    }

    float dux = fx * inv_rho;
    float duy = fy * inv_rho;
    float duz = fz * inv_rho;
    ux += 0.5f * dux; uy += 0.5f * duy; uz += 0.5f * duz;
    ux = mix(ux, payload[base + 5], STATE_NUDGE);
    uy = mix(uy, payload[base + 6], STATE_NUDGE);
    uz = mix(uz, payload[base + 7], STATE_NUDGE);

    float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > MAX_SPEED * MAX_SPEED) {
        float scale = MAX_SPEED * rsqrt(speed2);
        ux *= scale; uy *= scale; uz *= scale;
    }

    float raw[27];
    float central_pre[27];
    float central_post[27];
    float raw_post[27];
    for (int i = 0; i < 27; ++i) {
        raw[i] = 0.0f;
        central_pre[i] = 0.0f;
        central_post[i] = 0.0f;
        raw_post[i] = 0.0f;
    }

    for (int q = 0; q < KQ; ++q) {
        float fq = f_local[q];
        float cx = (float)CX[q], cy = (float)CY[q], cz = (float)CZ[q];
        float px[3] = {1.0f, cx, cx * cx};
        float py[3] = {1.0f, cy, cy * cy};
        float pz[3] = {1.0f, cz, cz * cz};
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) {
                for (int c = 0; c < 3; ++c) {
                    raw[MI(a, b, c)] += fq * px[a] * py[b] * pz[c];
                }
            }
        }
    }

    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int c = 0; c < 3; ++c) {
                float sum = 0.0f;
                for (int p = 0; p <= a; ++p) {
                    for (int qm = 0; qm <= b; ++qm) {
                        for (int rm = 0; rm <= c; ++rm) {
                            float sx = (a - p == 0) ? 1.0f : ((a - p == 1) ? -ux : ux * ux);
                            float sy = (b - qm == 0) ? 1.0f : ((b - qm == 1) ? -uy : uy * uy);
                            float sz = (c - rm == 0) ? 1.0f : ((c - rm == 1) ? -uz : uz * uz);
                            float coeff = (float)(binom(a, p) * binom(b, qm) * binom(c, rm));
                            sum += coeff * sx * sy * sz * raw[MI(p, qm, rm)];
                        }
                    }
                }
                central_pre[MI(a, b, c)] = sum;
            }
        }
    }

)CLC"
R"CLC(
    float tau_shear_local = tau_shear_eff;
    float tau_normal_local = clampf(tau_normal_eff, TAU_NORMAL_MIN, TAU_NORMAL_MAX);
    if (SGS_ENABLED && (benchmark_flags & BENCH_DISABLE_SGS) == 0) {
        float nu0 = fmax(1e-6f, (TAU_SHEAR - 0.5f) / 3.0f);
        float neq_xx = central_pre[MI(2, 0, 0)] - rho_safe * CS2;
        float neq_yy = central_pre[MI(0, 2, 0)] - rho_safe * CS2;
        float neq_zz = central_pre[MI(0, 0, 2)] - rho_safe * CS2;
        float neq_xy = central_pre[MI(1, 1, 0)];
        float neq_xz = central_pre[MI(1, 0, 1)];
        float neq_yz = central_pre[MI(0, 1, 1)];
        float q_norm2 = neq_xx * neq_xx + neq_yy * neq_yy + neq_zz * neq_zz
                        + 2.0f * (neq_xy * neq_xy + neq_xz * neq_xz + neq_yz * neq_yz);
        float q_mag = sqrt(fmax(0.0f, q_norm2));
        float s_mag = q_mag / fmax(1e-6f, 2.0f * rho_safe * nu0);
        float nu_t = SGS_C2 * s_mag;
        nu_t = fmin(nu_t, SGS_NUT_TO_NU0_MAX * nu0);
        tau_shear_local = clampf(0.5f + 3.0f * (nu0 + nu_t), TAU_SHEAR_MIN, TAU_SHEAR_MAX);
        tau_normal_local = clampf(
            TAU_NORMAL + (tau_shear_local - TAU_SHEAR) * SGS_BULK_COUPLING,
            TAU_NORMAL_MIN,
            TAU_NORMAL_MAX
        );
    }

    float omega_diag = 1.0f / tau_normal_local;
    float omega_offdiag = 1.0f / tau_shear_local;
    float s_diag = clampf(omega_diag, 0.0f, 1.95f);
    float s_offdiag = clampf(omega_offdiag, 0.0f, 1.95f);
    float eq_second = rho_safe * CS2;

    central_post[MI(0, 0, 0)] = rho_safe;
    central_post[MI(2, 0, 0)] = central_pre[MI(2, 0, 0)] + s_diag * (eq_second - central_pre[MI(2, 0, 0)]);
    central_post[MI(0, 2, 0)] = central_pre[MI(0, 2, 0)] + s_diag * (eq_second - central_pre[MI(0, 2, 0)]);
    central_post[MI(0, 0, 2)] = central_pre[MI(0, 0, 2)] + s_diag * (eq_second - central_pre[MI(0, 0, 2)]);
    central_post[MI(1, 1, 0)] = (1.0f - s_offdiag) * central_pre[MI(1, 1, 0)];
    central_post[MI(1, 0, 1)] = (1.0f - s_offdiag) * central_pre[MI(1, 0, 1)];
    central_post[MI(0, 1, 1)] = (1.0f - s_offdiag) * central_pre[MI(0, 1, 1)];

    float c200 = central_post[MI(2, 0, 0)];
    float c020 = central_post[MI(0, 2, 0)];
    float c002 = central_post[MI(0, 0, 2)];
    float c110 = central_post[MI(1, 1, 0)];
    float c101 = central_post[MI(1, 0, 1)];
    float c011 = central_post[MI(0, 1, 1)];
    float inv_rho_safe = 1.0f / fmax(1e-6f, rho_safe);

    central_post[MI(2, 2, 0)] = c200 * c020 * inv_rho_safe + 2.0f * c110 * c110 * inv_rho_safe;
    central_post[MI(2, 0, 2)] = c200 * c002 * inv_rho_safe + 2.0f * c101 * c101 * inv_rho_safe;
    central_post[MI(0, 2, 2)] = c020 * c002 * inv_rho_safe + 2.0f * c011 * c011 * inv_rho_safe;
    central_post[MI(2, 1, 1)] = (c200 * c011 + 2.0f * c110 * c101) * inv_rho_safe;
    central_post[MI(1, 2, 1)] = (c020 * c101 + 2.0f * c110 * c011) * inv_rho_safe;
    central_post[MI(1, 1, 2)] = (c002 * c110 + 2.0f * c101 * c011) * inv_rho_safe;
    central_post[MI(2, 2, 2)] = (c200 * c020 * c002
                                + 2.0f * c110 * c110 * c002
                                + 2.0f * c101 * c101 * c020
                                + 2.0f * c011 * c011 * c200
                                + 8.0f * c110 * c101 * c011) * inv_rho_safe * inv_rho_safe;

    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int c = 0; c < 3; ++c) {
                float sum = 0.0f;
                for (int p = 0; p <= a; ++p) {
                    for (int qm = 0; qm <= b; ++qm) {
                        for (int rm = 0; rm <= c; ++rm) {
                            float ux_pow = (a - p == 0) ? 1.0f : ((a - p == 1) ? ux : ux * ux);
                            float uy_pow = (b - qm == 0) ? 1.0f : ((b - qm == 1) ? uy : uy * uy);
                            float uz_pow = (c - rm == 0) ? 1.0f : ((c - rm == 1) ? uz : uz * uz);
                            float coeff = (float)(binom(a, p) * binom(b, qm) * binom(c, rm));
                            sum += coeff * ux_pow * uy_pow * uz_pow * central_post[MI(p, qm, rm)];
                        }
                    }
                }
                raw_post[MI(a, b, c)] = sum;
            }
        }
    }

    float f_post_local[KQ];
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            for (int iz = 0; iz < 3; ++iz) {
                float sum = 0.0f;
                for (int a = 0; a < 3; ++a) {
                    for (int b = 0; b < 3; ++b) {
                        for (int c = 0; c < 3; ++c) {
                            sum += TINV[ix][a] * TINV[iy][b] * TINV[iz][c] * raw_post[MI(a, b, c)];
                        }
                    }
                }
                int q = (ix * 3 + iy) * 3 + iz;
                f_post_local[q] = sum;
            }
        }
    }

    float rho_corr = 0.0f, mx_corr = 0.0f, my_corr = 0.0f, mz_corr = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_post_local[q];
        rho_corr += fq;
        mx_corr += fq * (float)CX[q];
        my_corr += fq * (float)CY[q];
        mz_corr += fq * (float)CZ[q];
    }
    float inv_rho_corr = 1.0f / fmax(1e-6f, rho_corr);
    float ux_corr = mx_corr * inv_rho_corr;
    float uy_corr = my_corr * inv_rho_corr;
    float uz_corr = mz_corr * inv_rho_corr;
    for (int q = 0; q < KQ; ++q) {
        f_post_local[q] += feq(q, rho_safe, ux, uy, uz) - feq(q, fmax(1e-6f, rho_corr), ux_corr, uy_corr, uz_corr);
    }

    float ux_minus = ux - 0.5f * dux;
    float uy_minus = uy - 0.5f * duy;
    float uz_minus = uz - 0.5f * duz;
    float ux_plus = ux + 0.5f * dux;
    float uy_plus = uy + 0.5f * duy;
    float uz_plus = uz + 0.5f * duz;
    float alpha_sponge = (benchmark_flags & BENCH_DISABLE_SPONGE) ? 0.0f : sponge_alpha(nx, ny, nz, x, y, z);
    float keep_sponge = 1.0f - alpha_sponge;

    for (int q = 0; q < KQ; ++q) {
        float edm_source = feq(q, rho_safe, ux_plus, uy_plus, uz_plus) - feq(q, rho_safe, ux_minus, uy_minus, uz_minus);
        float f_next = f_post_local[q] + edm_source;
        if (alpha_sponge > 0.0f) {
            float f_far = feq(q, 1.0f, 0.0f, 0.0f, 0.0f);
            f_next = keep_sponge * f_next + alpha_sponge * f_far;
        }
        f_write[q * cells + cell] = f_next;
    }
}

)CLC"
R"CLC(
kernel void output_macro(
    __global const float* f, __global const float* payload,
    int in_ch, int out_ch, int cells, __global float* out
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;

    int out_base = cell * out_ch;
    if (payload[cell * in_ch + 0] > 0.5f) {
        for (int c = 0; c < out_ch; ++c) out[out_base + c] = 0.0f;
        return;
    }

    float rho = 0.0f, ux = 0.0f, uy = 0.0f, uz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f[q * cells + cell];
        rho += fq;
        ux += fq * (float)CX[q]; uy += fq * (float)CY[q]; uz += fq * (float)CZ[q];
    }

    rho = clampf(rho, RHO_MIN, RHO_MAX);
    float inv_rho = 1.0f / fmax(1e-6f, rho);
    float vx = ux * inv_rho;
    float vy = uy * inv_rho;
    float vz = uz * inv_rho;
    if (fabs(vx) < 1e-7f) vx = 0.0f;
    if (fabs(vy) < 1e-7f) vy = 0.0f;
    if (fabs(vz) < 1e-7f) vz = 0.0f;
    if (fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;

    out[out_base + 0] = vx;
    out[out_base + 1] = vy;
    out[out_base + 2] = vz;
    out[out_base + 3] = rho - 1.0f;
    for (int c = 4; c < out_ch; ++c) out[out_base + c] = 0.0f;
}
)CLC";

// const char* cl_error_to_string(cl_int err) {
//     switch (err) {
//         case CL_SUCCESS: return "CL_SUCCESS";
//         case CL_DEVICE_NOT_FOUND: return "CL_DEVICE_NOT_FOUND";
//         case CL_DEVICE_NOT_AVAILABLE: return "CL_DEVICE_NOT_AVAILABLE";
//         case CL_COMPILER_NOT_AVAILABLE: return "CL_COMPILER_NOT_AVAILABLE";
//         case CL_MEM_OBJECT_ALLOCATION_FAILURE: return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
//         case CL_OUT_OF_RESOURCES: return "CL_OUT_OF_RESOURCES";
//         case CL_OUT_OF_HOST_MEMORY: return "CL_OUT_OF_HOST_MEMORY";
//         case CL_BUILD_PROGRAM_FAILURE: return "CL_BUILD_PROGRAM_FAILURE";
//         case CL_INVALID_VALUE: return "CL_INVALID_VALUE";
//         case CL_INVALID_DEVICE: return "CL_INVALID_DEVICE";
//         case CL_INVALID_BINARY: return "CL_INVALID_BINARY";
//         case CL_INVALID_BUILD_OPTIONS: return "CL_INVALID_BUILD_OPTIONS";
//         case CL_INVALID_PROGRAM: return "CL_INVALID_PROGRAM";
//         case CL_INVALID_KERNEL_NAME: return "CL_INVALID_KERNEL_NAME";
//         case CL_INVALID_KERNEL_DEFINITION: return "CL_INVALID_KERNEL_DEFINITION";
//         case CL_INVALID_KERNEL: return "CL_INVALID_KERNEL";
//         case CL_INVALID_MEM_OBJECT: return "CL_INVALID_MEM_OBJECT";
//         case CL_INVALID_OPERATION: return "CL_INVALID_OPERATION";
//         case CL_INVALID_COMMAND_QUEUE: return "CL_INVALID_COMMAND_QUEUE";
//         case CL_INVALID_CONTEXT: return "CL_INVALID_CONTEXT";
//         default: return "CL_UNKNOWN_ERROR";
//     }
// }

std::string read_device_name(cl_device_id device) {
    if (device == nullptr) return "unknown";
    size_t bytes = 0;
    if (clGetDeviceInfo(device, CL_DEVICE_NAME, 0, nullptr, &bytes) != CL_SUCCESS || bytes == 0) return "unknown";
    std::string name(bytes, '\0');
    if (clGetDeviceInfo(device, CL_DEVICE_NAME, bytes, name.data(), nullptr) != CL_SUCCESS) return "unknown";
    if (!name.empty() && name.back() == '\0') name.pop_back();
    return name;
}

std::string read_program_build_log(cl_program program, cl_device_id device) {
    if (!program || !device) return {};
    size_t bytes = 0;
    if (clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &bytes) != CL_SUCCESS || bytes == 0) {
        return {};
    }
    std::string log(bytes, '\0');
    if (clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, bytes, log.data(), nullptr) != CL_SUCCESS) {
        return {};
    }
    while (!log.empty() && (log.back() == '\0' || log.back() == '\n' || log.back() == '\r')) {
        log.pop_back();
    }
    return log;
}

void release_opencl_runtime() {
    if (g_opencl.k_output) clReleaseKernel(g_opencl.k_output);
    if (g_opencl.k_stream_collide) clReleaseKernel(g_opencl.k_stream_collide);
    if (g_opencl.k_init) clReleaseKernel(g_opencl.k_init);
    if (g_opencl.program) clReleaseProgram(g_opencl.program);
    if (g_opencl.queue) clReleaseCommandQueue(g_opencl.queue);
    if (g_opencl.context) clReleaseContext(g_opencl.context);
    g_opencl = OpenClRuntime{};
}

void release_context_gpu_buffers(ContextState& ctx) {
    if (ctx.d_temp_next) clReleaseMemObject(ctx.d_temp_next);
    if (ctx.d_temp) clReleaseMemObject(ctx.d_temp);
    if (ctx.d_output) clReleaseMemObject(ctx.d_output);
    if (ctx.d_f_post) clReleaseMemObject(ctx.d_f_post);
    if (ctx.d_f) clReleaseMemObject(ctx.d_f);
    if (ctx.d_payload) clReleaseMemObject(ctx.d_payload);
    ctx.d_temp_next = ctx.d_temp = ctx.d_output = ctx.d_f_post = ctx.d_f = ctx.d_payload = nullptr;
    ctx.gpu_buffers_ready = ctx.gpu_initialized = false;
}

bool initialize_opencl_runtime() {
    if (g_opencl.available) return true;

    cl_uint platform_count = 0;
    cl_int err = clGetPlatformIDs(0, nullptr, &platform_count);
    if (err != CL_SUCCESS || platform_count == 0) {
        g_opencl.error = "No OpenCL platform"; return false;
    }

    std::vector<cl_platform_id> platforms(platform_count);
    err = clGetPlatformIDs(platform_count, platforms.data(), nullptr);
    if (err != CL_SUCCESS) return false;

    cl_device_id selected_device = nullptr;
    cl_platform_id selected_platform = nullptr;

    for (cl_platform_id platform : platforms) {
        if (clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &selected_device, nullptr) == CL_SUCCESS) {
            selected_platform = platform; break;
        }
    }
    if (!selected_device) {
        for (cl_platform_id platform : platforms) {
            if (clGetDeviceIDs(platform, CL_DEVICE_TYPE_DEFAULT, 1, &selected_device, nullptr) == CL_SUCCESS) {
                selected_platform = platform; break;
            }
        }
    }
    if (!selected_device) { g_opencl.error = "No usable OpenCL device"; return false; }

    cl_context context = clCreateContext(nullptr, 1, &selected_device, nullptr, nullptr, &err);
    if (err != CL_SUCCESS) return false;

    cl_command_queue queue = clCreateCommandQueue(context, selected_device, 0, &err);
    if (err != CL_SUCCESS) { clReleaseContext(context); return false; }

    const char* src = kOpenClSource;
    const size_t src_len = std::strlen(kOpenClSource);
    cl_program program = clCreateProgramWithSource(context, 1, &src, &src_len, &err);
    if (err != CL_SUCCESS) { clReleaseCommandQueue(queue); clReleaseContext(context); return false; }

    err = clBuildProgram(program, 1, &selected_device, "-cl-fast-relaxed-math", nullptr, nullptr);
    if (err != CL_SUCCESS) {
        std::string build_log = read_program_build_log(program, selected_device);
        clReleaseProgram(program); clReleaseCommandQueue(queue); clReleaseContext(context);
        const std::string err_text = "clBuildProgram failed (" + std::to_string(static_cast<int>(err)) + ")";
        g_opencl.error = build_log.empty() ? err_text
                                           : err_text + ": " + build_log;
        return false;
    }

    cl_kernel k_init = clCreateKernel(program, "init_distributions", &err);
    cl_kernel k_stream_collide = clCreateKernel(program, "stream_collide_step", &err);
    cl_kernel k_output = clCreateKernel(program, "output_macro", &err);

    if (!k_init || !k_stream_collide || !k_output) {
        if (k_init) clReleaseKernel(k_init);
        if (k_stream_collide) clReleaseKernel(k_stream_collide);
        if (k_output) clReleaseKernel(k_output);
        clReleaseProgram(program); clReleaseCommandQueue(queue); clReleaseContext(context);
        g_opencl.error = "Kernel creation failed"; return false;
    }

    g_opencl.context = context; g_opencl.queue = queue; g_opencl.program = program;
    g_opencl.k_init = k_init; g_opencl.k_stream_collide = k_stream_collide; g_opencl.k_output = k_output;
    g_opencl.platform = selected_platform; g_opencl.device = selected_device;
    g_opencl.available = true; g_opencl.device_name = read_device_name(selected_device);
    return true;
}

bool ensure_context_gpu_buffers(ContextState& ctx) {
    if (!g_opencl.available || ctx.cells == 0) return false;
    if (ctx.gpu_buffers_ready) return true;

    const std::size_t payload_bytes = ctx.cells * g_cfg.input_channels * sizeof(float);
    const std::size_t dist_bytes = ctx.cells * kQ * sizeof(float);
    const std::size_t output_bytes = ctx.cells * g_cfg.output_channels * sizeof(float);
    const std::size_t temp_bytes = ctx.cells * sizeof(float);

    cl_int err = CL_SUCCESS;
    ctx.d_payload = clCreateBuffer(g_opencl.context, CL_MEM_READ_ONLY, payload_bytes, nullptr, &err);
    ctx.d_f = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, dist_bytes, nullptr, &err);
    ctx.d_f_post = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, dist_bytes, nullptr, &err);
    ctx.d_output = clCreateBuffer(g_opencl.context, CL_MEM_WRITE_ONLY, output_bytes, nullptr, &err);
    ctx.d_temp = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, temp_bytes, nullptr, &err);
    ctx.d_temp_next = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, temp_bytes, nullptr, &err);

    if (!ctx.d_payload || !ctx.d_f || !ctx.d_f_post || !ctx.d_output || !ctx.d_temp || !ctx.d_temp_next) {
        release_context_gpu_buffers(ctx); return false;
    }
    ctx.gpu_buffers_ready = true; ctx.gpu_initialized = false;
    return true;
}

bool enqueue_kernel_1d(cl_kernel kernel, int cells) {
    const size_t global_size = static_cast<size_t>(cells);
    return clEnqueueNDRangeKernel(g_opencl.queue, kernel, 1, nullptr, &global_size, nullptr, 0, nullptr, nullptr) == CL_SUCCESS;
}

bool opencl_step(ContextState& ctx, const float* payload, float* out, StepTiming& timing) {
    if (!ensure_context_gpu_buffers(ctx)) return false;

    const int cells_i32 = static_cast<int>(ctx.cells);
    const std::size_t payload_bytes = ctx.cells * g_cfg.input_channels * sizeof(float);
    const std::size_t output_bytes = ctx.cells * g_cfg.output_channels * sizeof(float);

    if (benchmark_mode_active() && g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D) {
        if (ctx.obstacle.size() != ctx.cells) {
            ctx.obstacle.assign(ctx.cells, 0);
        }
        for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
            const std::size_t base = cell * g_cfg.input_channels;
            ctx.obstacle[cell] = payload[base + kChannelObstacle] > 0.5f ? 1 : 0;
        }
    }

    auto upload_begin = Clock::now();
    if (clEnqueueWriteBuffer(g_opencl.queue, ctx.d_payload, CL_TRUE, 0, payload_bytes, payload, 0, nullptr, nullptr) != CL_SUCCESS) return false;
    timing.payload_copy_ms += elapsed_ms(upload_begin, Clock::now());

    auto solver_begin = Clock::now();
    cl_int err = CL_SUCCESS;
    if (!ctx.gpu_initialized) {
        err |= clSetKernelArg(g_opencl.k_init, 0, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_init, 1, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_init, 2, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_init, 3, sizeof(cl_mem), &ctx.d_f);
        err |= clSetKernelArg(g_opencl.k_init, 4, sizeof(cl_mem), &ctx.d_f_post);
        if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_init, cells_i32)) return false;

        const std::size_t temp_bytes = ctx.cells * sizeof(float);
        if (clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr) != CL_SUCCESS) {
            return false;
        }
        if (clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp_next, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr) != CL_SUCCESS) {
            return false;
        }
        ctx.gpu_initialized = true;
    }

    const int tick_i32 = static_cast<int>(ctx.step_counter & 0x7FFFFFFF);
    const int benchmark_flags = opencl_benchmark_flags();
    const int hydro_periodic_mask = opencl_hydrodynamic_periodic_mask();
    const int thermal_periodic_mask = opencl_thermal_periodic_mask();
    const int benchmark_preset = benchmark_mode_active() ? g_benchmark_cfg.preset : 0;
    const auto hydro_face_kinds = opencl_hydrodynamic_face_kinds();
    const auto hydro_face_data = opencl_hydrodynamic_face_data();
    const auto thermal_face_kinds = opencl_thermal_face_kinds();
    const auto thermal_face_data = opencl_thermal_face_data();
    const auto tau_pair = opencl_effective_tau_pair();
    
    // Ping-Pong 双重缓冲交换
    cl_mem read_buf = (ctx.step_counter % 2 == 0) ? ctx.d_f : ctx.d_f_post;
    cl_mem write_buf = (ctx.step_counter % 2 == 0) ? ctx.d_f_post : ctx.d_f;
    cl_mem temp_read = (ctx.step_counter % 2 == 0) ? ctx.d_temp : ctx.d_temp_next;
    cl_mem temp_write = (ctx.step_counter % 2 == 0) ? ctx.d_temp_next : ctx.d_temp;

    err |= clSetKernelArg(g_opencl.k_stream_collide, 0, sizeof(cl_mem), &read_buf);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 1, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 2, sizeof(cl_mem), &temp_read);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 3, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 4, sizeof(int), &ctx.nx);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 5, sizeof(int), &ctx.ny);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 6, sizeof(int), &ctx.nz);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 7, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 8, sizeof(int), &tick_i32);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 9, sizeof(int), &benchmark_flags);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 10, sizeof(int), &hydro_periodic_mask);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 11, sizeof(int), &thermal_periodic_mask);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 12, sizeof(float), &tau_pair[0]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 13, sizeof(float), &tau_pair[1]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 14, sizeof(int), &hydro_face_kinds[0]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 15, sizeof(int), &hydro_face_kinds[1]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 16, sizeof(int), &hydro_face_kinds[2]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 17, sizeof(int), &hydro_face_kinds[3]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 18, sizeof(int), &hydro_face_kinds[4]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 19, sizeof(int), &hydro_face_kinds[5]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 20, sizeof(OpenClFaceData), hydro_face_data[0].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 21, sizeof(OpenClFaceData), hydro_face_data[1].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 22, sizeof(OpenClFaceData), hydro_face_data[2].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 23, sizeof(OpenClFaceData), hydro_face_data[3].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 24, sizeof(OpenClFaceData), hydro_face_data[4].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 25, sizeof(OpenClFaceData), hydro_face_data[5].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 26, sizeof(int), &thermal_face_kinds[0]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 27, sizeof(int), &thermal_face_kinds[1]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 28, sizeof(int), &thermal_face_kinds[2]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 29, sizeof(int), &thermal_face_kinds[3]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 30, sizeof(int), &thermal_face_kinds[4]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 31, sizeof(int), &thermal_face_kinds[5]);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 32, sizeof(OpenClFaceData), thermal_face_data[0].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 33, sizeof(OpenClFaceData), thermal_face_data[1].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 34, sizeof(OpenClFaceData), thermal_face_data[2].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 35, sizeof(OpenClFaceData), thermal_face_data[3].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 36, sizeof(OpenClFaceData), thermal_face_data[4].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 37, sizeof(OpenClFaceData), thermal_face_data[5].data());
    err |= clSetKernelArg(g_opencl.k_stream_collide, 38, sizeof(int), &benchmark_preset);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 39, sizeof(cl_mem), &write_buf);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 40, sizeof(cl_mem), &temp_write);
    if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_stream_collide, cells_i32)) return false;

    err |= clSetKernelArg(g_opencl.k_output, 0, sizeof(cl_mem), &write_buf);
    err |= clSetKernelArg(g_opencl.k_output, 1, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_output, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_output, 3, sizeof(int), &g_cfg.output_channels);
    err |= clSetKernelArg(g_opencl.k_output, 4, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_output, 5, sizeof(cl_mem), &ctx.d_output);
    if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_output, cells_i32)) return false;

    clFinish(g_opencl.queue);
    timing.solver_ms += elapsed_ms(solver_begin, Clock::now());

    auto readback_begin = Clock::now();
    if (clEnqueueReadBuffer(g_opencl.queue, ctx.d_output, CL_TRUE, 0, output_bytes, out, 0, nullptr, nullptr) != CL_SUCCESS) return false;
    if (benchmark_mode_active() && g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D) {
        const std::size_t dist_bytes = ctx.cells * kQ * sizeof(float);
        if (ctx.f.size() != ctx.cells * kQ) {
            ctx.f.assign(ctx.cells * kQ, 0.0f);
        }
        if (clEnqueueReadBuffer(g_opencl.queue, write_buf, CL_TRUE, 0, dist_bytes, ctx.f.data(), 0, nullptr, nullptr) != CL_SUCCESS) {
            return false;
        }
        compute_benchmark_force_from_distributions(ctx, ctx.f.data());
    } else {
        ctx.last_force[0] = 0.0f;
        ctx.last_force[1] = 0.0f;
        ctx.last_force[2] = 0.0f;
    }
    timing.readback_ms += elapsed_ms(readback_begin, Clock::now());
    
    ctx.step_counter += 1;
    return true;
}

bool sync_context_temperature_from_gpu(ContextState& ctx) {
    if (!ctx.gpu_buffers_ready || !ctx.gpu_initialized) {
        return true;
    }
    if (ctx.temperature.size() != ctx.cells) {
        ctx.temperature.assign(ctx.cells, 0.0f);
        ctx.temperature_next.assign(ctx.cells, 0.0f);
    }
    cl_mem current_temp = (ctx.step_counter % 2 == 0) ? ctx.d_temp : ctx.d_temp_next;
    const std::size_t temp_bytes = ctx.cells * sizeof(float);
    if (clEnqueueReadBuffer(g_opencl.queue, current_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr) != CL_SUCCESS) {
        return false;
    }
    std::copy(ctx.temperature.begin(), ctx.temperature.end(), ctx.temperature_next.begin());
    return true;
}

bool sync_context_temperature_to_gpu(ContextState& ctx) {
    if (!ctx.gpu_buffers_ready || !ctx.gpu_initialized) {
        return true;
    }
    if (ctx.temperature.size() != ctx.cells) {
        ctx.temperature.assign(ctx.cells, 0.0f);
    }
    if (ctx.temperature_next.size() != ctx.cells) {
        ctx.temperature_next.assign(ctx.cells, 0.0f);
    }
    const std::size_t temp_bytes = ctx.cells * sizeof(float);
    if (clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr) != CL_SUCCESS) {
        return false;
    }
    if (clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp_next, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr) != CL_SUCCESS) {
        return false;
    }
    return true;
}

#else
struct OpenClRuntime { bool available = false; std::string error; std::string device_name; };
OpenClRuntime g_opencl;
void release_context_gpu_buffers(ContextState&) {}
void release_opencl_runtime() {}
bool initialize_opencl_runtime() { g_opencl.error = "Disabled"; return false; }
bool opencl_step(ContextState&, const float*, float*, StepTiming&) { return false; }
bool sync_context_temperature_from_gpu(ContextState&) { return true; }
bool sync_context_temperature_to_gpu(ContextState&) { return true; }
#endif

void clear_context(ContextState& ctx) { release_context_gpu_buffers(ctx); ctx = ContextState{}; }
void clear_all_contexts() { for (auto& e : g_contexts) clear_context(e.second); g_contexts.clear(); }
void reset_runtime_state() { clear_all_contexts(); release_opencl_runtime(); g_cfg = Config{}; reset_timing_stats(); }

void disable_opencl_runtime(const std::string& reason) {
    for (auto& e : g_contexts) release_context_gpu_buffers(e.second);
    release_opencl_runtime();
    g_cfg.opencl_enabled = false;
    g_cfg.runtime_info = "cpu|cumulant-d3q27+sgs+bouss (" + reason + ")";
}

bool should_force_cpu_backend() {
#if defined(_MSC_VER)
    char* env_buf = nullptr;
    size_t env_len = 0;
    if (_dupenv_s(&env_buf, &env_len, "AERO_LBM_CPU_ONLY") != 0 || env_buf == nullptr) {
        return false;
    }
    std::string env_value(env_buf);
    std::free(env_buf);
    std::transform(env_value.begin(), env_value.end(), env_value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return env_value == "1" || env_value == "true";
#else
    const char* env = std::getenv("AERO_LBM_CPU_ONLY");
    return env && (std::strcmp(env, "1") == 0 || std::strcmp(env, "true") == 0 || std::strcmp(env, "TRUE") == 0);
#endif
}

void ensure_context_shape(ContextState& ctx, int nx, int ny, int nz, std::size_t cells) {
    if (ctx.nx == nx && ctx.ny == ny && ctx.nz == nz && ctx.cells == cells) return;
    clear_context(ctx);
    ctx.nx = nx;
    ctx.ny = ny;
    ctx.nz = nz;
    ctx.cells = cells;
}

void ensure_context_temperature_storage(ContextState& ctx) {
    if (ctx.temperature.size() != ctx.cells) {
        ctx.temperature.assign(ctx.cells, 0.0f);
    }
    if (ctx.temperature_next.size() != ctx.cells) {
        ctx.temperature_next.assign(ctx.cells, 0.0f);
    }
}

void assign_temperature_state(ContextState& ctx, const float* temperature_state) {
    ensure_context_temperature_storage(ctx);
    for (std::size_t i = 0; i < ctx.cells; ++i) {
        ctx.temperature[i] = clampf(temperature_state[i], kThermalMin, kThermalMax);
    }
    std::copy(ctx.temperature.begin(), ctx.temperature.end(), ctx.temperature_next.begin());
}

void run_cpu_step(ContextState& ctx, const float* packet, float* out) {
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);
    ingest_payload(ctx, packet, g_cfg.input_channels);
    if (!ctx.cpu_initialized) initialize_distributions(ctx);
    if (should_update_temperature(ctx.step_counter)) {
        update_temperature_field(ctx);
    }
    collide(ctx);
    compute_benchmark_force(ctx);
    stream_and_bounce(ctx);
    write_output(ctx, out, g_cfg.output_channels);
    ctx.step_counter += 1;
}

bool run_solver_step(ContextState& ctx, const float* packet, float* out, StepTiming& timing) {
    bool ok = false;
    if (g_cfg.opencl_enabled && benchmark_opencl_supported()) {
        if (!(ok = opencl_step(ctx, packet, out, timing))) disable_opencl_runtime("OpenCL fail");
    }
    if (!ok) {
        auto solver_begin = Clock::now();
        run_cpu_step(ctx, packet, out);
        timing.solver_ms += elapsed_ms(solver_begin, Clock::now());
        ok = true;
    }
    return ok;
}

}  // namespace

extern "C" {

static std::size_t configured_cells() {
    return static_cast<std::size_t>(g_cfg.nx) * g_cfg.ny * g_cfg.nz;
}

static jboolean native_init_dims_impl(jint nx, jint ny, jint nz, jint input_channels, jint output_channels) {
    if (nx <= 0 || ny <= 0 || nz <= 0 || input_channels < 9 || output_channels < 4) {
        reset_runtime_state();
        return JNI_FALSE;
    }
    clear_all_contexts(); reset_timing_stats();
    g_cfg.grid_size = (nx == ny && ny == nz) ? nx : 0;
    g_cfg.nx = nx;
    g_cfg.ny = ny;
    g_cfg.nz = nz;
    g_cfg.input_channels = input_channels;
    g_cfg.output_channels = output_channels;
    g_cfg.initialized = true;

    if (should_force_cpu_backend()) {
        g_cfg.opencl_enabled = false;
        g_cfg.runtime_info = "cpu|cumulant-d3q27+sgs+bouss (forced)";
        return JNI_TRUE;
    }
    g_cfg.opencl_enabled = initialize_opencl_runtime();
    if (g_cfg.opencl_enabled) {
        g_cfg.runtime_info = "opencl|cumulant-d3q27+sgs+bouss:" + g_opencl.device_name;
    } else {
        g_cfg.runtime_info = "cpu|cumulant-d3q27+sgs+bouss (" + g_opencl.error + ")";
    }
    return JNI_TRUE;
}

static jboolean native_init_impl(jint grid_size, jint input_channels, jint output_channels) {
    return native_init_dims_impl(grid_size, grid_size, grid_size, input_channels, output_channels);
}

static bool native_step_raw_dims_impl(const float* packet, jint nx, jint ny, jint nz, jlong context_key, float* output_flow) {
    auto tick_begin = Clock::now();
    StepTiming timing;

    if (!g_cfg.initialized || !packet || !output_flow) return false;
    if (nx != g_cfg.nx || ny != g_cfg.ny || nz != g_cfg.nz) return false;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;

    ContextState& ctx = g_contexts[context_key];
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);

    const bool ok = run_solver_step(ctx, packet, output_flow, timing);
    timing.total_ms = elapsed_ms(tick_begin, Clock::now());
    record_timing(timing);
    return ok;
}

static jboolean native_step_impl(
    JNIEnv* env, jclass, jbyteArray payload, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    auto tick_begin = Clock::now();
    StepTiming timing;

    if (!g_cfg.initialized || !payload || !output_flow || grid_size != g_cfg.grid_size) return JNI_FALSE;
    const std::size_t cells = configured_cells();
    const std::size_t payload_bytes = cells * g_cfg.input_channels * sizeof(float);
    if (env->GetArrayLength(payload) != static_cast<jsize>(payload_bytes)) return JNI_FALSE;

    auto copy_begin = Clock::now();
    // jbyte* payload_bytes_ptr = env->GetByteArrayElements(payload, nullptr);
    // if (!payload_bytes_ptr) return JNI_FALSE;
    // std::vector<float> packet(cells * g_cfg.input_channels, 0.0f);
    // std::memcpy(packet.data(), payload_bytes_ptr, payload_bytes);
    // env->ReleaseByteArrayElements(payload, payload_bytes_ptr, JNI_ABORT);
    // timing.payload_copy_ms += elapsed_ms(copy_begin, Clock::now());

    jfloat* out = env->GetFloatArrayElements(output_flow, nullptr);
    if (!out) return JNI_FALSE;

    ContextState& ctx = g_contexts[context_key];
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    // Ensure CPU-side buffers exist and reuse packet buffer to avoid per-step allocations
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.packet.size() != cells * (std::size_t)g_cfg.input_channels) {
        ctx.packet.assign(cells * (std::size_t)g_cfg.input_channels, 0.0f);
    }
    jbyte* payload_bytes_ptr = env->GetByteArrayElements(payload, nullptr);
    if (!payload_bytes_ptr) {
        env->ReleaseFloatArrayElements(output_flow, out, 0);
        return JNI_FALSE;
    }
    std::memcpy(ctx.packet.data(), payload_bytes_ptr, payload_bytes);
    env->ReleaseByteArrayElements(payload, payload_bytes_ptr, JNI_ABORT);
    timing.payload_copy_ms += elapsed_ms(copy_begin, Clock::now());

    bool ok = run_solver_step(ctx, ctx.packet.data(), out, timing);

    env->ReleaseFloatArrayElements(output_flow, out, 0);
    timing.total_ms = elapsed_ms(tick_begin, Clock::now());
    record_timing(timing);
    return ok ? JNI_TRUE : JNI_FALSE;
}

static jboolean native_step_direct_impl(
    JNIEnv* env, jclass, jobject payload_buffer, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    auto tick_begin = Clock::now();
    StepTiming timing;

    if (!g_cfg.initialized || !payload_buffer || !output_flow || grid_size != g_cfg.grid_size) return JNI_FALSE;
    const std::size_t cells = configured_cells();
    const std::size_t payload_bytes = cells * g_cfg.input_channels * sizeof(float);

    void* payload_raw = env->GetDirectBufferAddress(payload_buffer);
    if (!payload_raw) return JNI_FALSE;
    const jlong payload_capacity = env->GetDirectBufferCapacity(payload_buffer);
    if (payload_capacity < 0 || static_cast<std::size_t>(payload_capacity) != payload_bytes) return JNI_FALSE;

    jfloat* out = env->GetFloatArrayElements(output_flow, nullptr);
    if (!out) return JNI_FALSE;

    ContextState& ctx = g_contexts[context_key];
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);

    const float* packet = reinterpret_cast<const float*>(payload_raw);
    bool ok = run_solver_step(ctx, packet, out, timing);

    env->ReleaseFloatArrayElements(output_flow, out, 0);
    timing.total_ms = elapsed_ms(tick_begin, Clock::now());
    record_timing(timing);
    return ok ? JNI_TRUE : JNI_FALSE;
}

static bool native_step_raw_impl(const float* packet, jint grid_size, jlong context_key, float* output_flow) {
    return native_step_raw_dims_impl(packet, grid_size, grid_size, grid_size, context_key, output_flow);
}

static bool native_get_temperature_state_raw_dims_impl(jint nx, jint ny, jint nz, jlong context_key, float* temperature_out) {
    if (!g_cfg.initialized || !temperature_out) return false;
    if (nx != g_cfg.nx || ny != g_cfg.ny || nz != g_cfg.nz) return false;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    auto it = g_contexts.find(context_key);
    if (it == g_contexts.end()) return false;
    ContextState& ctx = it->second;
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (g_cfg.opencl_enabled && !sync_context_temperature_from_gpu(ctx)) return false;
    if (ctx.temperature.size() != cells) return false;
    std::memcpy(temperature_out, ctx.temperature.data(), cells * sizeof(float));
    return true;
}

static jboolean native_get_temperature_state_impl(
    JNIEnv* env, jclass, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    if (!g_cfg.initialized || !temperature_state || grid_size != g_cfg.grid_size) return JNI_FALSE;
    const std::size_t cells = configured_cells();
    if (env->GetArrayLength(temperature_state) != static_cast<jsize>(cells)) return JNI_FALSE;

    auto it = g_contexts.find(context_key);
    if (it == g_contexts.end()) return JNI_FALSE;
    ContextState& ctx = it->second;
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (g_cfg.opencl_enabled && !sync_context_temperature_from_gpu(ctx)) return JNI_FALSE;
    if (ctx.temperature.size() != cells) return JNI_FALSE;

    env->SetFloatArrayRegion(temperature_state, 0, static_cast<jsize>(cells), ctx.temperature.data());
    return JNI_TRUE;
}

static jboolean native_set_temperature_state_impl(
    JNIEnv* env, jclass, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    if (!g_cfg.initialized || !temperature_state || grid_size != g_cfg.grid_size) return JNI_FALSE;
    const std::size_t cells = configured_cells();
    if (env->GetArrayLength(temperature_state) != static_cast<jsize>(cells)) return JNI_FALSE;

    ContextState& ctx = g_contexts[context_key];
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);
    jfloat* temperature_ptr = env->GetFloatArrayElements(temperature_state, nullptr);
    if (!temperature_ptr) return JNI_FALSE;
    assign_temperature_state(ctx, temperature_ptr);
    env->ReleaseFloatArrayElements(temperature_state, temperature_ptr, JNI_ABORT);
    if (g_cfg.opencl_enabled && !sync_context_temperature_to_gpu(ctx)) return JNI_FALSE;
    return JNI_TRUE;
}

static void native_release_context_impl(jlong context_key) {
    auto it = g_contexts.find(context_key);
    if (it != g_contexts.end()) { clear_context(it->second); g_contexts.erase(it); }
}

static void native_shutdown_impl() { reset_runtime_state(); }
static jstring native_runtime_info_impl(JNIEnv* env) {
    return env->NewStringUTF((g_cfg.runtime_info.empty() ? "uninitialized" : g_cfg.runtime_info).c_str());
}
static jstring native_timing_info_impl(JNIEnv* env) {
    return env->NewStringUTF(timing_info_string().c_str());
}

static void native_benchmark_default_config_impl(AeroLbmBenchmarkConfig* out_config) {
    if (!out_config) return;
    *out_config = make_default_benchmark_config();
}

static jboolean native_benchmark_default_preset_config_impl(jint preset, AeroLbmBenchmarkConfig* out_config) {
    if (!out_config || !valid_benchmark_preset(preset)) return JNI_FALSE;
    AeroLbmBenchmarkConfig cfg{};
    apply_benchmark_preset_defaults(cfg, preset);
    sanitize_benchmark_config(cfg);
    *out_config = cfg;
    return JNI_TRUE;
}

static jboolean native_benchmark_set_config_impl(const AeroLbmBenchmarkConfig* config) {
    if (!config) return JNI_FALSE;
    if (config->abi_version != AERO_LBM_BENCHMARK_ABI_VERSION) return JNI_FALSE;
    if (config->struct_size != sizeof(AeroLbmBenchmarkConfig)) return JNI_FALSE;
    g_benchmark_cfg = *config;
    sanitize_benchmark_config(g_benchmark_cfg);
    return JNI_TRUE;
}

static jboolean native_benchmark_get_config_impl(AeroLbmBenchmarkConfig* out_config) {
    if (!out_config) return JNI_FALSE;
    *out_config = g_benchmark_cfg;
    out_config->abi_version = AERO_LBM_BENCHMARK_ABI_VERSION;
    out_config->struct_size = sizeof(AeroLbmBenchmarkConfig);
    return JNI_TRUE;
}

static void native_benchmark_reset_config_impl() {
    g_benchmark_cfg = make_default_benchmark_config();
}

AERO_LBM_CAPI_EXPORT int aero_lbm_init(int grid_size, int input_channels, int output_channels) {
    return native_init_impl(grid_size, input_channels, output_channels) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_step(const float* packet, int grid_size, long long context_key, float* output_flow) {
    return native_step_raw_impl(packet, grid_size, static_cast<jlong>(context_key), output_flow) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_init_rect(int nx, int ny, int nz, int input_channels, int output_channels) {
    return native_init_dims_impl(nx, ny, nz, input_channels, output_channels) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_step_rect(const float* packet, int nx, int ny, int nz, long long context_key, float* output_flow) {
    return native_step_raw_dims_impl(packet, nx, ny, nz, static_cast<jlong>(context_key), output_flow) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_get_temperature_state_rect(int nx, int ny, int nz, long long context_key, float* out_temperature) {
    return native_get_temperature_state_raw_dims_impl(nx, ny, nz, static_cast<jlong>(context_key), out_temperature) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_get_last_force(long long context_key, float* out_fx, float* out_fy, float* out_fz) {
    const auto it = g_contexts.find(static_cast<jlong>(context_key));
    if (it == g_contexts.end()) return 0;
    const ContextState& ctx = it->second;
    if (out_fx) *out_fx = ctx.last_force[0];
    if (out_fy) *out_fy = ctx.last_force[1];
    if (out_fz) *out_fz = ctx.last_force[2];
    return 1;
}

AERO_LBM_CAPI_EXPORT void aero_lbm_release_context(long long context_key) {
    native_release_context_impl(static_cast<jlong>(context_key));
}

AERO_LBM_CAPI_EXPORT void aero_lbm_shutdown(void) {
    native_shutdown_impl();
}

AERO_LBM_CAPI_EXPORT const char* aero_lbm_runtime_info(void) {
    return g_cfg.runtime_info.empty() ? "uninitialized" : g_cfg.runtime_info.c_str();
}

AERO_LBM_CAPI_EXPORT const char* aero_lbm_timing_info(void) {
    static std::string timing_text;
    timing_text = timing_info_string();
    return timing_text.c_str();
}

AERO_LBM_CAPI_EXPORT void aero_lbm_reset_timing(void) {
    reset_timing_stats();
}

AERO_LBM_CAPI_EXPORT int aero_lbm_get_timing_snapshot(AeroLbmTimingSnapshot* out_snapshot) {
    if (!out_snapshot) return 0;
    const double inv = g_timing.ticks == 0 ? 0.0 : 1.0 / static_cast<double>(g_timing.ticks);
    out_snapshot->ticks = g_timing.ticks;
    out_snapshot->last_payload_copy_ms = g_timing.last.payload_copy_ms;
    out_snapshot->last_solver_ms = g_timing.last.solver_ms;
    out_snapshot->last_readback_ms = g_timing.last.readback_ms;
    out_snapshot->last_total_ms = g_timing.last.total_ms;
    out_snapshot->avg_payload_copy_ms = g_timing.payload_copy_ms_sum * inv;
    out_snapshot->avg_solver_ms = g_timing.solver_ms_sum * inv;
    out_snapshot->avg_readback_ms = g_timing.readback_ms_sum * inv;
    out_snapshot->avg_total_ms = g_timing.total_ms_sum * inv;
    return 1;
}

AERO_LBM_CAPI_EXPORT void aero_lbm_benchmark_default_config(AeroLbmBenchmarkConfig* out_config) {
    native_benchmark_default_config_impl(out_config);
}

AERO_LBM_CAPI_EXPORT int aero_lbm_benchmark_default_preset_config(int preset, AeroLbmBenchmarkConfig* out_config) {
    return native_benchmark_default_preset_config_impl(preset, out_config) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_benchmark_set_config(const AeroLbmBenchmarkConfig* config) {
    return native_benchmark_set_config_impl(config) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_benchmark_get_config(AeroLbmBenchmarkConfig* out_config) {
    return native_benchmark_get_config_impl(out_config) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT void aero_lbm_benchmark_reset_config(void) {
    native_benchmark_reset_config_impl();
}

AERO_LBM_CAPI_EXPORT const char* aero_lbm_benchmark_info(void) {
    static std::string benchmark_text;
    benchmark_text = benchmark_info_string(g_benchmark_cfg);
    return benchmark_text.c_str();
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeInit(
    JNIEnv*, jclass, jint grid_size, jint input_channels, jint output_channels
) {
    return native_init_impl(grid_size, input_channels, output_channels);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeInit(
    JNIEnv*, jclass, jint grid_size, jint input_channels, jint output_channels
) {
    return native_init_impl(grid_size, input_channels, output_channels);
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeStep(
    JNIEnv* env, jclass clazz, jbyteArray payload, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    return native_step_impl(env, clazz, payload, grid_size, context_key, output_flow);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeStep(
    JNIEnv* env, jclass clazz, jbyteArray payload, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    return native_step_impl(env, clazz, payload, grid_size, context_key, output_flow);
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeStepDirect(
    JNIEnv* env, jclass clazz, jobject payload, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    return native_step_direct_impl(env, clazz, payload, grid_size, context_key, output_flow);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeStepDirect(
    JNIEnv* env, jclass clazz, jobject payload, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    return native_step_direct_impl(env, clazz, payload, grid_size, context_key, output_flow);
}

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeReleaseContext(JNIEnv*, jclass, jlong context_key) {
    native_release_context_impl(context_key);
}
JNIEXPORT void JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeReleaseContext(JNIEnv*, jclass, jlong context_key) {
    native_release_context_impl(context_key);
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeGetTemperatureState(
    JNIEnv* env, jclass clazz, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    return native_get_temperature_state_impl(env, clazz, grid_size, context_key, temperature_state);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeGetTemperatureState(
    JNIEnv* env, jclass clazz, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    return native_get_temperature_state_impl(env, clazz, grid_size, context_key, temperature_state);
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeSetTemperatureState(
    JNIEnv* env, jclass clazz, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    return native_set_temperature_state_impl(env, clazz, grid_size, context_key, temperature_state);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeSetTemperatureState(
    JNIEnv* env, jclass clazz, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    return native_set_temperature_state_impl(env, clazz, grid_size, context_key, temperature_state);
}

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeShutdown(JNIEnv*, jclass) {
    native_shutdown_impl();
}
JNIEXPORT void JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeShutdown(JNIEnv*, jclass) {
    native_shutdown_impl();
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeRuntimeInfo(JNIEnv* env, jclass) {
    return native_runtime_info_impl(env);
}
JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeRuntimeInfo(JNIEnv* env, jclass) {
    return native_runtime_info_impl(env);
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeTimingInfo(JNIEnv* env, jclass) {
    return native_timing_info_impl(env);
}
JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeTimingInfo(JNIEnv* env, jclass) {
    return native_timing_info_impl(env);
}

}  // extern "C"
