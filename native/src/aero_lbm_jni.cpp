#include <jni.h>

#include "aero_lbm_capi.h"
#include "aero_lbm_hydro_core.h"
#include "aero_lbm_mesoscale.h"
#include "aero_lbm_thermal_core.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(AERO_LBM_OPENCL) && !defined(CL_TARGET_OPENCL_VERSION)
#define CL_TARGET_OPENCL_VERSION 120
#endif

#if defined(AERO_LBM_OPENCL)
#if defined(__APPLE__)
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif
#endif

namespace {

constexpr int kQ = aero_lbm::hydro_core::kQ;
constexpr int kThermalQ = aero_lbm::thermal_core::kQ;
constexpr int lattice_q(int ix, int iy, int iz) {
    return aero_lbm::hydro_core::lattice_q(ix, iy, iz);
}
constexpr std::array<std::array<float, 3>, 3> kMomentInv1D = {{
    {{0.0f, -0.5f, 0.5f}},  // c = -1
    {{1.0f, 0.0f, -1.0f}},  // c =  0
    {{0.0f, 0.5f, 0.5f}}    // c = +1
}};
constexpr auto kCx = aero_lbm::hydro_core::kCx;
constexpr auto kCy = aero_lbm::hydro_core::kCy;
constexpr auto kCz = aero_lbm::hydro_core::kCz;
constexpr auto kOpp = aero_lbm::hydro_core::kOpp;
constexpr auto kW = aero_lbm::hydro_core::kW;

constexpr auto kThermalCx = aero_lbm::thermal_core::kCx;
constexpr auto kThermalCy = aero_lbm::thermal_core::kCy;
constexpr auto kThermalCz = aero_lbm::thermal_core::kCz;
constexpr auto kThermalOpp = aero_lbm::thermal_core::kOpp;
constexpr auto kThermalW = aero_lbm::thermal_core::kW;
constexpr float kThermalCs2 = aero_lbm::thermal_core::kCs2;

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
constexpr int kChannelStateTemp = 10;
constexpr int kSparseOverlayChannels = 6;
constexpr int kSparseOverlayStateVx = 0;
constexpr int kSparseOverlayStateVy = 1;
constexpr int kSparseOverlayStateVz = 2;
constexpr int kSparseOverlayStateP = 3;
constexpr int kSparseOverlayStateTemp = 4;
constexpr int kSparseOverlayThermalSource = 5;

constexpr float kLatticeSoundSpeed = 0.57735026919f;
constexpr float kMaxMach = 0.60f;
constexpr float kHardMaxLatticeSpeed = kLatticeSoundSpeed * kMaxMach;
constexpr float kCs2 = 1.0f / 3.0f;
constexpr float kRuntimeSecondsPerStep = 1.0f / 20.0f;
constexpr float kRuntimeMetersPerCell = 1.0f;
constexpr float kRuntimeVelocityScale = kRuntimeMetersPerCell / kRuntimeSecondsPerStep;
constexpr float kRuntimeAirKinematicViscosityMetersSqPerSecond = 1.50e-5f;
constexpr float kRuntimeAirPrandtl = 0.71f;
constexpr float kRuntimeTurbulentPrandtl = 0.85f;
constexpr float kRuntimeAirThermalDiffusivityMetersSqPerSecond =
    aero_lbm::thermal_core::thermal_diffusivity_from_nu_pr(
        kRuntimeAirKinematicViscosityMetersSqPerSecond,
        kRuntimeAirPrandtl
    );
constexpr float kRuntimeTemperatureScaleKelvin = 20.0f;
constexpr float kRuntimeAirThermalExpansionPerKelvin = 1.0f / 300.0f;
constexpr float kRuntimeGravityMetersPerSecondSq = 9.81f;
constexpr float kRuntimeMolecularNu0 =
    aero_lbm::hydro_core::physical_diffusivity_to_lattice(
        kRuntimeAirKinematicViscosityMetersSqPerSecond,
        kRuntimeMetersPerCell,
        kRuntimeSecondsPerStep
    );
constexpr float kRuntimeMolecularAlpha0 =
    aero_lbm::hydro_core::physical_diffusivity_to_lattice(
        kRuntimeAirThermalDiffusivityMetersSqPerSecond,
        kRuntimeMetersPerCell,
        kRuntimeSecondsPerStep
    );

constexpr float kRhoMin = 0.97f;
constexpr float kRhoMax = 1.03f;
constexpr float kPressureMin = -0.03f;
constexpr float kPressureMax = 0.03f;

// D3Q27 cumulant closure with an explicit molecular viscosity baseline.
constexpr float kNuShearMin = 1.0e-8f;
constexpr float kNuShearMax = (0.95f - 0.5f) / 3.0f;
constexpr float kNuNormalMin = 1.0e-8f;
constexpr float kNuNormalMax = (0.95f - 0.5f) / 3.0f;
constexpr float kTauShear = aero_lbm::hydro_core::tau_from_lattice_diffusivity(kRuntimeMolecularNu0);
constexpr float kTauShearMin = 0.5f + 3.0f * kNuShearMin;
constexpr float kTauShearMax = 0.95f;
constexpr float kTauNormal = aero_lbm::hydro_core::tau_from_lattice_diffusivity(kRuntimeMolecularNu0);
constexpr float kTauNormalMin = 0.5f + 3.0f * kNuNormalMin;
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
constexpr float kFanTargetScale = 1.0f / kRuntimeVelocityScale;
constexpr float kFanTargetMax = 0.28f;
constexpr float kFanNoiseAmp = 0.02f;
constexpr float kFanSpeedSoftCap = 0.24f;
constexpr float kFanSpeedDampWidth = 0.04f;
constexpr float kFanPerpDamp = 1.0f;
constexpr float kRuntimeStateNudge = 0.08f;
constexpr int kRuntimeStateNudgeLayers = 8;
constexpr float kMaxSpeed = kHardMaxLatticeSpeed;

// Boussinesq approximation with an internal thermal scalar field.
constexpr bool kEnableBoussinesq = true;
constexpr float kThermalDiffusivity = 0.035f;
constexpr float kThermalCooling = 0.0f;
constexpr float kThermalSourceScale = 0.0012f;
constexpr float kThermalSourceMax = 0.006f;
constexpr float kThermalMin = -1.00f;
constexpr float kThermalMax = 1.00f;
constexpr int kThermalUpdateStride = 2;
constexpr float kBoussinesqBeta = aero_lbm::thermal_core::runtime_boussinesq_beta(
    kRuntimeGravityMetersPerSecondSq,
    kRuntimeAirThermalExpansionPerKelvin,
    kRuntimeTemperatureScaleKelvin,
    kRuntimeSecondsPerStep,
    kRuntimeMetersPerCell
);
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
            cfg.mach_number = 0.15f;
            cfg.flags |= AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE;
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

struct SpinMutex {
    std::atomic_flag flag = ATOMIC_FLAG_INIT;

    void lock() noexcept {
        while (flag.test_and_set(std::memory_order_acquire)) {
        }
    }

    void unlock() noexcept {
        flag.clear(std::memory_order_release);
    }
};

struct ContextState {
    std::unique_ptr<SpinMutex> mutex = std::make_unique<SpinMutex>();
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
    std::vector<float> ref_temperature;

    std::vector<float> packet; // reused host-side payload buffer (floats)
    std::vector<float> payload_cache; // last payload uploaded to the GPU runtime
    std::vector<float> fan_mask;
    std::vector<float> fan_ux;
    std::vector<float> fan_uy;
    std::vector<float> fan_uz;
    std::vector<float> thermal_source;
    std::vector<uint8_t> obstacle;
    std::vector<float> temperature;
    std::vector<float> temperature_next;
    std::vector<float> temperature_scratch;
    std::vector<float> thermal_f;
    std::vector<float> thermal_f_post;
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
    cl_mem d_temp_scratch = nullptr;
    cl_mem d_thermal_f = nullptr;
    cl_mem d_thermal_f_post = nullptr;
#endif
};

Config g_cfg;
std::unordered_map<jlong, ContextState> g_contexts;
SpinMutex g_contexts_mutex;
std::string g_last_native_error;

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

inline void clear_last_native_error() {
    g_last_native_error.clear();
}

inline void set_last_native_error(std::string error) {
    g_last_native_error = std::move(error);
}

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

inline float effective_state_nudge() {
    return benchmark_mode_active() ? 0.0f : kRuntimeStateNudge;
}

inline float effective_runtime_state_nudge(int nx, int ny, int nz, int x, int y, int z) {
    const float base = effective_state_nudge();
    if (base <= 0.0f || kRuntimeStateNudgeLayers <= 0) {
        return 0.0f;
    }
    const int d = std::min({x, y, z, nx - 1 - x, ny - 1 - y, nz - 1 - z});
    if (d >= kRuntimeStateNudgeLayers) {
        return 0.0f;
    }
    const float eta = static_cast<float>(kRuntimeStateNudgeLayers - d) / static_cast<float>(kRuntimeStateNudgeLayers);
    return base * eta * eta;
}

bool halo_exchange_slab_bounds(
    int offset_x,
    int offset_y,
    int offset_z,
    int nx,
    int ny,
    int nz,
    int halo,
    int core,
    int* neg_src_x0,
    int* neg_src_y0,
    int* neg_src_z0,
    int* pos_dst_x0,
    int* pos_dst_y0,
    int* pos_dst_z0,
    int* pos_src_x0,
    int* pos_src_y0,
    int* pos_src_z0,
    int* neg_dst_x0,
    int* neg_dst_y0,
    int* neg_dst_z0,
    int* size_x,
    int* size_y,
    int* size_z
);

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
    if (g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D
        || g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D) {
        return clampf(finite_or(g_benchmark_cfg.mach_number, 0.05f), 1.0e-4f, 0.30f) * std::sqrt(kCs2);
    }
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

inline bool heated_cavity_benchmark_active() {
    return benchmark_mode_active()
        && (g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D
            || g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D);
}

inline bool thermal_bfecc_active() {
    return !benchmark_mode_active() || heated_cavity_benchmark_active();
}

inline bool thermal_ddf_benchmark_active() {
    // Benchmark-mode thermal transport stays scalar to avoid the extra bandwidth
    // cost of a second distribution set; heated cavities use BFECC instead.
    return false;
}

inline float benchmark_temperature_span() {
    if (!benchmark_mode_active()) return 1.0f;
    const AeroLbmBoundaryFaceConfig faces[6] = {
        g_benchmark_cfg.x_min, g_benchmark_cfg.x_max,
        g_benchmark_cfg.y_min, g_benchmark_cfg.y_max,
        g_benchmark_cfg.z_min, g_benchmark_cfg.z_max,
    };
    float t_min = 1.0e9f;
    float t_max = -1.0e9f;
    bool any_dirichlet = false;
    for (const AeroLbmBoundaryFaceConfig& face : faces) {
        if (face.thermal_kind != AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET) continue;
        any_dirichlet = true;
        t_min = std::min(t_min, face.temperature);
        t_max = std::max(t_max, face.temperature);
    }
    if (!any_dirichlet) return 1.0f;
    return std::max(1.0e-6f, t_max - t_min);
}

inline int effective_thermal_update_stride() {
    if (!kEnableBoussinesq) return 1;
    if (benchmark_mode_active()) return 1;
    return std::max(1, kThermalUpdateStride);
}

inline float effective_thermal_dt() {
    return static_cast<float>(effective_thermal_update_stride());
}

inline float clamp_shear_nu(float nu) {
    return clampf(nu, kNuShearMin, kNuShearMax);
}

inline float clamp_normal_nu(float nu) {
    return clampf(nu, kNuNormalMin, kNuNormalMax);
}

inline float tau_from_shear_nu(float nu) {
    return 0.5f + 3.0f * clamp_shear_nu(nu);
}

inline float tau_from_normal_nu(float nu) {
    return 0.5f + 3.0f * clamp_normal_nu(nu);
}

inline float runtime_molecular_nu0() {
    return clamp_shear_nu(kRuntimeMolecularNu0);
}

inline float runtime_molecular_alpha0() {
    return std::max(1.0e-8f, kRuntimeMolecularAlpha0);
}

inline float effective_base_nu_shear() {
    if (!benchmark_mode_active()) return runtime_molecular_nu0();
    if (heated_cavity_benchmark_active()) {
        const float ra = std::max(1.0e-6f, finite_or(g_benchmark_cfg.rayleigh_number, 1.0e5f));
        const float pr = std::max(1.0e-6f, finite_or(g_benchmark_cfg.prandtl_number, 0.71f));
        const float ref_speed = benchmark_reference_speed();
        const float ref_length = std::max(1.0e-6f, finite_or(g_benchmark_cfg.reference_length, 1.0f));
        return clamp_shear_nu(ref_speed * ref_length * std::sqrt(pr / ra));
    }
    const float re = finite_or(g_benchmark_cfg.reynolds_number, 0.0f);
    const float ref_speed = benchmark_reference_speed();
    const float ref_length = std::max(1.0e-6f, finite_or(g_benchmark_cfg.reference_length, 1.0f));
    if (re <= 1.0e-6f || ref_speed <= 1.0e-8f) return runtime_molecular_nu0();
    return clamp_shear_nu(ref_speed * ref_length / re);
}

inline float effective_base_nu_normal(float nu_shear_eff) {
    if (!benchmark_mode_active()) return clamp_normal_nu(runtime_molecular_nu0());
    return clamp_normal_nu(nu_shear_eff);
}

inline float effective_benchmark_tau_shear() {
    return clampf(tau_from_shear_nu(effective_base_nu_shear()), kTauShearMin, kTauShearMax);
}

inline float effective_benchmark_tau_normal(float tau_shear_eff) {
    if (!benchmark_mode_active()) return clampf(kTauNormal, kTauNormalMin, kTauNormalMax);
    (void)tau_shear_eff;
    return clampf(tau_from_normal_nu(effective_base_nu_normal(effective_base_nu_shear())), kTauNormalMin, kTauNormalMax);
}

inline float effective_thermal_diffusivity() {
    if (!benchmark_mode_active()) return runtime_molecular_alpha0();
    if (!heated_cavity_benchmark_active()) return kThermalDiffusivity;
    const float pr = std::max(1.0e-6f, finite_or(g_benchmark_cfg.prandtl_number, 0.71f));
    return effective_base_nu_shear() / pr;
}

inline float effective_thermal_tau() {
    const float alpha = std::max(1.0e-6f, effective_thermal_diffusivity());
    return clampf(aero_lbm::thermal_core::tau_from_alpha(alpha), 0.5005f, 3.0f);
}

inline float thermal_feq(int q, float temperature, float ux, float uy, float uz) {
    return aero_lbm::thermal_core::equilibrium_d3q7(q, temperature, ux, uy, uz);
}

inline float effective_thermal_cooling() {
    return 0.0f;
}

inline float effective_boussinesq_beta() {
    if (!heated_cavity_benchmark_active()) return kBoussinesqBeta;
    const float ref_speed = benchmark_reference_speed();
    const float ref_length = std::max(1.0e-6f, finite_or(g_benchmark_cfg.reference_length, 1.0f));
    const float delta_t = benchmark_temperature_span();
    return (ref_speed * ref_speed) / std::max(1.0e-6f, ref_length * delta_t);
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

inline float opencl_effective_base_nu_shear() {
    return effective_base_nu_shear();
}

inline std::array<float, 3> opencl_effective_thermal_transport() {
    return {
        effective_thermal_diffusivity(),
        effective_thermal_cooling(),
        effective_boussinesq_beta(),
    };
}

inline int opencl_effective_thermal_update_stride() {
    return effective_thermal_update_stride();
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

inline std::size_t thermal_dist_index(std::size_t cell, int q, std::size_t cells) {
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
    return aero_lbm::hydro_core::equilibrium_d3q27(q, rho, ux, uy, uz);
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
    ctx.ref_temperature.assign(ctx.cells, 0.0f);
    ctx.packet.assign(ctx.cells * g_cfg.input_channels, 0.0f);
    ctx.payload_cache.clear();

    ctx.fan_mask.assign(ctx.cells, 0.0f);
    ctx.fan_ux.assign(ctx.cells, 0.0f);
    ctx.fan_uy.assign(ctx.cells, 0.0f);
    ctx.fan_uz.assign(ctx.cells, 0.0f);
    ctx.thermal_source.assign(ctx.cells, 0.0f);
    ctx.obstacle.assign(ctx.cells, 0);
    ctx.temperature.assign(ctx.cells, 0.0f);
    ctx.temperature_next.assign(ctx.cells, 0.0f);
    ctx.temperature_scratch.assign(ctx.cells, 0.0f);
    ctx.thermal_f.clear();
    ctx.thermal_f_post.clear();
    ctx.last_force[0] = 0.0f;
    ctx.last_force[1] = 0.0f;
    ctx.last_force[2] = 0.0f;
}

void ensure_context_thermal_ddf_storage(ContextState& ctx) {
    if (!thermal_ddf_benchmark_active()) return;
    const std::size_t thermal_cells = ctx.cells * kThermalQ;
    if (ctx.thermal_f.size() != thermal_cells) {
        ctx.thermal_f.assign(thermal_cells, 0.0f);
    }
    if (ctx.thermal_f_post.size() != thermal_cells) {
        ctx.thermal_f_post.assign(thermal_cells, 0.0f);
    }
}

void ensure_context_temperature_storage(ContextState& ctx);

void rebuild_thermal_distributions_from_temperature(ContextState& ctx) {
    if (!thermal_ddf_benchmark_active()) return;
    ensure_context_thermal_ddf_storage(ctx);
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        const float temperature = ctx.obstacle[cell] ? 0.0f : clampf(ctx.temperature[cell], kThermalMin, kThermalMax);
        const float ux = ctx.obstacle[cell] ? 0.0f : ctx.ux[cell];
        const float uy = ctx.obstacle[cell] ? 0.0f : ctx.uy[cell];
        const float uz = ctx.obstacle[cell] ? 0.0f : ctx.uz[cell];
        for (int q = 0; q < kThermalQ; ++q) {
            const float geq = thermal_feq(q, temperature, ux, uy, uz);
            ctx.thermal_f[thermal_dist_index(cell, q, ctx.cells)] = geq;
            ctx.thermal_f_post[thermal_dist_index(cell, q, ctx.cells)] = geq;
        }
    }
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
        float ref_temperature = 0.0f;
        if (in_channels > kChannelStateTemp) {
            ref_temperature = payload[base + kChannelStateTemp];
        }
        ctx.ref_temperature[cell] = clampf(ref_temperature, kThermalMin, kThermalMax);
        float thermal_src = 0.0f;
        if (in_channels > kChannelThermalSource) {
            thermal_src = payload[base + kChannelThermalSource];
        }
        ctx.thermal_source[cell] = clampf(thermal_src, -kThermalSourceMax, kThermalSourceMax);
    }
}

void apply_sparse_payload_overlays(
    ContextState& ctx,
    const int* overlay_cells,
    const float* overlay_values,
    int overlay_count
) {
    if (overlay_count <= 0 || !overlay_cells || !overlay_values) {
        return;
    }
    for (int overlay_index = 0; overlay_index < overlay_count; ++overlay_index) {
        const int cell = overlay_cells[overlay_index];
        if (cell < 0 || static_cast<std::size_t>(cell) >= ctx.cells) {
            continue;
        }
        const float* values = overlay_values + static_cast<std::size_t>(overlay_index) * kSparseOverlayChannels;
        ctx.ref_ux[static_cast<std::size_t>(cell)] = finite_or(values[kSparseOverlayStateVx], 0.0f);
        ctx.ref_uy[static_cast<std::size_t>(cell)] = finite_or(values[kSparseOverlayStateVy], 0.0f);
        ctx.ref_uz[static_cast<std::size_t>(cell)] = finite_or(values[kSparseOverlayStateVz], 0.0f);
        ctx.ref_pressure[static_cast<std::size_t>(cell)] = clampf(values[kSparseOverlayStateP], kPressureMin, kPressureMax);
        ctx.ref_temperature[static_cast<std::size_t>(cell)] = clampf(values[kSparseOverlayStateTemp], kThermalMin, kThermalMax);
        ctx.thermal_source[static_cast<std::size_t>(cell)] = clampf(
            values[kSparseOverlayThermalSource],
            -kThermalSourceMax,
            kThermalSourceMax
        );
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
        } else if (!ctx.obstacle[cell] && ctx.ref_temperature.size() == ctx.cells) {
            ctx.temperature[cell] = clampf(ctx.ref_temperature[cell], kThermalMin, kThermalMax);
        }
        ctx.temperature[cell] = clampf(ctx.temperature[cell], kThermalMin, kThermalMax);
        ctx.temperature_next[cell] = ctx.temperature[cell];
    }
    rebuild_thermal_distributions_from_temperature(ctx);
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

inline float thermal_field_neighbor_or_self(
    const ContextState& ctx,
    const std::vector<float>& field,
    int x,
    int y,
    int z,
    float self_value
) {
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
    return field[cell];
}

inline float thermal_neighbor_or_self(const ContextState& ctx, int x, int y, int z, float self_value) {
    return thermal_field_neighbor_or_self(ctx, ctx.temperature, x, y, z, self_value);
}

inline float thermal_boundary_distribution(
    const ContextState& ctx,
    const std::vector<float>& thermal_read,
    std::size_t cell,
    int q,
    float self_temperature
) {
    const float outgoing = thermal_read[thermal_dist_index(cell, kThermalOpp[q], ctx.cells)];
    return outgoing;
}

inline float thermal_streamed_value(
    const ContextState& ctx,
    const std::vector<float>& thermal_read,
    std::size_t cell,
    int x,
    int y,
    int z,
    int q,
    float self_temperature
) {
    int sx = x - kThermalCx[q];
    int sy = y - kThermalCy[q];
    int sz = z - kThermalCz[q];
    remap_thermal_coords(sx, sy, sz, ctx.nx, ctx.ny, ctx.nz);
    if (sx < 0 || sy < 0 || sz < 0 || sx >= ctx.nx || sy >= ctx.ny || sz >= ctx.nz) {
        const AeroLbmBoundaryFaceConfig* face = thermal_face_for_oob(sx, sy, sz, ctx.nx, ctx.ny, ctx.nz);
        const int kind = face ? face->thermal_kind : AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME;
        switch (kind) {
            case AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET: {
                const float wall_t = clampf(face->temperature, kThermalMin, kThermalMax);
                return -thermal_read[thermal_dist_index(cell, kThermalOpp[q], ctx.cells)] + 2.0f * kThermalW[q] * wall_t;
            }
            case AERO_LBM_THERMAL_BOUNDARY_HEAT_FLUX_NEUMANN: {
                const float wall_t = clampf(self_temperature + face->heat_flux, kThermalMin, kThermalMax);
                return -thermal_read[thermal_dist_index(cell, kThermalOpp[q], ctx.cells)] + 2.0f * kThermalW[q] * wall_t;
            }
            case AERO_LBM_THERMAL_BOUNDARY_DISABLED:
            case AERO_LBM_THERMAL_BOUNDARY_ADIABATIC:
            case AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME:
            default:
                return thermal_boundary_distribution(ctx, thermal_read, cell, q, self_temperature);
        }
    }
    const std::size_t src = cell_index(sx, sy, sz, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.obstacle[src]) {
        return thermal_boundary_distribution(ctx, thermal_read, cell, q, self_temperature);
    }
    return thermal_read[thermal_dist_index(src, q, ctx.cells)];
}

inline void local_temperature_bounds(
    const ContextState& ctx,
    const std::vector<float>& field,
    int x,
    int y,
    int z,
    float& out_min,
    float& out_max
) {
    const std::size_t cell = cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz);
    const float center = field[cell];
    out_min = center;
    out_max = center;
    const float neighbors[6] = {
        thermal_field_neighbor_or_self(ctx, field, x + 1, y, z, center),
        thermal_field_neighbor_or_self(ctx, field, x - 1, y, z, center),
        thermal_field_neighbor_or_self(ctx, field, x, y + 1, z, center),
        thermal_field_neighbor_or_self(ctx, field, x, y - 1, z, center),
        thermal_field_neighbor_or_self(ctx, field, x, y, z + 1, center),
        thermal_field_neighbor_or_self(ctx, field, x, y, z - 1, center),
    };
    for (float value : neighbors) {
        out_min = std::min(out_min, value);
        out_max = std::max(out_max, value);
    }
}

inline float sample_temperature_trilinear_field(
    const ContextState& ctx,
    const std::vector<float>& field,
    float x,
    float y,
    float z,
    float fallback
) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int z0 = static_cast<int>(std::floor(z));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const int z1 = z0 + 1;

    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);
    const float fz = z - static_cast<float>(z0);

    const float c000 = thermal_field_neighbor_or_self(ctx, field, x0, y0, z0, fallback);
    const float c100 = thermal_field_neighbor_or_self(ctx, field, x1, y0, z0, fallback);
    const float c010 = thermal_field_neighbor_or_self(ctx, field, x0, y1, z0, fallback);
    const float c110 = thermal_field_neighbor_or_self(ctx, field, x1, y1, z0, fallback);
    const float c001 = thermal_field_neighbor_or_self(ctx, field, x0, y0, z1, fallback);
    const float c101 = thermal_field_neighbor_or_self(ctx, field, x1, y0, z1, fallback);
    const float c011 = thermal_field_neighbor_or_self(ctx, field, x0, y1, z1, fallback);
    const float c111 = thermal_field_neighbor_or_self(ctx, field, x1, y1, z1, fallback);

    const float c00 = c000 + (c100 - c000) * fx;
    const float c10 = c010 + (c110 - c010) * fx;
    const float c01 = c001 + (c101 - c001) * fx;
    const float c11 = c011 + (c111 - c011) * fx;
    const float c0 = c00 + (c10 - c00) * fy;
    const float c1 = c01 + (c11 - c01) * fy;
    return c0 + (c1 - c0) * fz;
}

inline float sample_temperature_trilinear(
    const ContextState& ctx, float x, float y, float z, float fallback
) {
    return sample_temperature_trilinear_field(ctx, ctx.temperature, x, y, z, fallback);
}

inline float velocity_component_or_self(
    const ContextState& ctx,
    int x,
    int y,
    int z,
    int component,
    float fallback
) {
    if (x < 0 || y < 0 || z < 0 || x >= ctx.nx || y >= ctx.ny || z >= ctx.nz) return fallback;
    const std::size_t neighbor = cell_index(x, y, z, ctx.nx, ctx.ny, ctx.nz);
    if (ctx.obstacle[neighbor]) return fallback;
    switch (component) {
        case 0: return ctx.ux[neighbor];
        case 1: return ctx.uy[neighbor];
        case 2: return ctx.uz[neighbor];
        default: return fallback;
    }
}

inline float runtime_local_turbulent_nu(
    const ContextState& ctx,
    int x,
    int y,
    int z,
    float ux_center,
    float uy_center,
    float uz_center
) {
    if (benchmark_mode_active() || !effective_enable_sgs()) return 0.0f;

    const float ux_px = velocity_component_or_self(ctx, x + 1, y, z, 0, ux_center);
    const float ux_mx = velocity_component_or_self(ctx, x - 1, y, z, 0, ux_center);
    const float ux_py = velocity_component_or_self(ctx, x, y + 1, z, 0, ux_center);
    const float ux_my = velocity_component_or_self(ctx, x, y - 1, z, 0, ux_center);
    const float ux_pz = velocity_component_or_self(ctx, x, y, z + 1, 0, ux_center);
    const float ux_mz = velocity_component_or_self(ctx, x, y, z - 1, 0, ux_center);

    const float uy_px = velocity_component_or_self(ctx, x + 1, y, z, 1, uy_center);
    const float uy_mx = velocity_component_or_self(ctx, x - 1, y, z, 1, uy_center);
    const float uy_py = velocity_component_or_self(ctx, x, y + 1, z, 1, uy_center);
    const float uy_my = velocity_component_or_self(ctx, x, y - 1, z, 1, uy_center);
    const float uy_pz = velocity_component_or_self(ctx, x, y, z + 1, 1, uy_center);
    const float uy_mz = velocity_component_or_self(ctx, x, y, z - 1, 1, uy_center);

    const float uz_px = velocity_component_or_self(ctx, x + 1, y, z, 2, uz_center);
    const float uz_mx = velocity_component_or_self(ctx, x - 1, y, z, 2, uz_center);
    const float uz_py = velocity_component_or_self(ctx, x, y + 1, z, 2, uz_center);
    const float uz_my = velocity_component_or_self(ctx, x, y - 1, z, 2, uz_center);
    const float uz_pz = velocity_component_or_self(ctx, x, y, z + 1, 2, uz_center);
    const float uz_mz = velocity_component_or_self(ctx, x, y, z - 1, 2, uz_center);

    const float dux_dx = 0.5f * (ux_px - ux_mx);
    const float dux_dy = 0.5f * (ux_py - ux_my);
    const float dux_dz = 0.5f * (ux_pz - ux_mz);
    const float duy_dx = 0.5f * (uy_px - uy_mx);
    const float duy_dy = 0.5f * (uy_py - uy_my);
    const float duy_dz = 0.5f * (uy_pz - uy_mz);
    const float duz_dx = 0.5f * (uz_px - uz_mx);
    const float duz_dy = 0.5f * (uz_py - uz_my);
    const float duz_dz = 0.5f * (uz_pz - uz_mz);

    const float s_xx = dux_dx;
    const float s_yy = duy_dy;
    const float s_zz = duz_dz;
    const float s_xy = 0.5f * (dux_dy + duy_dx);
    const float s_xz = 0.5f * (dux_dz + duz_dx);
    const float s_yz = 0.5f * (duy_dz + duz_dy);
    const float s_mag = std::sqrt(std::max(
        0.0f,
        2.0f * (s_xx * s_xx + s_yy * s_yy + s_zz * s_zz)
        + 4.0f * (s_xy * s_xy + s_xz * s_xz + s_yz * s_yz)
    ));

    const float nu0 = runtime_molecular_nu0();
    float nu_t = kSgsC2 * s_mag;
    nu_t = std::min(nu_t, kSgsNutToNu0Max * nu0);
    return std::max(0.0f, nu_t);
}

inline float effective_local_thermal_diffusivity(
    const ContextState& ctx,
    int x,
    int y,
    int z,
    float ux_center,
    float uy_center,
    float uz_center
) {
    if (benchmark_mode_active()) return effective_thermal_diffusivity();
    const float alpha_mol = runtime_molecular_alpha0();
    const float nu_t = runtime_local_turbulent_nu(ctx, x, y, z, ux_center, uy_center, uz_center);
    const float alpha_t = nu_t / std::max(1.0e-6f, kRuntimeTurbulentPrandtl);
    return alpha_mol + alpha_t;
}

void update_temperature_field(ContextState& ctx) {
    if (thermal_bfecc_active()) {
        // BFECC reduces the excessive dissipation of first-order semi-Lagrangian
        // advection without introducing a full thermal DDF in benchmark mode.
        ensure_context_temperature_storage(ctx);
        const float thermal_cooling = effective_thermal_cooling();
        const float thermal_dt = effective_thermal_dt();

        for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
            if (ctx.obstacle[cell]) {
                ctx.temperature_scratch[cell] = 0.0f;
                continue;
            }
            int x = 0, y = 0, z = 0;
            decode_cell(cell, ctx.nx, ctx.ny, ctx.nz, x, y, z);
            const float t_center = ctx.temperature[cell];
            ctx.temperature_scratch[cell] = sample_temperature_trilinear_field(
                ctx,
                ctx.temperature,
                static_cast<float>(x) - thermal_dt * ctx.ux[cell],
                static_cast<float>(y) - thermal_dt * ctx.uy[cell],
                static_cast<float>(z) - thermal_dt * ctx.uz[cell],
                t_center
            );
        }

        for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
            if (ctx.obstacle[cell]) {
                ctx.temperature_next[cell] = 0.0f;
                continue;
            }
            int x = 0, y = 0, z = 0;
            decode_cell(cell, ctx.nx, ctx.ny, ctx.nz, x, y, z);
            const float t_center = ctx.temperature[cell];
            const float backward = sample_temperature_trilinear_field(
                ctx,
                ctx.temperature_scratch,
                static_cast<float>(x) + thermal_dt * ctx.ux[cell],
                static_cast<float>(y) + thermal_dt * ctx.uy[cell],
                static_cast<float>(z) + thermal_dt * ctx.uz[cell],
                ctx.temperature_scratch[cell]
            );
            ctx.temperature_next[cell] = clampf(t_center + 0.5f * (t_center - backward), kThermalMin, kThermalMax);
        }

        for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
            if (ctx.obstacle[cell]) {
                ctx.temperature_scratch[cell] = 0.0f;
                continue;
            }
            int x = 0, y = 0, z = 0;
            decode_cell(cell, ctx.nx, ctx.ny, ctx.nz, x, y, z);
            const float t_center = ctx.temperature[cell];
            float advected = sample_temperature_trilinear_field(
                ctx,
                ctx.temperature_next,
                static_cast<float>(x) - thermal_dt * ctx.ux[cell],
                static_cast<float>(y) - thermal_dt * ctx.uy[cell],
                static_cast<float>(z) - thermal_dt * ctx.uz[cell],
                ctx.temperature_next[cell]
            );
            float local_min = 0.0f;
            float local_max = 0.0f;
            local_temperature_bounds(ctx, ctx.temperature, x, y, z, local_min, local_max);
            advected = clampf(advected, local_min, local_max);

            const float thermal_diffusivity = effective_local_thermal_diffusivity(
                ctx,
                x,
                y,
                z,
                ctx.ux[cell],
                ctx.uy[cell],
                ctx.uz[cell]
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
            ctx.temperature_scratch[cell] = clampf(
                advected + thermal_dt * (thermal_diffusivity * laplacian + source - thermal_cooling * advected),
                kThermalMin,
                kThermalMax
            );
        }

        ctx.temperature.swap(ctx.temperature_scratch);
        std::copy(ctx.temperature.begin(), ctx.temperature.end(), ctx.temperature_next.begin());
        return;
    }

    if (!kEnableBoussinesq) return;
    const float thermal_cooling = effective_thermal_cooling();
    const float thermal_dt = effective_thermal_dt();
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
            static_cast<float>(x) - thermal_dt * ctx.ux[cell],
            static_cast<float>(y) - thermal_dt * ctx.uy[cell],
            static_cast<float>(z) - thermal_dt * ctx.uz[cell],
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
        const float thermal_diffusivity = effective_local_thermal_diffusivity(
            ctx,
            x,
            y,
            z,
            ctx.ux[cell],
            ctx.uy[cell],
            ctx.uz[cell]
        );
        const float t_next = advected
                             + thermal_dt * (thermal_diffusivity * laplacian + source - thermal_cooling * advected);
        ctx.temperature_next[cell] = clampf(t_next, kThermalMin, kThermalMax);
    }
    ctx.temperature.swap(ctx.temperature_next);
}

inline bool should_update_temperature(std::uint64_t step_counter) {
    if (!kEnableBoussinesq) return false;
    const int stride = effective_thermal_update_stride();
    if (stride <= 1) return true;
    return (step_counter % static_cast<std::uint64_t>(stride)) == 0;
}

inline bool temperature_was_updated_last_step(const ContextState& ctx) {
    const int stride = effective_thermal_update_stride();
    if (stride <= 1) return true;
    if (ctx.step_counter == 0) return true;
    return ((ctx.step_counter - 1) % static_cast<std::uint64_t>(stride)) == 0;
}

inline bool should_exchange_temperature_halo(const ContextState& first, const ContextState& second) {
    return temperature_was_updated_last_step(first) || temperature_was_updated_last_step(second);
}

void collide(ContextState& ctx) {
    const float boussinesq_beta = effective_boussinesq_beta();
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
            if (!std::isfinite(fq)) {
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
                boussinesq_beta * ctx.temperature[cell],
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

        const int yz = ctx.ny * ctx.nz;
        const int x = static_cast<int>(cell / yz);
        const int rem = static_cast<int>(cell - static_cast<std::size_t>(x) * yz);
        const int y = rem / ctx.nz;
        const int z = rem - y * ctx.nz;
        const float state_nudge = effective_runtime_state_nudge(ctx.nx, ctx.ny, ctx.nz, x, y, z);
        ux = (1.0f - state_nudge) * ux + state_nudge * ctx.ref_ux[cell];
        uy = (1.0f - state_nudge) * uy + state_nudge * ctx.ref_uy[cell];
        uz = (1.0f - state_nudge) * uz + state_nudge * ctx.ref_uz[cell];
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

        const float tau_shear_base = effective_benchmark_tau_shear();
        const float tau_normal_base = effective_benchmark_tau_normal(tau_shear_base);
        float tau_shear_local = tau_shear_base;
        float tau_normal_local = tau_normal_base;
        if (effective_enable_sgs()) {
            const float nu0 = std::max(1e-6f, effective_base_nu_shear());
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
            tau_shear_local = clampf(tau_from_shear_nu(nu0 + nu_t), kTauShearMin, kTauShearMax);
            tau_normal_local = clampf(
                tau_normal_base + (tau_shear_local - tau_shear_base) * kSgsBulkCoupling,
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
            if (!std::isfinite(fq)) {
                rho = 1.0f;
                ux = uy = uz = 0.0f;
                for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
                goto next_cell;
            }
            rho += fq;
            ux += fq * kCx[q]; uy += fq * kCy[q]; uz += fq * kCz[q];
        }
        if (!std::isfinite(rho) || !std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
            for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
            goto next_cell;
        }
        rho = clampf(rho, kRhoMin, kRhoMax);
        if (!std::isfinite(rho) || rho <= 1e-6f) {
            for (int c = 0; c < out_channels; ++c) out[out_base + c] = 0.0f;
            goto next_cell;
        }
        ux /= rho; uy /= rho; uz /= rho;
        if (!std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
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
    cl_kernel k_apply_temperature_reference = nullptr;
    cl_kernel k_thermal_bfecc_forward = nullptr;
    cl_kernel k_thermal_bfecc_correct = nullptr;
    cl_kernel k_thermal_bfecc_finalize = nullptr;
    cl_kernel k_stream_collide_tgv = nullptr;
    cl_kernel k_stream_collide_hydro_bench = nullptr;
    cl_kernel k_stream_collide_hydro_forced = nullptr;
    cl_kernel k_output = nullptr;
    cl_kernel k_output_strided = nullptr;
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
__constant int TQ = 7;
__constant int TCX[7] = {0, 1, -1, 0, 0, 0, 0};
__constant int TCY[7] = {0, 0, 0, 1, -1, 0, 0};
__constant int TCZ[7] = {0, 0, 0, 0, 0, 1, -1};
__constant int TOPP[7] = {0, 2, 1, 4, 3, 6, 5};
__constant float TW[7] = {0.25f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f};
__constant float TINV[3][3] = {
    {0.0f, -0.5f, 0.5f},
    {1.0f, 0.0f, -1.0f},
    {0.0f, 0.5f, 0.5f}
};

__constant float RUNTIME_MOLECULAR_NU0 = 7.500000e-7f;
__constant float RUNTIME_MOLECULAR_ALPHA0 = 1.056338e-6f;
__constant float RUNTIME_TURBULENT_PRANDTL = 8.500000e-1f;
__constant float NU_SHEAR_MIN = 1.0e-8f;
__constant float NU_SHEAR_MAX = 1.500000e-1f;
__constant float NU_NORMAL_MIN = 1.0e-8f;
__constant float NU_NORMAL_MAX = 1.500000e-1f;
__constant float TAU_SHEAR = 5.0000225e-1f;
__constant float TAU_SHEAR_MIN = 5.0000003e-1f;
__constant float TAU_SHEAR_MAX = 9.5e-1f;
__constant float TAU_NORMAL = 5.0000225e-1f;
__constant float TAU_NORMAL_MIN = 5.0000003e-1f;
__constant float TAU_NORMAL_MAX = 9.5e-1f;
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
__constant float THERMAL_MIN = -1.00f;
__constant float THERMAL_MAX = 1.00f;
__constant int THERMAL_UPDATE_STRIDE = 2;
__constant float THERMAL_CS2 = 0.25f;
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
#define BENCH_PRESET_HEATED_CAVITY_2D 5
#define BENCH_PRESET_HEATED_CAVITY_3D 6

#define PERIODIC_AXIS_X 1
#define PERIODIC_AXIS_Y 2
#define PERIODIC_AXIS_Z 4

#define CYLINDER_BENCHMARK_LENGTH 2.2f
#define CYLINDER_BENCHMARK_HEIGHT 0.41f

inline float clampf(float v, float lo, float hi) { return fmin(hi, fmax(lo, v)); }
inline int clampi(int v, int lo, int hi) { return min(hi, max(lo, v)); }
inline float clamp_shear_nu(float nu) { return clampf(nu, NU_SHEAR_MIN, NU_SHEAR_MAX); }
inline float clamp_normal_nu(float nu) { return clampf(nu, NU_NORMAL_MIN, NU_NORMAL_MAX); }
inline float tau_from_shear_nu(float nu) { return 0.5f + 3.0f * clamp_shear_nu(nu); }
inline float tau_from_normal_nu(float nu) { return 0.5f + 3.0f * clamp_normal_nu(nu); }
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

)CLC"
R"CLC(
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

inline float velocity_component_or_self(
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int x,
    int y,
    int z,
    int channel,
    float fallback
) {
    if (x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz) return fallback;
    int cell = (x * ny + y) * nz + z;
    int base = cell * in_ch;
    if (payload[base + 0] > 0.5f) return fallback;
    return payload[base + channel];
}

inline float runtime_local_turbulent_nu(
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int x,
    int y,
    int z,
    float ux_center,
    float uy_center,
    float uz_center
) {
    float ux_px = velocity_component_or_self(payload, in_ch, nx, ny, nz, x + 1, y, z, 5, ux_center);
    float ux_mx = velocity_component_or_self(payload, in_ch, nx, ny, nz, x - 1, y, z, 5, ux_center);
    float ux_py = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y + 1, z, 5, ux_center);
    float ux_my = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y - 1, z, 5, ux_center);
    float ux_pz = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y, z + 1, 5, ux_center);
    float ux_mz = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y, z - 1, 5, ux_center);

    float uy_px = velocity_component_or_self(payload, in_ch, nx, ny, nz, x + 1, y, z, 6, uy_center);
    float uy_mx = velocity_component_or_self(payload, in_ch, nx, ny, nz, x - 1, y, z, 6, uy_center);
    float uy_py = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y + 1, z, 6, uy_center);
    float uy_my = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y - 1, z, 6, uy_center);
    float uy_pz = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y, z + 1, 6, uy_center);
    float uy_mz = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y, z - 1, 6, uy_center);

    float uz_px = velocity_component_or_self(payload, in_ch, nx, ny, nz, x + 1, y, z, 7, uz_center);
    float uz_mx = velocity_component_or_self(payload, in_ch, nx, ny, nz, x - 1, y, z, 7, uz_center);
    float uz_py = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y + 1, z, 7, uz_center);
    float uz_my = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y - 1, z, 7, uz_center);
    float uz_pz = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y, z + 1, 7, uz_center);
    float uz_mz = velocity_component_or_self(payload, in_ch, nx, ny, nz, x, y, z - 1, 7, uz_center);

    float dux_dx = 0.5f * (ux_px - ux_mx);
    float dux_dy = 0.5f * (ux_py - ux_my);
    float dux_dz = 0.5f * (ux_pz - ux_mz);
    float duy_dx = 0.5f * (uy_px - uy_mx);
    float duy_dy = 0.5f * (uy_py - uy_my);
    float duy_dz = 0.5f * (uy_pz - uy_mz);
    float duz_dx = 0.5f * (uz_px - uz_mx);
    float duz_dy = 0.5f * (uz_py - uz_my);
    float duz_dz = 0.5f * (uz_pz - uz_mz);

    float s_xx = dux_dx;
    float s_yy = duy_dy;
    float s_zz = duz_dz;
    float s_xy = 0.5f * (dux_dy + duy_dx);
    float s_xz = 0.5f * (dux_dz + duz_dx);
    float s_yz = 0.5f * (duy_dz + duz_dy);
    float s_mag = sqrt(fmax(
        0.0f,
        2.0f * (s_xx * s_xx + s_yy * s_yy + s_zz * s_zz)
        + 4.0f * (s_xy * s_xy + s_xz * s_xz + s_yz * s_yz)
    ));

    float nu_t = SGS_C2 * s_mag;
    nu_t = fmin(nu_t, SGS_NUT_TO_NU0_MAX * RUNTIME_MOLECULAR_NU0);
    return fmax(0.0f, nu_t);
}

inline float runtime_local_thermal_diffusivity(
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int x,
    int y,
    int z,
    float ux_center,
    float uy_center,
    float uz_center
) {
    float nu_t = runtime_local_turbulent_nu(payload, in_ch, nx, ny, nz, x, y, z, ux_center, uy_center, uz_center);
    return RUNTIME_MOLECULAR_ALPHA0 + nu_t / fmax(1.0e-6f, RUNTIME_TURBULENT_PRANDTL);
}

inline int thermal_ddf_benchmark_enabled(int benchmark_preset) {
    return 0;
}

inline int thermal_bfecc_benchmark_enabled(int benchmark_preset) {
    return benchmark_preset == BENCH_PRESET_HEATED_CAVITY_2D || benchmark_preset == BENCH_PRESET_HEATED_CAVITY_3D;
}

inline float thermal_feq_opencl(int q, float temperature, float ux, float uy, float uz) {
    float cu = (float)TCX[q] * ux + (float)TCY[q] * uy + (float)TCZ[q] * uz;
    return TW[q] * temperature * (1.0f + cu / THERMAL_CS2);
}

inline float thermal_boundary_distribution_opencl(
    __global const float* thermal_f_read,
    int cells,
    int cell,
    int q
) {
    return thermal_f_read[TOPP[q] * cells + cell];
}

inline float thermal_streamed_value_opencl(
    __global const float* thermal_f_read,
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
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
    int cell,
    int x,
    int y,
    int z,
    int q,
    float self_temperature
) {
    int sx = x - TCX[q];
    int sy = y - TCY[q];
    int sz = z - TCZ[q];
    sx = wrap_axis_if_periodic(sx, nx, PERIODIC_AXIS_X, thermal_periodic_mask);
    sy = wrap_axis_if_periodic(sy, ny, PERIODIC_AXIS_Y, thermal_periodic_mask);
    sz = wrap_axis_if_periodic(sz, nz, PERIODIC_AXIS_Z, thermal_periodic_mask);
    if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz) {
        int face_kind = face_kind_for_oob(
            sx, sy, sz, nx, ny, nz,
            tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind
        );
        float4 face_data = face_data_for_oob(
            sx, sy, sz, nx, ny, nz,
            tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data
        );
        if (face_kind == 4) {
            return -thermal_f_read[TOPP[q] * cells + cell] + 2.0f * TW[q] * clampf(face_data.x, THERMAL_MIN, THERMAL_MAX);
        }
        if (face_kind == 5) {
            float wall_t = clampf(self_temperature + face_data.y, THERMAL_MIN, THERMAL_MAX);
            return -thermal_f_read[TOPP[q] * cells + cell] + 2.0f * TW[q] * wall_t;
        }
        return thermal_boundary_distribution_opencl(thermal_f_read, cells, cell, q);
    }
    int src = (sx * ny + sy) * nz + sz;
    if (payload[src * in_ch + 0] > 0.5f) {
        return thermal_boundary_distribution_opencl(thermal_f_read, cells, cell, q);
    }
    return thermal_f_read[q * cells + src];
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

kernel void apply_temperature_reference(
    __global const float* payload,
    __global float* temperature,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
    float state_nudge
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;

    int base = cell * in_ch;
    if (payload[base + 0] > 0.5f) {
        temperature[cell] = 0.0f;
        return;
    }
    if (in_ch <= 10 || state_nudge <= 0.0f) return;

    int yz = ny * nz;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / nz;
    int z = rem - y * nz;
    int edge_distance = min(min(min(x, y), z), min(min(nx - 1 - x, ny - 1 - y), nz - 1 - z));
    if (edge_distance >= 8) return;

    float eta = (8.0f - (float)edge_distance) / 8.0f;
    float local_state_nudge = state_nudge * eta * eta;
    float ref_temperature = clampf(payload[base + 10], THERMAL_MIN, THERMAL_MAX);
    temperature[cell] = clampf(mix(temperature[cell], ref_temperature, local_state_nudge), THERMAL_MIN, THERMAL_MAX);
}

)CLC"
R"CLC(
kernel void thermal_bfecc_forward(
    __global const float* payload,
    __global const float* temp_read,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
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
    float thermal_dt,
    __global float* temp_forward
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;
    int base = cell * in_ch;
    if (payload[base + 0] > 0.5f) {
        temp_forward[cell] = 0.0f;
        return;
    }
    int yz = ny * nz;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / nz;
    int z = rem - y * nz;
    float ux = payload[base + 5];
    float uy = payload[base + 6];
    float uz = payload[base + 7];
    temp_forward[cell] = sample_temperature_trilinear(
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
        (float)x - thermal_dt * ux,
        (float)y - thermal_dt * uy,
        (float)z - thermal_dt * uz,
        temp_read[cell]
    );
}

kernel void thermal_bfecc_correct(
    __global const float* payload,
    __global const float* temp_read,
    __global const float* temp_forward,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
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
    float thermal_dt,
    __global float* temp_corrected
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;
    int base = cell * in_ch;
    if (payload[base + 0] > 0.5f) {
        temp_corrected[cell] = 0.0f;
        return;
    }
    int yz = ny * nz;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / nz;
    int z = rem - y * nz;
    float ux = payload[base + 5];
    float uy = payload[base + 6];
    float uz = payload[base + 7];
    float t_center = temp_read[cell];
    float backward = sample_temperature_trilinear(
        temp_forward,
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
        (float)x + thermal_dt * ux,
        (float)y + thermal_dt * uy,
        (float)z + thermal_dt * uz,
        temp_forward[cell]
    );
    temp_corrected[cell] = clampf(t_center + 0.5f * (t_center - backward), THERMAL_MIN, THERMAL_MAX);
}

kernel void thermal_bfecc_finalize(
    __global const float* payload,
    __global const float* temp_read,
    __global const float* temp_corrected,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
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
    float thermal_dt,
    float thermal_diffusivity_eff,
    float thermal_cooling_eff,
    int benchmark_flags,
    int runtime_local_thermal_model,
    __global float* temp_out
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;
    int base = cell * in_ch;
    if (payload[base + 0] > 0.5f) {
        temp_out[cell] = 0.0f;
        return;
    }
    int yz = ny * nz;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / nz;
    int z = rem - y * nz;
    float ux = payload[base + 5];
    float uy = payload[base + 6];
    float uz = payload[base + 7];
    float t_center = temp_read[cell];
    float advected = sample_temperature_trilinear(
        temp_corrected,
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
        (float)x - thermal_dt * ux,
        (float)y - thermal_dt * uy,
        (float)z - thermal_dt * uz,
        temp_corrected[cell]
    );
    float txp0 = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x + 1, y, z, t_center);
    float txm0 = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x - 1, y, z, t_center);
    float typ0 = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y + 1, z, t_center);
    float tym0 = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y - 1, z, t_center);
    float tzp0 = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y, z + 1, t_center);
    float tzm0 = temperature_or_self(temp_read, payload, in_ch, nx, ny, nz, thermal_periodic_mask, tx_min_kind, tx_max_kind, ty_min_kind, ty_max_kind, tz_min_kind, tz_max_kind, tx_min_data, tx_max_data, ty_min_data, ty_max_data, tz_min_data, tz_max_data, x, y, z - 1, t_center);
    float local_min = fmin(fmin(fmin(txp0, txm0), fmin(typ0, tym0)), fmin(fmin(tzp0, tzm0), t_center));
    float local_max = fmax(fmax(fmax(txp0, txm0), fmax(typ0, tym0)), fmax(fmax(tzp0, tzm0), t_center));
    advected = clampf(advected, local_min, local_max);
    float thermal_diffusivity_local = thermal_diffusivity_eff;
    if (runtime_local_thermal_model != 0) {
        thermal_diffusivity_local = runtime_local_thermal_diffusivity(payload, in_ch, nx, ny, nz, x, y, z, ux, uy, uz);
    }
    float laplacian_t = (txp0 + txm0 + typ0 + tym0 + tzp0 + tzm0) - 6.0f * t_center;
    float thermal_source = 0.0f;
    if ((benchmark_flags & BENCH_DISABLE_INTERNAL_THERMAL_SOURCE) != 0) {
        thermal_source = 0.0f;
    } else if (in_ch > 9) {
        thermal_source = clampf(payload[base + 9], -THERMAL_SOURCE_MAX, THERMAL_SOURCE_MAX);
    }
    temp_out[cell] = clampf(
        advected + thermal_dt * (thermal_diffusivity_local * laplacian_t + thermal_source - thermal_cooling_eff * advected),
        THERMAL_MIN,
        THERMAL_MAX
    );
}

kernel void stream_collide_hydro_forced_step(
    __global const float* f_read,
    __global const float* payload,
    __global const float* temp_read,
    int in_ch, int nx, int ny, int nz, int cells, int tick, int benchmark_flags, int hydro_periodic_mask,
    float tau_shear_eff, float tau_normal_eff, float nu0_shear_eff, float boussinesq_beta_eff, float state_nudge,
    int x_min_kind, int x_max_kind, int y_min_kind, int y_max_kind, int z_min_kind, int z_max_kind,
    float4 x_min_data, float4 x_max_data, float4 y_min_data, float4 y_max_data, float4 z_min_data, float4 z_max_data,
    int benchmark_preset,
    __global float* f_write
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
        int sx = x - CX[q];
        int sy = y - CY[q];
        int sz = z - CZ[q];
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
        for (int q = 0; q < KQ; ++q) f_write[q * cells + cell] = f_local[q];
        return;
    }

    float rho = 0.0f, mx = 0.0f, my = 0.0f, mz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_local[q];
        rho += fq;
        mx += fq * (float)CX[q];
        my += fq * (float)CY[q];
        mz += fq * (float)CZ[q];
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

    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
    if (fan > 0.0f && fan_norm > 1e-8f) {
        float inv_norm = 1.0f / fan_norm;
        float noise_amp = (benchmark_flags & BENCH_DISABLE_FAN_NOISE) ? 0.0f : FAN_NOISE_AMP;
        float noise = 1.0f + noise_amp * signed_noise((uint)cell, (uint)tick);
        float target_speed = clampf(
            fan_norm * FAN_TARGET_SCALE * fmax(0.0f, noise),
            0.0f,
            FAN_TARGET_MAX
        );
        float fan_nx = fan_ux * inv_norm;
        float fan_ny = fan_uy * inv_norm;
        float fan_nz = fan_uz * inv_norm;
        float u_para = ux * fan_nx + uy * fan_ny + uz * fan_nz;
        float u_perp_x = ux - u_para * fan_nx;
        float u_perp_y = uy - u_para * fan_ny;
        float u_perp_z = uz - u_para * fan_nz;
        float axial_push = fmax(0.0f, target_speed - u_para);
        float speed_damp = 1.0f;
        if (speed_pre > FAN_SPEED_SOFT_CAP) {
            float r = (speed_pre - FAN_SPEED_SOFT_CAP) / fmax(1e-4f, FAN_SPEED_DAMP_WIDTH);
            speed_damp = 1.0f / (1.0f + r * r);
        }
        float beta = fan * FAN_BETA * speed_damp;
        fx = beta * rho_safe * (axial_push * fan_nx - FAN_PERP_DAMP * u_perp_x);
        fy = beta * rho_safe * (axial_push * fan_ny - FAN_PERP_DAMP * u_perp_y);
        fz = beta * rho_safe * (axial_push * fan_nz - FAN_PERP_DAMP * u_perp_z);
    }

    if (BOUSSINESQ_ENABLED && (benchmark_flags & BENCH_DISABLE_BUOYANCY) == 0) {
        float buoyancy = clampf(boussinesq_beta_eff * temp_center, -BOUSSINESQ_FORCE_MAX, BOUSSINESQ_FORCE_MAX);
        fy += rho_safe * buoyancy;
    }

    float dux = fx * inv_rho;
    float duy = fy * inv_rho;
    float duz = fz * inv_rho;
    ux += 0.5f * dux;
    uy += 0.5f * duy;
    uz += 0.5f * duz;
    int edge_distance = min(min(min(x, y), z), min(min(nx - 1 - x, ny - 1 - y), nz - 1 - z));
    float local_state_nudge = 0.0f;
    if (state_nudge > 0.0f && edge_distance < 8) {
        float eta = (8.0f - (float)edge_distance) / 8.0f;
        local_state_nudge = state_nudge * eta * eta;
    }
    ux = mix(ux, payload[base + 5], local_state_nudge);
    uy = mix(uy, payload[base + 6], local_state_nudge);
    uz = mix(uz, payload[base + 7], local_state_nudge);

    float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > MAX_SPEED * MAX_SPEED) {
        float scale = MAX_SPEED * rsqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
    }

)CLC"
R"CLC(
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

    float tau_shear_base = tau_shear_eff;
    float tau_normal_base = clampf(tau_normal_eff, TAU_NORMAL_MIN, TAU_NORMAL_MAX);
    float tau_shear_local = tau_shear_base;
    float tau_normal_local = tau_normal_base;
    if (SGS_ENABLED && (benchmark_flags & BENCH_DISABLE_SGS) == 0) {
        float nu0 = fmax(1e-6f, clamp_shear_nu(nu0_shear_eff));
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
        tau_shear_local = clampf(tau_from_shear_nu(nu0 + nu_t), TAU_SHEAR_MIN, TAU_SHEAR_MAX);
        tau_normal_local = clampf(
            tau_normal_base + (tau_shear_local - tau_shear_base) * SGS_BULK_COUPLING,
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
kernel void stream_collide_tgv_step(
    __global const float* f_read,
    int nx,
    int ny,
    int nz,
    int cells,
    float tau_shear_eff,
    float tau_normal_eff,
    __global float* f_write
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;

    int yz = ny * nz;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / nz;
    int z = rem - y * nz;

    float f_local[KQ];
    for (int q = 0; q < KQ; ++q) {
        int sx = x - CX[q];
        int sy = y - CY[q];
        int sz = z - CZ[q];
        if (sx < 0) sx += nx;
        else if (sx >= nx) sx -= nx;
        if (sy < 0) sy += ny;
        else if (sy >= ny) sy -= ny;
        if (sz < 0) sz += nz;
        else if (sz >= nz) sz -= nz;
        int src = (sx * ny + sy) * nz + sz;
        f_local[q] = f_read[q * cells + src];
    }

    float rho = 0.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_local[q];
        rho += fq;
        mx += fq * (float)CX[q];
        my += fq * (float)CY[q];
        mz += fq * (float)CZ[q];
    }

    rho = clampf(rho, RHO_MIN, RHO_MAX);
    float inv_rho = 1.0f / fmax(1.0e-6f, rho);
    float ux = mx * inv_rho;
    float uy = my * inv_rho;
    float uz = mz * inv_rho;
    float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > MAX_SPEED * MAX_SPEED && speed2 > 0.0f) {
        float scale = MAX_SPEED * rsqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
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
        float cx = (float)CX[q];
        float cy = (float)CY[q];
        float cz = (float)CZ[q];
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

    float tau_shear_base = clampf(tau_shear_eff, TAU_SHEAR_MIN, TAU_SHEAR_MAX);
    float tau_normal_base = clampf(tau_normal_eff, TAU_NORMAL_MIN, TAU_NORMAL_MAX);
    float omega_diag = 1.0f / tau_normal_base;
    float omega_offdiag = 1.0f / tau_shear_base;
    float s_diag = clampf(omega_diag, 0.0f, 1.95f);
    float s_offdiag = clampf(omega_offdiag, 0.0f, 1.95f);
    float eq_second = rho * CS2;

    central_post[MI(0, 0, 0)] = rho;
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
    float inv_rho_safe = 1.0f / fmax(1.0e-6f, rho);

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

    float rho_corr = 0.0f;
    float mx_corr = 0.0f;
    float my_corr = 0.0f;
    float mz_corr = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_post_local[q];
        rho_corr += fq;
        mx_corr += fq * (float)CX[q];
        my_corr += fq * (float)CY[q];
        mz_corr += fq * (float)CZ[q];
    }
    float inv_rho_corr = 1.0f / fmax(1.0e-6f, rho_corr);
    float ux_corr = mx_corr * inv_rho_corr;
    float uy_corr = my_corr * inv_rho_corr;
    float uz_corr = mz_corr * inv_rho_corr;
    for (int q = 0; q < KQ; ++q) {
        f_write[q * cells + cell] =
            f_post_local[q] + feq(q, rho, ux, uy, uz) - feq(q, fmax(1.0e-6f, rho_corr), ux_corr, uy_corr, uz_corr);
    }
}

)CLC"
R"CLC(
kernel void stream_collide_hydro_benchmark_step(
    __global const float* f_read,
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int cells,
    int benchmark_flags,
    int hydro_periodic_mask,
    float tau_shear_eff,
    float tau_normal_eff,
    float nu0_shear_eff,
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
    int benchmark_preset,
    __global float* f_write
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

        int sx = x - CX[q];
        int sy = y - CY[q];
        int sz = z - CZ[q];
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
        for (int q = 0; q < KQ; ++q) f_write[q * cells + cell] = f_local[q];
        return;
    }

    float rho = 0.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_local[q];
        rho += fq;
        mx += fq * (float)CX[q];
        my += fq * (float)CY[q];
        mz += fq * (float)CZ[q];
    }

    rho = clampf(rho, RHO_MIN, RHO_MAX);
    float inv_rho = 1.0f / fmax(1.0e-6f, rho);
    float ux = mx * inv_rho;
    float uy = my * inv_rho;
    float uz = mz * inv_rho;
    float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > MAX_SPEED * MAX_SPEED && speed2 > 0.0f) {
        float scale = MAX_SPEED * rsqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
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
        float cx = (float)CX[q];
        float cy = (float)CY[q];
        float cz = (float)CZ[q];
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

    float tau_shear_base = clampf(tau_shear_eff, TAU_SHEAR_MIN, TAU_SHEAR_MAX);
    float tau_normal_base = clampf(tau_normal_eff, TAU_NORMAL_MIN, TAU_NORMAL_MAX);
    float tau_shear_local = tau_shear_base;
    float tau_normal_local = tau_normal_base;
    if (SGS_ENABLED && (benchmark_flags & BENCH_DISABLE_SGS) == 0) {
        float nu0 = fmax(1e-6f, clamp_shear_nu(nu0_shear_eff));
        float neq_xx = central_pre[MI(2, 0, 0)] - rho * CS2;
        float neq_yy = central_pre[MI(0, 2, 0)] - rho * CS2;
        float neq_zz = central_pre[MI(0, 0, 2)] - rho * CS2;
        float neq_xy = central_pre[MI(1, 1, 0)];
        float neq_xz = central_pre[MI(1, 0, 1)];
        float neq_yz = central_pre[MI(0, 1, 1)];
        float q_norm2 = neq_xx * neq_xx + neq_yy * neq_yy + neq_zz * neq_zz
                        + 2.0f * (neq_xy * neq_xy + neq_xz * neq_xz + neq_yz * neq_yz);
        float q_mag = sqrt(fmax(0.0f, q_norm2));
        float s_mag = q_mag / fmax(1e-6f, 2.0f * rho * nu0);
        float nu_t = SGS_C2 * s_mag;
        nu_t = fmin(nu_t, SGS_NUT_TO_NU0_MAX * nu0);
        tau_shear_local = clampf(tau_from_shear_nu(nu0 + nu_t), TAU_SHEAR_MIN, TAU_SHEAR_MAX);
        tau_normal_local = clampf(
            tau_normal_base + (tau_shear_local - tau_shear_base) * SGS_BULK_COUPLING,
            TAU_NORMAL_MIN,
            TAU_NORMAL_MAX
        );
    }

    float omega_diag = 1.0f / tau_normal_local;
    float omega_offdiag = 1.0f / tau_shear_local;
    float s_diag = clampf(omega_diag, 0.0f, 1.95f);
    float s_offdiag = clampf(omega_offdiag, 0.0f, 1.95f);
    float eq_second = rho * CS2;

    central_post[MI(0, 0, 0)] = rho;
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
    float inv_rho_safe = 1.0f / fmax(1.0e-6f, rho);

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

    float rho_corr = 0.0f;
    float mx_corr = 0.0f;
    float my_corr = 0.0f;
    float mz_corr = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_post_local[q];
        rho_corr += fq;
        mx_corr += fq * (float)CX[q];
        my_corr += fq * (float)CY[q];
        mz_corr += fq * (float)CZ[q];
    }
    float inv_rho_corr = 1.0f / fmax(1.0e-6f, rho_corr);
    float ux_corr = mx_corr * inv_rho_corr;
    float uy_corr = my_corr * inv_rho_corr;
    float uz_corr = mz_corr * inv_rho_corr;
    float alpha_sponge = (benchmark_flags & BENCH_DISABLE_SPONGE) ? 0.0f : sponge_alpha(nx, ny, nz, x, y, z);
    float keep_sponge = 1.0f - alpha_sponge;
    for (int q = 0; q < KQ; ++q) {
        float f_next =
            f_post_local[q] + feq(q, rho, ux, uy, uz) - feq(q, fmax(1.0e-6f, rho_corr), ux_corr, uy_corr, uz_corr);
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

kernel void output_macro_strided(
    __global const float* f,
    __global const float* payload,
    int in_ch,
    int nx,
    int ny,
    int nz,
    int stride,
    int sx,
    int sy,
    int sz,
    __global float* out
) {
    int atlas_cell = (int)get_global_id(0);
    int atlas_cells = sx * sy * sz;
    if (atlas_cell >= atlas_cells) return;

    int ayz = sy * sz;
    int ax = atlas_cell / ayz;
    int rem = atlas_cell - ax * ayz;
    int ay = rem / sz;
    int az = rem - ay * sz;

    int gx = min(nx - 1, ax * stride);
    int gy = min(ny - 1, ay * stride);
    int gz = min(nz - 1, az * stride);
    int cell = (gx * ny + gy) * nz + gz;
    int cells = nx * ny * nz;
    int out_base = atlas_cell * 4;

    if (payload[cell * in_ch + 0] > 0.5f) {
        out[out_base + 0] = 0.0f;
        out[out_base + 1] = 0.0f;
        out[out_base + 2] = 0.0f;
        out[out_base + 3] = 0.0f;
        return;
    }

    float rho = 0.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f[q * cells + cell];
        rho += fq;
        ux += fq * (float)CX[q];
        uy += fq * (float)CY[q];
        uz += fq * (float)CZ[q];
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

std::string format_opencl_api_error(const char* api, cl_int err) {
    std::ostringstream oss;
    oss << api << " failed (" << static_cast<int>(err) << ")";
    return oss.str();
}

void release_opencl_runtime() {
    if (g_opencl.k_output_strided) clReleaseKernel(g_opencl.k_output_strided);
    if (g_opencl.k_apply_temperature_reference) clReleaseKernel(g_opencl.k_apply_temperature_reference);
    if (g_opencl.k_thermal_bfecc_finalize) clReleaseKernel(g_opencl.k_thermal_bfecc_finalize);
    if (g_opencl.k_thermal_bfecc_correct) clReleaseKernel(g_opencl.k_thermal_bfecc_correct);
    if (g_opencl.k_thermal_bfecc_forward) clReleaseKernel(g_opencl.k_thermal_bfecc_forward);
    if (g_opencl.k_output) clReleaseKernel(g_opencl.k_output);
    if (g_opencl.k_stream_collide_hydro_forced) clReleaseKernel(g_opencl.k_stream_collide_hydro_forced);
    if (g_opencl.k_stream_collide_hydro_bench) clReleaseKernel(g_opencl.k_stream_collide_hydro_bench);
    if (g_opencl.k_stream_collide_tgv) clReleaseKernel(g_opencl.k_stream_collide_tgv);
    if (g_opencl.k_init) clReleaseKernel(g_opencl.k_init);
    if (g_opencl.program) clReleaseProgram(g_opencl.program);
    if (g_opencl.queue) clReleaseCommandQueue(g_opencl.queue);
    if (g_opencl.context) clReleaseContext(g_opencl.context);
    g_opencl = OpenClRuntime{};
}

void release_context_gpu_buffers(ContextState& ctx) {
    if (ctx.d_thermal_f_post) clReleaseMemObject(ctx.d_thermal_f_post);
    if (ctx.d_thermal_f) clReleaseMemObject(ctx.d_thermal_f);
    if (ctx.d_temp_scratch) clReleaseMemObject(ctx.d_temp_scratch);
    if (ctx.d_temp_next) clReleaseMemObject(ctx.d_temp_next);
    if (ctx.d_temp) clReleaseMemObject(ctx.d_temp);
    if (ctx.d_output) clReleaseMemObject(ctx.d_output);
    if (ctx.d_f_post) clReleaseMemObject(ctx.d_f_post);
    if (ctx.d_f) clReleaseMemObject(ctx.d_f);
    if (ctx.d_payload) clReleaseMemObject(ctx.d_payload);
    ctx.d_thermal_f_post = ctx.d_thermal_f = ctx.d_temp_scratch = ctx.d_temp_next = ctx.d_temp = ctx.d_output = ctx.d_f_post = ctx.d_f = ctx.d_payload = nullptr;
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
    cl_kernel k_apply_temperature_reference = clCreateKernel(program, "apply_temperature_reference", &err);
    cl_kernel k_thermal_bfecc_forward = clCreateKernel(program, "thermal_bfecc_forward", &err);
    cl_kernel k_thermal_bfecc_correct = clCreateKernel(program, "thermal_bfecc_correct", &err);
    cl_kernel k_thermal_bfecc_finalize = clCreateKernel(program, "thermal_bfecc_finalize", &err);
    cl_kernel k_stream_collide_tgv = clCreateKernel(program, "stream_collide_tgv_step", &err);
    cl_kernel k_stream_collide_hydro_bench = clCreateKernel(program, "stream_collide_hydro_benchmark_step", &err);
    cl_kernel k_stream_collide_hydro_forced = clCreateKernel(program, "stream_collide_hydro_forced_step", &err);
    cl_kernel k_output = clCreateKernel(program, "output_macro", &err);
    cl_kernel k_output_strided = clCreateKernel(program, "output_macro_strided", &err);

    if (!k_init || !k_apply_temperature_reference || !k_thermal_bfecc_forward || !k_thermal_bfecc_correct || !k_thermal_bfecc_finalize
        || !k_stream_collide_tgv || !k_stream_collide_hydro_bench || !k_stream_collide_hydro_forced || !k_output
        || !k_output_strided) {
        if (k_init) clReleaseKernel(k_init);
        if (k_apply_temperature_reference) clReleaseKernel(k_apply_temperature_reference);
        if (k_thermal_bfecc_forward) clReleaseKernel(k_thermal_bfecc_forward);
        if (k_thermal_bfecc_correct) clReleaseKernel(k_thermal_bfecc_correct);
        if (k_thermal_bfecc_finalize) clReleaseKernel(k_thermal_bfecc_finalize);
        if (k_stream_collide_tgv) clReleaseKernel(k_stream_collide_tgv);
        if (k_stream_collide_hydro_bench) clReleaseKernel(k_stream_collide_hydro_bench);
        if (k_stream_collide_hydro_forced) clReleaseKernel(k_stream_collide_hydro_forced);
        if (k_output) clReleaseKernel(k_output);
        if (k_output_strided) clReleaseKernel(k_output_strided);
        clReleaseProgram(program); clReleaseCommandQueue(queue); clReleaseContext(context);
        g_opencl.error = "Kernel creation failed"; return false;
    }

    g_opencl.context = context; g_opencl.queue = queue; g_opencl.program = program;
    g_opencl.k_init = k_init;
    g_opencl.k_apply_temperature_reference = k_apply_temperature_reference;
    g_opencl.k_thermal_bfecc_forward = k_thermal_bfecc_forward;
    g_opencl.k_thermal_bfecc_correct = k_thermal_bfecc_correct;
    g_opencl.k_thermal_bfecc_finalize = k_thermal_bfecc_finalize;
    g_opencl.k_stream_collide_tgv = k_stream_collide_tgv;
    g_opencl.k_stream_collide_hydro_bench = k_stream_collide_hydro_bench;
    g_opencl.k_stream_collide_hydro_forced = k_stream_collide_hydro_forced;
    g_opencl.k_output = k_output;
    g_opencl.k_output_strided = k_output_strided;
    g_opencl.platform = selected_platform; g_opencl.device = selected_device;
    g_opencl.available = true; g_opencl.device_name = read_device_name(selected_device);
    return true;
}

bool ensure_context_gpu_buffers(ContextState& ctx) {
    if (!g_opencl.available || ctx.cells == 0) return false;
    if (ctx.gpu_buffers_ready) return true;

    const std::size_t payload_bytes = ctx.cells * g_cfg.input_channels * sizeof(float);
    const std::size_t dist_bytes = ctx.cells * kQ * sizeof(float);
    const std::size_t thermal_dist_bytes = ctx.cells * kThermalQ * sizeof(float);
    const std::size_t output_bytes = ctx.cells * g_cfg.output_channels * sizeof(float);
    const std::size_t temp_bytes = ctx.cells * sizeof(float);

    auto create_buffer = [&](cl_mem& target, cl_mem_flags flags, std::size_t bytes, const char* label) -> bool {
        cl_int err = CL_SUCCESS;
        target = clCreateBuffer(g_opencl.context, flags, bytes, nullptr, &err);
        if (err != CL_SUCCESS || !target) {
            g_opencl.error = format_opencl_api_error(label, err);
            return false;
        }
        return true;
    };

    if (!create_buffer(ctx.d_payload, CL_MEM_READ_ONLY, payload_bytes, "clCreateBuffer(d_payload)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (!create_buffer(ctx.d_f, CL_MEM_READ_WRITE, dist_bytes, "clCreateBuffer(d_f)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (!create_buffer(ctx.d_f_post, CL_MEM_READ_WRITE, dist_bytes, "clCreateBuffer(d_f_post)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (!create_buffer(ctx.d_output, CL_MEM_WRITE_ONLY, output_bytes, "clCreateBuffer(d_output)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (!create_buffer(ctx.d_temp, CL_MEM_READ_WRITE, temp_bytes, "clCreateBuffer(d_temp)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (!create_buffer(ctx.d_temp_next, CL_MEM_READ_WRITE, temp_bytes, "clCreateBuffer(d_temp_next)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (!create_buffer(ctx.d_temp_scratch, CL_MEM_READ_WRITE, temp_bytes, "clCreateBuffer(d_temp_scratch)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    if (thermal_ddf_benchmark_active()) {
        if (!create_buffer(ctx.d_thermal_f, CL_MEM_READ_WRITE, thermal_dist_bytes, "clCreateBuffer(d_thermal_f)")) {
            release_context_gpu_buffers(ctx);
            return false;
        }
        if (!create_buffer(ctx.d_thermal_f_post, CL_MEM_READ_WRITE, thermal_dist_bytes, "clCreateBuffer(d_thermal_f_post)")) {
            release_context_gpu_buffers(ctx);
            return false;
        }
    }

    if (!ctx.d_payload || !ctx.d_f || !ctx.d_f_post || !ctx.d_output || !ctx.d_temp || !ctx.d_temp_next || !ctx.d_temp_scratch
        || (thermal_ddf_benchmark_active() && (!ctx.d_thermal_f || !ctx.d_thermal_f_post))) {
        g_opencl.error = "ensure_context_gpu_buffers incomplete allocation";
        release_context_gpu_buffers(ctx); return false;
    }
    ctx.gpu_buffers_ready = true; ctx.gpu_initialized = false;
    return true;
}

cl_int enqueue_kernel_1d(cl_kernel kernel, int cells) {
    const size_t global_size = static_cast<size_t>(cells);
    return clEnqueueNDRangeKernel(g_opencl.queue, kernel, 1, nullptr, &global_size, nullptr, 0, nullptr, nullptr);
}

bool opencl_step(
    ContextState& ctx,
    const float* payload,
    const int* overlay_cells,
    const float* overlay_values,
    int overlay_count,
    float* out,
    StepTiming& timing
) {
    if (!ensure_context_gpu_buffers(ctx)) return false;

    auto fail_cl = [&](const char* api, cl_int err) -> bool {
        g_opencl.error = format_opencl_api_error(api, err);
        return false;
    };
    auto stage_fence = [&](const char* stage) -> bool {
        if (!benchmark_mode_active()) return true;
        const std::string label = std::string("clFinish(") + stage + ")";
        cl_int finish_err = clFinish(g_opencl.queue);
        if (finish_err != CL_SUCCESS) return fail_cl(label.c_str(), finish_err);
        return true;
    };

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
    {
        constexpr std::size_t kPayloadIncrementalMaxRanges = 96;
        const std::size_t payload_values = ctx.cells * static_cast<std::size_t>(g_cfg.input_channels);
        const std::size_t cell_bytes = static_cast<std::size_t>(g_cfg.input_channels) * sizeof(float);
        thread_local std::vector<int> overlay_lookup;
        thread_local std::vector<int> overlay_lookup_touched;
        thread_local std::vector<float> upload_staging;
        if (overlay_lookup.size() != ctx.cells) {
            overlay_lookup.assign(ctx.cells, -1);
        } else {
            for (int cell : overlay_lookup_touched) {
                if (cell >= 0 && static_cast<std::size_t>(cell) < overlay_lookup.size()) {
                    overlay_lookup[static_cast<std::size_t>(cell)] = -1;
                }
            }
        }
        overlay_lookup_touched.clear();
        for (int overlay_index = 0; overlay_index < overlay_count; ++overlay_index) {
            const int cell = overlay_cells[overlay_index];
            if (cell < 0 || static_cast<std::size_t>(cell) >= ctx.cells) {
                continue;
            }
            if (overlay_lookup[static_cast<std::size_t>(cell)] < 0) {
                overlay_lookup_touched.push_back(cell);
            }
            overlay_lookup[static_cast<std::size_t>(cell)] = overlay_index;
        }
        auto write_effective_cell = [&](std::size_t cell, float* dst) {
            const std::size_t base = cell * static_cast<std::size_t>(g_cfg.input_channels);
            std::memcpy(dst, payload + base, cell_bytes);
            if (cell >= overlay_lookup.size()) {
                return;
            }
            const int overlay_index = overlay_lookup[cell];
            if (overlay_index < 0) {
                return;
            }
            const float* overlay = overlay_values + static_cast<std::size_t>(overlay_index) * kSparseOverlayChannels;
            dst[kChannelStateVx] = overlay[kSparseOverlayStateVx];
            dst[kChannelStateVy] = overlay[kSparseOverlayStateVy];
            dst[kChannelStateVz] = overlay[kSparseOverlayStateVz];
            dst[kChannelStateP] = overlay[kSparseOverlayStateP];
            if (g_cfg.input_channels > kChannelThermalSource) {
                dst[kChannelThermalSource] = overlay[kSparseOverlayThermalSource];
            }
            if (g_cfg.input_channels > kChannelStateTemp) {
                dst[kChannelStateTemp] = overlay[kSparseOverlayStateTemp];
            }
        };
        auto full_upload = [&]() -> bool {
            if (ctx.payload_cache.size() != payload_values) {
                ctx.payload_cache.assign(payload_values, 0.0f);
            }
            if (overlay_count <= 0) {
                std::memcpy(ctx.payload_cache.data(), payload, payload_bytes);
            } else {
                for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
                    write_effective_cell(cell, ctx.payload_cache.data() + cell * static_cast<std::size_t>(g_cfg.input_channels));
                }
            }
            cl_int upload_err = clEnqueueWriteBuffer(
                g_opencl.queue,
                ctx.d_payload,
                CL_TRUE,
                0,
                payload_bytes,
                ctx.payload_cache.data(),
                0,
                nullptr,
                nullptr
            );
            if (upload_err != CL_SUCCESS) return fail_cl("clEnqueueWriteBuffer(d_payload)", upload_err);
            return true;
        };

        bool upload_ok = true;
        if (!ctx.gpu_initialized || ctx.payload_cache.size() != payload_values) {
            upload_ok = full_upload();
        } else {
            std::vector<std::array<std::size_t, 2>> changed_ranges;
            changed_ranges.reserve(32);
            char* cached_bytes_ptr = reinterpret_cast<char*>(ctx.payload_cache.data());
            std::vector<float> desired_cell_buffer(static_cast<std::size_t>(g_cfg.input_channels), 0.0f);
            bool in_range = false;
            std::size_t range_start_cell = 0;
            std::size_t changed_cells = 0;
            for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
                const std::size_t byte_offset = cell * cell_bytes;
                bool changed = false;
                if (overlay_count <= 0 || overlay_lookup[cell] < 0) {
                    changed = std::memcmp(
                        cached_bytes_ptr + byte_offset,
                        reinterpret_cast<const char*>(payload) + byte_offset,
                        cell_bytes
                    ) != 0;
                } else {
                    write_effective_cell(cell, desired_cell_buffer.data());
                    changed = std::memcmp(cached_bytes_ptr + byte_offset, desired_cell_buffer.data(), cell_bytes) != 0;
                }
                if (changed) {
                    ++changed_cells;
                    if (!in_range) {
                        range_start_cell = cell;
                        in_range = true;
                    }
                } else if (in_range) {
                    changed_ranges.push_back({range_start_cell, cell - range_start_cell});
                    in_range = false;
                }
            }
            if (in_range) {
                changed_ranges.push_back({range_start_cell, ctx.cells - range_start_cell});
            }

            if (changed_cells == 0) {
                upload_ok = true;
            } else if (changed_ranges.size() > kPayloadIncrementalMaxRanges || changed_cells * 3 >= ctx.cells) {
                upload_ok = full_upload();
            } else {
                for (const std::array<std::size_t, 2>& range : changed_ranges) {
                    const std::size_t byte_offset = range[0] * cell_bytes;
                    const std::size_t cells_in_range = range[1];
                    const std::size_t values_in_range = cells_in_range * static_cast<std::size_t>(g_cfg.input_channels);
                    upload_staging.resize(values_in_range);
                    for (std::size_t local = 0; local < cells_in_range; ++local) {
                        const std::size_t cell = range[0] + local;
                        write_effective_cell(
                            cell,
                            upload_staging.data() + local * static_cast<std::size_t>(g_cfg.input_channels)
                        );
                    }
                    const std::size_t byte_count = values_in_range * sizeof(float);
                    std::memcpy(cached_bytes_ptr + byte_offset, upload_staging.data(), byte_count);
                    cl_int upload_err = clEnqueueWriteBuffer(
                        g_opencl.queue,
                        ctx.d_payload,
                        CL_TRUE,
                        byte_offset,
                        byte_count,
                        upload_staging.data(),
                        0,
                        nullptr,
                        nullptr
                    );
                    if (upload_err != CL_SUCCESS) {
                        upload_ok = fail_cl("clEnqueueWriteBuffer(d_payload_range)", upload_err);
                        break;
                    }
                }
            }
        }
        if (!upload_ok) {
            return false;
        }
    }
    if (!stage_fence("after_payload_upload")) return false;
    timing.payload_copy_ms += elapsed_ms(upload_begin, Clock::now());

    auto solver_begin = Clock::now();
    cl_int err = CL_SUCCESS;
    if (!ctx.gpu_initialized) {
        rebuild_thermal_distributions_from_temperature(ctx);
        err |= clSetKernelArg(g_opencl.k_init, 0, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_init, 1, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_init, 2, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_init, 3, sizeof(cl_mem), &ctx.d_f);
        err |= clSetKernelArg(g_opencl.k_init, 4, sizeof(cl_mem), &ctx.d_f_post);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_init)", err);
        err = enqueue_kernel_1d(g_opencl.k_init, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_init)", err);

        const std::size_t temp_bytes = ctx.cells * sizeof(float);
        err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueWriteBuffer(d_temp)", err);
        err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp_next, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueWriteBuffer(d_temp_next)", err);
        if (thermal_ddf_benchmark_active()) {
            const std::size_t thermal_bytes = ctx.cells * kThermalQ * sizeof(float);
            err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_thermal_f, CL_TRUE, 0, thermal_bytes, ctx.thermal_f.data(), 0, nullptr, nullptr);
            if (err != CL_SUCCESS) return fail_cl("clEnqueueWriteBuffer(d_thermal_f)", err);
            err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_thermal_f_post, CL_TRUE, 0, thermal_bytes, ctx.thermal_f.data(), 0, nullptr, nullptr);
            if (err != CL_SUCCESS) return fail_cl("clEnqueueWriteBuffer(d_thermal_f_post)", err);
        }
        if (!stage_fence("after_init")) return false;
        ctx.gpu_initialized = true;
    }

    const int tick_i32 = static_cast<int>(ctx.step_counter & 0x7FFFFFFF);
    const int benchmark_flags = opencl_benchmark_flags();
    const int hydro_periodic_mask = opencl_hydrodynamic_periodic_mask();
    const int thermal_periodic_mask = opencl_thermal_periodic_mask();
    const int benchmark_preset = benchmark_mode_active() ? g_benchmark_cfg.preset : 0;
    const bool use_tgv_fastpath = benchmark_preset == AERO_LBM_BENCHMARK_PRESET_TAYLOR_GREEN_3D;
    const bool use_hydro_bench_fastpath =
        benchmark_preset == AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D
        || benchmark_preset == AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_3D
        || benchmark_preset == AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D;
    const bool use_bfecc_thermal =
        !use_tgv_fastpath
        && !use_hydro_bench_fastpath
        && thermal_bfecc_active();
    const auto hydro_face_kinds = opencl_hydrodynamic_face_kinds();
    const auto hydro_face_data = opencl_hydrodynamic_face_data();
    const auto thermal_face_kinds = opencl_thermal_face_kinds();
    const auto thermal_face_data = opencl_thermal_face_data();
    const auto tau_pair = opencl_effective_tau_pair();
    const float base_nu_shear = opencl_effective_base_nu_shear();
    const auto thermal_transport = opencl_effective_thermal_transport();
    const int thermal_update_stride = opencl_effective_thermal_update_stride();
    const int runtime_local_thermal_model = benchmark_mode_active() ? 0 : 1;
    
    // Ping-Pong 双重缓冲交换
    cl_mem read_buf = (ctx.step_counter % 2 == 0) ? ctx.d_f : ctx.d_f_post;
    cl_mem write_buf = (ctx.step_counter % 2 == 0) ? ctx.d_f_post : ctx.d_f;
    cl_mem temp_read = (ctx.step_counter % 2 == 0) ? ctx.d_temp : ctx.d_temp_next;
    cl_mem temp_write = (ctx.step_counter % 2 == 0) ? ctx.d_temp_next : ctx.d_temp;
    const float state_nudge = effective_state_nudge();

    err = CL_SUCCESS;
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 0, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 3, sizeof(int), &ctx.nx);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 4, sizeof(int), &ctx.ny);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 5, sizeof(int), &ctx.nz);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 6, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 7, sizeof(float), &state_nudge);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 1, sizeof(cl_mem), &temp_read);
    if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_apply_temperature_reference,temp_read)", err);
    err = enqueue_kernel_1d(g_opencl.k_apply_temperature_reference, cells_i32);
    if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_apply_temperature_reference,temp_read)", err);
    err = CL_SUCCESS;
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 0, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 1, sizeof(cl_mem), &temp_write);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 3, sizeof(int), &ctx.nx);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 4, sizeof(int), &ctx.ny);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 5, sizeof(int), &ctx.nz);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 6, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_apply_temperature_reference, 7, sizeof(float), &state_nudge);
    if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_apply_temperature_reference,temp_write)", err);
    err = enqueue_kernel_1d(g_opencl.k_apply_temperature_reference, cells_i32);
    if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_apply_temperature_reference,temp_write)", err);
    if (use_bfecc_thermal) {
        const float thermal_dt = static_cast<float>(thermal_update_stride);

        err = CL_SUCCESS;
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 0, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 1, sizeof(cl_mem), &temp_read);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 2, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 3, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 4, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 5, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 6, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 7, sizeof(int), &thermal_periodic_mask);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 8, sizeof(int), &thermal_face_kinds[0]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 9, sizeof(int), &thermal_face_kinds[1]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 10, sizeof(int), &thermal_face_kinds[2]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 11, sizeof(int), &thermal_face_kinds[3]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 12, sizeof(int), &thermal_face_kinds[4]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 13, sizeof(int), &thermal_face_kinds[5]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 14, sizeof(OpenClFaceData), thermal_face_data[0].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 15, sizeof(OpenClFaceData), thermal_face_data[1].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 16, sizeof(OpenClFaceData), thermal_face_data[2].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 17, sizeof(OpenClFaceData), thermal_face_data[3].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 18, sizeof(OpenClFaceData), thermal_face_data[4].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 19, sizeof(OpenClFaceData), thermal_face_data[5].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 20, sizeof(float), &thermal_dt);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_forward, 21, sizeof(cl_mem), &ctx.d_temp_scratch);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_thermal_bfecc_forward)", err);
        err = enqueue_kernel_1d(g_opencl.k_thermal_bfecc_forward, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_thermal_bfecc_forward)", err);

        err = CL_SUCCESS;
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 0, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 1, sizeof(cl_mem), &temp_read);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 2, sizeof(cl_mem), &ctx.d_temp_scratch);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 3, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 4, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 5, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 6, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 7, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 8, sizeof(int), &thermal_periodic_mask);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 9, sizeof(int), &thermal_face_kinds[0]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 10, sizeof(int), &thermal_face_kinds[1]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 11, sizeof(int), &thermal_face_kinds[2]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 12, sizeof(int), &thermal_face_kinds[3]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 13, sizeof(int), &thermal_face_kinds[4]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 14, sizeof(int), &thermal_face_kinds[5]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 15, sizeof(OpenClFaceData), thermal_face_data[0].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 16, sizeof(OpenClFaceData), thermal_face_data[1].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 17, sizeof(OpenClFaceData), thermal_face_data[2].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 18, sizeof(OpenClFaceData), thermal_face_data[3].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 19, sizeof(OpenClFaceData), thermal_face_data[4].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 20, sizeof(OpenClFaceData), thermal_face_data[5].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 21, sizeof(float), &thermal_dt);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_correct, 22, sizeof(cl_mem), &temp_write);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_thermal_bfecc_correct)", err);
        err = enqueue_kernel_1d(g_opencl.k_thermal_bfecc_correct, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_thermal_bfecc_correct)", err);

        err = CL_SUCCESS;
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 0, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 1, sizeof(cl_mem), &temp_read);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 2, sizeof(cl_mem), &temp_write);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 3, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 4, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 5, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 6, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 7, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 8, sizeof(int), &thermal_periodic_mask);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 9, sizeof(int), &thermal_face_kinds[0]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 10, sizeof(int), &thermal_face_kinds[1]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 11, sizeof(int), &thermal_face_kinds[2]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 12, sizeof(int), &thermal_face_kinds[3]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 13, sizeof(int), &thermal_face_kinds[4]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 14, sizeof(int), &thermal_face_kinds[5]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 15, sizeof(OpenClFaceData), thermal_face_data[0].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 16, sizeof(OpenClFaceData), thermal_face_data[1].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 17, sizeof(OpenClFaceData), thermal_face_data[2].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 18, sizeof(OpenClFaceData), thermal_face_data[3].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 19, sizeof(OpenClFaceData), thermal_face_data[4].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 20, sizeof(OpenClFaceData), thermal_face_data[5].data());
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 21, sizeof(float), &thermal_dt);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 22, sizeof(float), &thermal_transport[0]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 23, sizeof(float), &thermal_transport[1]);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 24, sizeof(int), &benchmark_flags);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 25, sizeof(int), &runtime_local_thermal_model);
        err |= clSetKernelArg(g_opencl.k_thermal_bfecc_finalize, 26, sizeof(cl_mem), &ctx.d_temp_scratch);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_thermal_bfecc_finalize)", err);
        err = enqueue_kernel_1d(g_opencl.k_thermal_bfecc_finalize, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_thermal_bfecc_finalize)", err);

        err = clEnqueueCopyBuffer(g_opencl.queue, ctx.d_temp_scratch, temp_write, 0, 0, ctx.cells * sizeof(float), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueCopyBuffer(temp_scratch->temp_write)", err);
        if (!stage_fence("after_thermal_bfecc")) return false;
        temp_read = temp_write;
    }

    err = CL_SUCCESS;
    if (use_tgv_fastpath) {
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 0, sizeof(cl_mem), &read_buf);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 1, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 2, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 3, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 4, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 5, sizeof(float), &tau_pair[0]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 6, sizeof(float), &tau_pair[1]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_tgv, 7, sizeof(cl_mem), &write_buf);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_stream_collide_tgv)", err);
        err = enqueue_kernel_1d(g_opencl.k_stream_collide_tgv, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_stream_collide_tgv)", err);
    } else if (use_hydro_bench_fastpath) {
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 0, sizeof(cl_mem), &read_buf);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 1, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 2, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 3, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 4, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 5, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 6, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 7, sizeof(int), &benchmark_flags);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 8, sizeof(int), &hydro_periodic_mask);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 9, sizeof(float), &tau_pair[0]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 10, sizeof(float), &tau_pair[1]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 11, sizeof(float), &base_nu_shear);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 12, sizeof(int), &hydro_face_kinds[0]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 13, sizeof(int), &hydro_face_kinds[1]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 14, sizeof(int), &hydro_face_kinds[2]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 15, sizeof(int), &hydro_face_kinds[3]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 16, sizeof(int), &hydro_face_kinds[4]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 17, sizeof(int), &hydro_face_kinds[5]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 18, sizeof(OpenClFaceData), hydro_face_data[0].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 19, sizeof(OpenClFaceData), hydro_face_data[1].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 20, sizeof(OpenClFaceData), hydro_face_data[2].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 21, sizeof(OpenClFaceData), hydro_face_data[3].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 22, sizeof(OpenClFaceData), hydro_face_data[4].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 23, sizeof(OpenClFaceData), hydro_face_data[5].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 24, sizeof(int), &benchmark_preset);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_bench, 25, sizeof(cl_mem), &write_buf);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_stream_collide_hydro_bench)", err);
        err = enqueue_kernel_1d(g_opencl.k_stream_collide_hydro_bench, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_stream_collide_hydro_bench)", err);
    } else {
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 0, sizeof(cl_mem), &read_buf);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 1, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 2, sizeof(cl_mem), &temp_read);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 3, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 4, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 5, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 6, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 7, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 8, sizeof(int), &tick_i32);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 9, sizeof(int), &benchmark_flags);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 10, sizeof(int), &hydro_periodic_mask);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 11, sizeof(float), &tau_pair[0]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 12, sizeof(float), &tau_pair[1]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 13, sizeof(float), &base_nu_shear);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 14, sizeof(float), &thermal_transport[2]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 15, sizeof(float), &state_nudge);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 16, sizeof(int), &hydro_face_kinds[0]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 17, sizeof(int), &hydro_face_kinds[1]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 18, sizeof(int), &hydro_face_kinds[2]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 19, sizeof(int), &hydro_face_kinds[3]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 20, sizeof(int), &hydro_face_kinds[4]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 21, sizeof(int), &hydro_face_kinds[5]);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 22, sizeof(OpenClFaceData), hydro_face_data[0].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 23, sizeof(OpenClFaceData), hydro_face_data[1].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 24, sizeof(OpenClFaceData), hydro_face_data[2].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 25, sizeof(OpenClFaceData), hydro_face_data[3].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 26, sizeof(OpenClFaceData), hydro_face_data[4].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 27, sizeof(OpenClFaceData), hydro_face_data[5].data());
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 28, sizeof(int), &benchmark_preset);
        err |= clSetKernelArg(g_opencl.k_stream_collide_hydro_forced, 29, sizeof(cl_mem), &write_buf);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_stream_collide_hydro_forced)", err);
        err = enqueue_kernel_1d(g_opencl.k_stream_collide_hydro_forced, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_stream_collide_hydro_forced)", err);
    }
    if (!stage_fence("after_stream_collide")) return false;

    if (out) {
        err |= clSetKernelArg(g_opencl.k_output, 0, sizeof(cl_mem), &write_buf);
        err |= clSetKernelArg(g_opencl.k_output, 1, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_output, 2, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_output, 3, sizeof(int), &g_cfg.output_channels);
        err |= clSetKernelArg(g_opencl.k_output, 4, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_output, 5, sizeof(cl_mem), &ctx.d_output);
        if (err != CL_SUCCESS) return fail_cl("clSetKernelArg(k_output)", err);
        err = enqueue_kernel_1d(g_opencl.k_output, cells_i32);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueNDRangeKernel(k_output)", err);
        if (!stage_fence("after_output")) return false;
    }
    timing.solver_ms += elapsed_ms(solver_begin, Clock::now());

    auto readback_begin = Clock::now();
    if (out) {
        err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_output, CL_TRUE, 0, output_bytes, out, 0, nullptr, nullptr);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueReadBuffer(d_output)", err);
    }
    if (benchmark_mode_active() && g_benchmark_cfg.preset == AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D) {
        const std::size_t dist_bytes = ctx.cells * kQ * sizeof(float);
        if (ctx.f.size() != ctx.cells * kQ) {
            ctx.f.assign(ctx.cells * kQ, 0.0f);
        }
        err = clEnqueueReadBuffer(g_opencl.queue, write_buf, CL_TRUE, 0, dist_bytes, ctx.f.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) return fail_cl("clEnqueueReadBuffer(write_buf)", err);
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
    cl_int err = clEnqueueReadBuffer(g_opencl.queue, current_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(current_temp)", err);
        return false;
    }
    std::copy(ctx.temperature.begin(), ctx.temperature.end(), ctx.temperature_next.begin());
    rebuild_thermal_distributions_from_temperature(ctx);
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
    rebuild_thermal_distributions_from_temperature(ctx);
    const std::size_t temp_bytes = ctx.cells * sizeof(float);
    cl_int err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_temp)", err);
        return false;
    }
    err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp_next, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_temp_next)", err);
        return false;
    }
#if defined(AERO_LBM_OPENCL)
    if (thermal_ddf_benchmark_active()) {
        const std::size_t thermal_bytes = ctx.cells * kThermalQ * sizeof(float);
        err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_thermal_f, CL_TRUE, 0, thermal_bytes, ctx.thermal_f.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_thermal_f)", err);
            return false;
        }
        err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_thermal_f_post, CL_TRUE, 0, thermal_bytes, ctx.thermal_f.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_thermal_f_post)", err);
            return false;
        }
    }
#endif
    return true;
}

bool sync_context_state_from_gpu(ContextState& ctx) {
    if (!ctx.gpu_buffers_ready || !ctx.gpu_initialized) {
        return true;
    }
    if (ctx.f.size() != ctx.cells * kQ) {
        ctx.f.assign(ctx.cells * kQ, 0.0f);
    }
    if (ctx.f_post.size() != ctx.cells * kQ) {
        ctx.f_post.assign(ctx.cells * kQ, 0.0f);
    }
    ensure_context_temperature_storage(ctx);
    cl_int err = clEnqueueReadBuffer(
        g_opencl.queue,
        ctx.d_f,
        CL_TRUE,
        0,
        ctx.f.size() * sizeof(float),
        ctx.f.data(),
        0,
        nullptr,
        nullptr
    );
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_f)", err);
        return false;
    }
    err = clEnqueueReadBuffer(
        g_opencl.queue,
        ctx.d_f_post,
        CL_TRUE,
        0,
        ctx.f_post.size() * sizeof(float),
        ctx.f_post.data(),
        0,
        nullptr,
        nullptr
    );
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_f_post)", err);
        return false;
    }
    const std::size_t temp_bytes = ctx.cells * sizeof(float);
    err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_temp)", err);
        return false;
    }
    err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_temp_next, CL_TRUE, 0, temp_bytes, ctx.temperature_next.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_temp_next)", err);
        return false;
    }
    err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_temp_scratch, CL_TRUE, 0, temp_bytes, ctx.temperature_scratch.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_temp_scratch)", err);
        return false;
    }
    if (thermal_ddf_benchmark_active()) {
        ensure_context_thermal_ddf_storage(ctx);
        const std::size_t thermal_bytes = ctx.cells * kThermalQ * sizeof(float);
        err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_thermal_f, CL_TRUE, 0, thermal_bytes, ctx.thermal_f.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_thermal_f)", err);
            return false;
        }
        err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_thermal_f_post, CL_TRUE, 0, thermal_bytes, ctx.thermal_f_post.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_thermal_f_post)", err);
            return false;
        }
    }
    ctx.cpu_initialized = true;
    return true;
}

bool sync_context_state_to_gpu(ContextState& ctx) {
    if (!ctx.gpu_buffers_ready || !ctx.gpu_initialized) {
        return true;
    }
    cl_int err = clEnqueueWriteBuffer(
        g_opencl.queue,
        ctx.d_f,
        CL_TRUE,
        0,
        ctx.f.size() * sizeof(float),
        ctx.f.data(),
        0,
        nullptr,
        nullptr
    );
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_f)", err);
        return false;
    }
    err = clEnqueueWriteBuffer(
        g_opencl.queue,
        ctx.d_f_post,
        CL_TRUE,
        0,
        ctx.f_post.size() * sizeof(float),
        ctx.f_post.data(),
        0,
        nullptr,
        nullptr
    );
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_f_post)", err);
        return false;
    }
    const std::size_t temp_bytes = ctx.cells * sizeof(float);
    err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp, CL_TRUE, 0, temp_bytes, ctx.temperature.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_temp)", err);
        return false;
    }
    err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp_next, CL_TRUE, 0, temp_bytes, ctx.temperature_next.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_temp_next)", err);
        return false;
    }
    err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_temp_scratch, CL_TRUE, 0, temp_bytes, ctx.temperature_scratch.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_temp_scratch)", err);
        return false;
    }
    if (thermal_ddf_benchmark_active()) {
        const std::size_t thermal_bytes = ctx.cells * kThermalQ * sizeof(float);
        err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_thermal_f, CL_TRUE, 0, thermal_bytes, ctx.thermal_f.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_thermal_f)", err);
            return false;
        }
        err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_thermal_f_post, CL_TRUE, 0, thermal_bytes, ctx.thermal_f_post.data(), 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueWriteBuffer(d_thermal_f_post)", err);
            return false;
        }
    }
    return true;
}

bool copy_buffer_face_slab(
    cl_mem src,
    std::size_t src_base_bytes,
    cl_mem dst,
    std::size_t dst_base_bytes,
    int nx,
    int ny,
    int nz,
    int src_x0,
    int src_y0,
    int src_z0,
    int dst_x0,
    int dst_y0,
    int dst_z0,
    int size_x,
    int size_y,
    int size_z,
    const char* label
) {
    const std::size_t element_bytes = sizeof(float);
    const std::size_t row_pitch = static_cast<std::size_t>(nz) * element_bytes;
    const std::size_t slice_pitch = static_cast<std::size_t>(ny) * nz * element_bytes;
    const std::size_t src_origin[3] = {
        src_base_bytes + static_cast<std::size_t>(src_z0) * element_bytes,
        static_cast<std::size_t>(src_y0),
        static_cast<std::size_t>(src_x0)
    };
    const std::size_t dst_origin[3] = {
        dst_base_bytes + static_cast<std::size_t>(dst_z0) * element_bytes,
        static_cast<std::size_t>(dst_y0),
        static_cast<std::size_t>(dst_x0)
    };
    const std::size_t region[3] = {
        static_cast<std::size_t>(size_z) * element_bytes,
        static_cast<std::size_t>(size_y),
        static_cast<std::size_t>(size_x)
    };
    const cl_int err = clEnqueueCopyBufferRect(
        g_opencl.queue,
        src,
        dst,
        src_origin,
        dst_origin,
        region,
        row_pitch,
        slice_pitch,
        row_pitch,
        slice_pitch,
        0,
        nullptr,
        nullptr
    );
    if (err != CL_SUCCESS) {
        g_opencl.error = format_opencl_api_error(label, err);
        return false;
    }
    return true;
}

bool exchange_context_halo_gpu_layers(
    ContextState& first,
    ContextState& second,
    int offset_x,
    int offset_y,
    int offset_z,
    int halo_layers
) {
    if (!first.gpu_buffers_ready || !second.gpu_buffers_ready
        || !first.gpu_initialized || !second.gpu_initialized) {
        return true;
    }
    if (first.nx != second.nx || first.ny != second.ny || first.nz != second.nz) {
        std::ostringstream oss;
        oss << "native_exchange_halo: GPU context shape mismatch first="
            << first.nx << "x" << first.ny << "x" << first.nz
            << ", second=" << second.nx << "x" << second.ny << "x" << second.nz;
        set_last_native_error(oss.str());
        return false;
    }
    const int nx = first.nx;
    const int ny = first.ny;
    const int nz = first.nz;
    if (nx != ny || ny != nz) {
        std::ostringstream oss;
        oss << "native_exchange_halo: GPU halo exchange requires cubic contexts, got "
            << nx << "x" << ny << "x" << nz;
        set_last_native_error(oss.str());
        return false;
    }
    const int halo = std::max(1, std::min(halo_layers, nx / 2 - 1));
    const int core = nx - halo * 2;
    if (core <= 0) {
        std::ostringstream oss;
        oss << "native_exchange_halo: invalid GPU halo/core layout nx=" << nx
            << " halo=" << halo << " core=" << core;
        set_last_native_error(oss.str());
        return false;
    }

    int neg_src_x0 = 0;
    int neg_src_y0 = 0;
    int neg_src_z0 = 0;
    int pos_dst_x0 = 0;
    int pos_dst_y0 = 0;
    int pos_dst_z0 = 0;
    int pos_src_x0 = 0;
    int pos_src_y0 = 0;
    int pos_src_z0 = 0;
    int neg_dst_x0 = 0;
    int neg_dst_y0 = 0;
    int neg_dst_z0 = 0;
    int size_x = 0;
    int size_y = 0;
    int size_z = 0;
    if (!halo_exchange_slab_bounds(
        offset_x,
        offset_y,
        offset_z,
        nx,
        ny,
        nz,
        halo,
        core,
        &neg_src_x0,
        &neg_src_y0,
        &neg_src_z0,
        &pos_dst_x0,
        &pos_dst_y0,
        &pos_dst_z0,
        &pos_src_x0,
        &pos_src_y0,
        &pos_src_z0,
        &neg_dst_x0,
        &neg_dst_y0,
        &neg_dst_z0,
        &size_x,
        &size_y,
        &size_z
    )) {
        std::ostringstream oss;
        oss << "native_exchange_halo: halo slab bounds invalid for GPU offset ("
            << offset_x << "," << offset_y << "," << offset_z << ")"
            << " grid=" << nx << " halo=" << halo << " core=" << core;
        set_last_native_error(oss.str());
        return false;
    }

    const std::size_t plane_bytes = first.cells * sizeof(float);
    cl_mem first_current = (first.step_counter % 2 == 0) ? first.d_f : first.d_f_post;
    cl_mem second_current = (second.step_counter % 2 == 0) ? second.d_f : second.d_f_post;
    for (int q = 0; q < kQ; ++q) {
        const std::size_t base = static_cast<std::size_t>(q) * plane_bytes;
        if (!copy_buffer_face_slab(
            first_current, base,
            second_current, base,
            nx, ny, nz,
            neg_src_x0, neg_src_y0, neg_src_z0,
            pos_dst_x0, pos_dst_y0, pos_dst_z0,
            size_x, size_y, size_z,
            "clEnqueueCopyBufferRect(halo_f_first_to_second)"
        )) {
            return false;
        }
        if (!copy_buffer_face_slab(
            second_current, base,
            first_current, base,
            nx, ny, nz,
            pos_src_x0, pos_src_y0, pos_src_z0,
            neg_dst_x0, neg_dst_y0, neg_dst_z0,
            size_x, size_y, size_z,
            "clEnqueueCopyBufferRect(halo_f_second_to_first)"
        )) {
            return false;
        }
    }

    if (should_exchange_temperature_halo(first, second)) {
        cl_mem first_temp_current = (first.step_counter % 2 == 0) ? first.d_temp : first.d_temp_next;
        cl_mem second_temp_current = (second.step_counter % 2 == 0) ? second.d_temp : second.d_temp_next;
        if (!copy_buffer_face_slab(
            first_temp_current,
            0,
            second_temp_current,
            0,
            nx,
            ny,
            nz,
            neg_src_x0,
            neg_src_y0,
            neg_src_z0,
            pos_dst_x0,
            pos_dst_y0,
            pos_dst_z0,
            size_x,
            size_y,
            size_z,
            "clEnqueueCopyBufferRect(halo_temp_first_to_second)"
        )) {
            return false;
        }
        if (!copy_buffer_face_slab(
            second_temp_current,
            0,
            first_temp_current,
            0,
            nx,
            ny,
            nz,
            pos_src_x0,
            pos_src_y0,
            pos_src_z0,
            neg_dst_x0,
            neg_dst_y0,
            neg_dst_z0,
            size_x,
            size_y,
            size_z,
            "clEnqueueCopyBufferRect(halo_temp_second_to_first)"
        )) {
            return false;
        }
    }
    return true;
}

bool exchange_context_halo_gpu(ContextState& first, ContextState& second, int offset_x, int offset_y, int offset_z) {
    return exchange_context_halo_gpu_layers(first, second, offset_x, offset_y, offset_z, std::max(1, first.nx / 4));
}

#else
struct OpenClRuntime { bool available = false; std::string error; std::string device_name; };
OpenClRuntime g_opencl;
void release_context_gpu_buffers(ContextState&) {}
void release_opencl_runtime() {}
bool initialize_opencl_runtime() { g_opencl.error = "Disabled"; return false; }
bool opencl_step(ContextState&, const float*, const int*, const float*, int, float*, StepTiming&) { return false; }
bool sync_context_temperature_from_gpu(ContextState&) { return true; }
bool sync_context_temperature_to_gpu(ContextState&) { return true; }
bool sync_context_state_from_gpu(ContextState&) { return true; }
bool sync_context_state_to_gpu(ContextState&) { return true; }
bool exchange_context_halo_gpu_layers(ContextState&, ContextState&, int, int, int, int) { return true; }
bool exchange_context_halo_gpu(ContextState&, ContextState&, int, int, int) { return true; }
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

struct LockedContext {
    ContextState* ctx = nullptr;

    LockedContext(jlong context_key, bool create_if_missing) {
        g_contexts_mutex.lock();
        if (create_if_missing) {
            ctx = &g_contexts[context_key];
        } else {
            auto it = g_contexts.find(context_key);
            if (it != g_contexts.end()) {
                ctx = &it->second;
            }
        }
        if (ctx) {
            ctx->mutex->lock();
        }
        g_contexts_mutex.unlock();
    }

    ~LockedContext() {
        if (ctx) {
            ctx->mutex->unlock();
        }
    }
};

struct LockedContextPair {
    ContextState* first = nullptr;
    ContextState* second = nullptr;

    LockedContextPair(jlong first_key, jlong second_key) {
        g_contexts_mutex.lock();
        auto first_it = g_contexts.find(first_key);
        auto second_it = g_contexts.find(second_key);
        if (first_it == g_contexts.end() || second_it == g_contexts.end()) {
            g_contexts_mutex.unlock();
            return;
        }
        first = &first_it->second;
        second = &second_it->second;
        if (first_key == second_key) {
            first->mutex->lock();
        } else if (first_key < second_key) {
            first->mutex->lock();
            second->mutex->lock();
        } else {
            second->mutex->lock();
            first->mutex->lock();
        }
        g_contexts_mutex.unlock();
    }

    ~LockedContextPair() {
        if (!first) {
            return;
        }
        if (second && second != first) {
            second->mutex->unlock();
        }
        first->mutex->unlock();
    }
};

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
    if (ctx.temperature_scratch.size() != ctx.cells) {
        ctx.temperature_scratch.assign(ctx.cells, 0.0f);
    }
}

void assign_temperature_state(ContextState& ctx, const float* temperature_state) {
    ensure_context_temperature_storage(ctx);
    for (std::size_t i = 0; i < ctx.cells; ++i) {
        ctx.temperature[i] = clampf(temperature_state[i], kThermalMin, kThermalMax);
    }
    std::copy(ctx.temperature.begin(), ctx.temperature.end(), ctx.temperature_next.begin());
    rebuild_thermal_distributions_from_temperature(ctx);
}

void apply_runtime_temperature_reference(ContextState& ctx) {
    if (g_cfg.input_channels <= kChannelStateTemp || ctx.ref_temperature.size() != ctx.cells) {
        return;
    }
    ensure_context_temperature_storage(ctx);
    for (std::size_t i = 0; i < ctx.cells; ++i) {
        if (ctx.obstacle[i]) {
            ctx.temperature[i] = 0.0f;
            ctx.temperature_next[i] = 0.0f;
            continue;
        }
        int x = 0;
        int y = 0;
        int z = 0;
        decode_cell(i, ctx.nx, ctx.ny, ctx.nz, x, y, z);
        const float temperature_nudge = effective_runtime_state_nudge(ctx.nx, ctx.ny, ctx.nz, x, y, z);
        if (temperature_nudge <= 0.0f) {
            continue;
        }
        const float ref_temperature = clampf(ctx.ref_temperature[i], kThermalMin, kThermalMax);
        const float keep = 1.0f - temperature_nudge;
        ctx.temperature[i] = clampf(ctx.temperature[i] * keep + ref_temperature * temperature_nudge, kThermalMin, kThermalMax);
        ctx.temperature_next[i] = ctx.temperature[i];
    }
    if (thermal_ddf_benchmark_active()) {
        rebuild_thermal_distributions_from_temperature(ctx);
    }
}

template <typename T>
void shift_scalar_field(std::vector<T>& field, int nx, int ny, int nz, int dx, int dy, int dz, T fill_value) {
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    std::vector<T> shifted(cells, fill_value);
    if (field.size() != cells) {
        field = std::move(shifted);
        return;
    }
    for (int x = 0; x < nx; ++x) {
        const int old_x = x + dx;
        if (old_x < 0 || old_x >= nx) continue;
        for (int y = 0; y < ny; ++y) {
            const int old_y = y + dy;
            if (old_y < 0 || old_y >= ny) continue;
            for (int z = 0; z < nz; ++z) {
                const int old_z = z + dz;
                if (old_z < 0 || old_z >= nz) continue;
                shifted[cell_index(x, y, z, nx, ny, nz)] = field[cell_index(old_x, old_y, old_z, nx, ny, nz)];
            }
        }
    }
    field = std::move(shifted);
}

template <std::size_t Q>
void shift_population_field(
    std::vector<float>& field,
    int nx,
    int ny,
    int nz,
    int dx,
    int dy,
    int dz,
    const std::array<float, Q>& fringe_equilibrium
) {
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    std::vector<float> shifted(cells * Q, 0.0f);
    if (field.size() != cells * Q) {
        for (std::size_t cell = 0; cell < cells; ++cell) {
            for (std::size_t q = 0; q < Q; ++q) {
                shifted[q * cells + cell] = fringe_equilibrium[q];
            }
        }
        field = std::move(shifted);
        return;
    }
    for (int x = 0; x < nx; ++x) {
        const int old_x = x + dx;
        for (int y = 0; y < ny; ++y) {
            const int old_y = y + dy;
            for (int z = 0; z < nz; ++z) {
                const int old_z = z + dz;
                const std::size_t new_cell = cell_index(x, y, z, nx, ny, nz);
                if (old_x >= 0 && old_x < nx && old_y >= 0 && old_y < ny && old_z >= 0 && old_z < nz) {
                    const std::size_t old_cell = cell_index(old_x, old_y, old_z, nx, ny, nz);
                    for (std::size_t q = 0; q < Q; ++q) {
                        shifted[q * cells + new_cell] = field[q * cells + old_cell];
                    }
                } else {
                    for (std::size_t q = 0; q < Q; ++q) {
                        shifted[q * cells + new_cell] = fringe_equilibrium[q];
                    }
                }
            }
        }
    }
    field = std::move(shifted);
}

template <typename T>
void copy_scalar_slab(
    const std::vector<T>& src,
    std::vector<T>& dst,
    int nx,
    int ny,
    int nz,
    int src_x0,
    int src_y0,
    int src_z0,
    int dst_x0,
    int dst_y0,
    int dst_z0,
    int size_x,
    int size_y,
    int size_z
) {
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    if (src.size() != cells || dst.size() != cells) {
        return;
    }
    for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
            for (int z = 0; z < size_z; ++z) {
                const std::size_t src_cell = cell_index(src_x0 + x, src_y0 + y, src_z0 + z, nx, ny, nz);
                const std::size_t dst_cell = cell_index(dst_x0 + x, dst_y0 + y, dst_z0 + z, nx, ny, nz);
                dst[dst_cell] = src[src_cell];
            }
        }
    }
}

template <std::size_t Q>
void copy_population_slab(
    const std::vector<float>& src,
    std::vector<float>& dst,
    int nx,
    int ny,
    int nz,
    int src_x0,
    int src_y0,
    int src_z0,
    int dst_x0,
    int dst_y0,
    int dst_z0,
    int size_x,
    int size_y,
    int size_z
) {
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    if (src.size() != cells * Q || dst.size() != cells * Q) {
        return;
    }
    for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
            for (int z = 0; z < size_z; ++z) {
                const std::size_t src_cell = cell_index(src_x0 + x, src_y0 + y, src_z0 + z, nx, ny, nz);
                const std::size_t dst_cell = cell_index(dst_x0 + x, dst_y0 + y, dst_z0 + z, nx, ny, nz);
                for (std::size_t q = 0; q < Q; ++q) {
                    dst[q * cells + dst_cell] = src[q * cells + src_cell];
                }
            }
        }
    }
}

bool halo_exchange_slab_bounds(
    int offset_x,
    int offset_y,
    int offset_z,
    int nx,
    int ny,
    int nz,
    int halo,
    int core,
    int* neg_src_x0,
    int* neg_src_y0,
    int* neg_src_z0,
    int* pos_dst_x0,
    int* pos_dst_y0,
    int* pos_dst_z0,
    int* pos_src_x0,
    int* pos_src_y0,
    int* pos_src_z0,
    int* neg_dst_x0,
    int* neg_dst_y0,
    int* neg_dst_z0,
    int* size_x,
    int* size_y,
    int* size_z
) {
    if (!neg_src_x0 || !neg_src_y0 || !neg_src_z0
        || !pos_dst_x0 || !pos_dst_y0 || !pos_dst_z0
        || !pos_src_x0 || !pos_src_y0 || !pos_src_z0
        || !neg_dst_x0 || !neg_dst_y0 || !neg_dst_z0
        || !size_x || !size_y || !size_z) {
        return false;
    }
    auto fill_axis = [&](int offset, int extent, int* first_src, int* second_dst, int* second_src, int* first_dst, int* axis_size) {
        switch (offset) {
            case -1:
                *first_src = halo;
                *second_dst = extent - halo;
                *second_src = core;
                *first_dst = 0;
                *axis_size = halo;
                return true;
            case 0:
                *first_src = halo;
                *second_dst = halo;
                *second_src = halo;
                *first_dst = halo;
                *axis_size = core;
                return true;
            case 1:
                *first_src = core;
                *second_dst = 0;
                *second_src = halo;
                *first_dst = extent - halo;
                *axis_size = halo;
                return true;
            default:
                return false;
        }
    };

    return fill_axis(offset_x, nx, neg_src_x0, pos_dst_x0, pos_src_x0, neg_dst_x0, size_x)
        && fill_axis(offset_y, ny, neg_src_y0, pos_dst_y0, pos_src_y0, neg_dst_y0, size_y)
        && fill_axis(offset_z, nz, neg_src_z0, pos_dst_z0, pos_src_z0, neg_dst_z0, size_z);
}

bool exchange_context_halo_cpu_layers(
    ContextState& first,
    ContextState& second,
    int offset_x,
    int offset_y,
    int offset_z,
    int halo_layers
) {
    if (first.cells == 0 || second.cells == 0) return true;
    if (first.nx != second.nx || first.ny != second.ny || first.nz != second.nz) {
        std::ostringstream oss;
        oss << "native_exchange_halo: CPU context shape mismatch first="
            << first.nx << "x" << first.ny << "x" << first.nz
            << ", second=" << second.nx << "x" << second.ny << "x" << second.nz;
        set_last_native_error(oss.str());
        return false;
    }
    const int nx = first.nx;
    const int ny = first.ny;
    const int nz = first.nz;
    if (nx != ny || ny != nz) {
        std::ostringstream oss;
        oss << "native_exchange_halo: CPU halo exchange requires cubic contexts, got "
            << nx << "x" << ny << "x" << nz;
        set_last_native_error(oss.str());
        return false;
    }
    const int halo = std::max(1, std::min(halo_layers, nx / 2 - 1));
    const int core = nx - halo * 2;
    if (core <= 0) {
        std::ostringstream oss;
        oss << "native_exchange_halo: invalid CPU halo/core layout nx=" << nx
            << " halo=" << halo << " core=" << core;
        set_last_native_error(oss.str());
        return false;
    }

    int neg_src_x0 = 0;
    int neg_src_y0 = 0;
    int neg_src_z0 = 0;
    int pos_dst_x0 = 0;
    int pos_dst_y0 = 0;
    int pos_dst_z0 = 0;
    int pos_src_x0 = 0;
    int pos_src_y0 = 0;
    int pos_src_z0 = 0;
    int neg_dst_x0 = 0;
    int neg_dst_y0 = 0;
    int neg_dst_z0 = 0;
    int size_x = 0;
    int size_y = 0;
    int size_z = 0;
    if (!halo_exchange_slab_bounds(
        offset_x,
        offset_y,
        offset_z,
        nx,
        ny,
        nz,
        halo,
        core,
        &neg_src_x0,
        &neg_src_y0,
        &neg_src_z0,
        &pos_dst_x0,
        &pos_dst_y0,
        &pos_dst_z0,
        &pos_src_x0,
        &pos_src_y0,
        &pos_src_z0,
        &neg_dst_x0,
        &neg_dst_y0,
        &neg_dst_z0,
        &size_x,
        &size_y,
        &size_z
    )) {
        std::ostringstream oss;
        oss << "native_exchange_halo: halo slab bounds invalid for CPU offset ("
            << offset_x << "," << offset_y << "," << offset_z << ")"
            << " grid=" << nx << " halo=" << halo << " core=" << core;
        set_last_native_error(oss.str());
        return false;
    }

    copy_population_slab<kQ>(
        first.f,
        second.f,
        nx,
        ny,
        nz,
        neg_src_x0,
        neg_src_y0,
        neg_src_z0,
        pos_dst_x0,
        pos_dst_y0,
        pos_dst_z0,
        size_x,
        size_y,
        size_z
    );
    copy_population_slab<kQ>(
        second.f,
        first.f,
        nx,
        ny,
        nz,
        pos_src_x0,
        pos_src_y0,
        pos_src_z0,
        neg_dst_x0,
        neg_dst_y0,
        neg_dst_z0,
        size_x,
        size_y,
        size_z
    );
    if (should_exchange_temperature_halo(first, second)
        && first.temperature.size() == first.cells
        && second.temperature.size() == second.cells) {
        copy_scalar_slab(
            first.temperature,
            second.temperature,
            nx,
            ny,
            nz,
            neg_src_x0,
            neg_src_y0,
            neg_src_z0,
            pos_dst_x0,
            pos_dst_y0,
            pos_dst_z0,
            size_x,
            size_y,
            size_z
        );
        copy_scalar_slab(
            second.temperature,
            first.temperature,
            nx,
            ny,
            nz,
            pos_src_x0,
            pos_src_y0,
            pos_src_z0,
            neg_dst_x0,
            neg_dst_y0,
            neg_dst_z0,
            size_x,
            size_y,
            size_z
        );
        if (first.temperature_next.size() == first.cells) {
            std::copy(first.temperature.begin(), first.temperature.end(), first.temperature_next.begin());
        }
        if (second.temperature_next.size() == second.cells) {
            std::copy(second.temperature.begin(), second.temperature.end(), second.temperature_next.begin());
        }
    }
    return true;
}

bool exchange_context_halo_cpu(ContextState& first, ContextState& second, int offset_x, int offset_y, int offset_z) {
    return exchange_context_halo_cpu_layers(first, second, offset_x, offset_y, offset_z, std::max(1, first.nx / 4));
}

bool shift_context_cpu_state(ContextState& ctx, int dx, int dy, int dz) {
    if (ctx.cells == 0) return true;
    if (dx == 0 && dy == 0 && dz == 0) return true;
    if (std::abs(dx) >= ctx.nx || std::abs(dy) >= ctx.ny || std::abs(dz) >= ctx.nz) return false;

    std::array<float, kQ> hydro_eq{};
    for (int q = 0; q < kQ; ++q) {
        hydro_eq[q] = feq(q, 1.0f, 0.0f, 0.0f, 0.0f);
    }
    shift_population_field(ctx.f, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, hydro_eq);
    shift_population_field(ctx.f_post, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, hydro_eq);

    if (ctx.rho.size() == ctx.cells) shift_scalar_field(ctx.rho, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 1.0f);
    if (ctx.ux.size() == ctx.cells) shift_scalar_field(ctx.ux, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.uy.size() == ctx.cells) shift_scalar_field(ctx.uy, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.uz.size() == ctx.cells) shift_scalar_field(ctx.uz, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);

    if (ctx.ref_ux.size() == ctx.cells) shift_scalar_field(ctx.ref_ux, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.ref_uy.size() == ctx.cells) shift_scalar_field(ctx.ref_uy, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.ref_uz.size() == ctx.cells) shift_scalar_field(ctx.ref_uz, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.ref_pressure.size() == ctx.cells) shift_scalar_field(ctx.ref_pressure, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.ref_temperature.size() == ctx.cells) shift_scalar_field(ctx.ref_temperature, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);

    if (ctx.fan_mask.size() == ctx.cells) shift_scalar_field(ctx.fan_mask, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.fan_ux.size() == ctx.cells) shift_scalar_field(ctx.fan_ux, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.fan_uy.size() == ctx.cells) shift_scalar_field(ctx.fan_uy, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.fan_uz.size() == ctx.cells) shift_scalar_field(ctx.fan_uz, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.thermal_source.size() == ctx.cells) shift_scalar_field(ctx.thermal_source, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.obstacle.size() == ctx.cells) shift_scalar_field<uint8_t>(ctx.obstacle, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, static_cast<uint8_t>(0));
    if (ctx.temperature.size() == ctx.cells) shift_scalar_field(ctx.temperature, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.temperature_next.size() == ctx.cells) shift_scalar_field(ctx.temperature_next, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);
    if (ctx.temperature_scratch.size() == ctx.cells) shift_scalar_field(ctx.temperature_scratch, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, 0.0f);

    if (ctx.thermal_f.size() == ctx.cells * kThermalQ && ctx.thermal_f_post.size() == ctx.cells * kThermalQ) {
        std::array<float, kThermalQ> thermal_eq{};
        for (int q = 0; q < kThermalQ; ++q) {
            thermal_eq[q] = thermal_feq(q, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        shift_population_field(ctx.thermal_f, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, thermal_eq);
        shift_population_field(ctx.thermal_f_post, ctx.nx, ctx.ny, ctx.nz, dx, dy, dz, thermal_eq);
    }

    ctx.last_force[0] = 0.0f;
    ctx.last_force[1] = 0.0f;
    ctx.last_force[2] = 0.0f;
    return true;
}

void run_cpu_step(
    ContextState& ctx,
    const float* packet,
    const int* overlay_cells,
    const float* overlay_values,
    int overlay_count,
    float* out
) {
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);
    ingest_payload(ctx, packet, g_cfg.input_channels);
    apply_sparse_payload_overlays(ctx, overlay_cells, overlay_values, overlay_count);
    if (!ctx.cpu_initialized) {
        initialize_distributions(ctx);
    } else {
        apply_runtime_temperature_reference(ctx);
    }
    if (should_update_temperature(ctx.step_counter)) {
        update_temperature_field(ctx);
    }
    collide(ctx);
    compute_benchmark_force(ctx);
    stream_and_bounce(ctx);
    if (out) {
        write_output(ctx, out, g_cfg.output_channels);
    }
    ctx.step_counter += 1;
}

bool run_solver_step(
    ContextState& ctx,
    const float* packet,
    const int* overlay_cells,
    const float* overlay_values,
    int overlay_count,
    float* out,
    StepTiming& timing
) {
    bool ok = false;
    if (g_cfg.opencl_enabled && benchmark_opencl_supported()) {
        if (!(ok = opencl_step(ctx, packet, overlay_cells, overlay_values, overlay_count, out, timing))) {
            disable_opencl_runtime(g_opencl.error.empty() ? "OpenCL fail" : g_opencl.error);
        }
    }
    if (!ok) {
        auto solver_begin = Clock::now();
        run_cpu_step(ctx, packet, overlay_cells, overlay_values, overlay_count, out);
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

    clear_last_native_error();
    if (!g_cfg.initialized) {
        set_last_native_error("native_step_raw_dims: runtime not initialized");
        return false;
    }
    if (!packet) {
        set_last_native_error("native_step_raw_dims: missing packet");
        return false;
    }
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        set_last_native_error("native_step_raw_dims: invalid dimensions");
        return false;
    }
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;

    LockedContext locked_context(context_key, true);
    if (!locked_context.ctx) {
        set_last_native_error("native_step_raw_dims: failed to allocate or acquire context");
        return false;
    }
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);

    const bool ok = run_solver_step(ctx, packet, nullptr, nullptr, 0, output_flow, timing);
    if (!ok && g_last_native_error.empty()) {
        set_last_native_error("native_step_raw_dims: run_solver_step failed");
    }
    timing.total_ms = elapsed_ms(tick_begin, Clock::now());
    record_timing(timing);
    return ok;
}

static bool native_step_raw_dims_with_sparse_overlays_impl(
    const float* packet,
    jint nx,
    jint ny,
    jint nz,
    jlong context_key,
    const int* overlay_cells,
    const float* overlay_values,
    jint overlay_count,
    float* output_flow
) {
    auto tick_begin = Clock::now();
    StepTiming timing;

    clear_last_native_error();
    if (!g_cfg.initialized) {
        set_last_native_error("native_step_raw_dims_sparse: runtime not initialized");
        return false;
    }
    if (!packet) {
        set_last_native_error("native_step_raw_dims_sparse: missing packet");
        return false;
    }
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        set_last_native_error("native_step_raw_dims_sparse: invalid dimensions");
        return false;
    }
    if (overlay_count < 0) {
        set_last_native_error("native_step_raw_dims_sparse: invalid overlay count");
        return false;
    }
    if (overlay_count > 0 && (!overlay_cells || !overlay_values)) {
        set_last_native_error("native_step_raw_dims_sparse: missing overlay data");
        return false;
    }
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;

    LockedContext locked_context(context_key, true);
    if (!locked_context.ctx) {
        set_last_native_error("native_step_raw_dims_sparse: failed to allocate or acquire context");
        return false;
    }
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);

    const bool ok = run_solver_step(ctx, packet, overlay_cells, overlay_values, overlay_count, output_flow, timing);
    if (!ok && g_last_native_error.empty()) {
        set_last_native_error("native_step_raw_dims_sparse: run_solver_step failed");
    }
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

    LockedContext locked_context(context_key, true);
    if (!locked_context.ctx) {
        env->ReleaseFloatArrayElements(output_flow, out, 0);
        return JNI_FALSE;
    }
    ContextState& ctx = *locked_context.ctx;
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

    bool ok = run_solver_step(ctx, ctx.packet.data(), nullptr, nullptr, 0, out, timing);

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

    LockedContext locked_context(context_key, true);
    if (!locked_context.ctx) {
        env->ReleaseFloatArrayElements(output_flow, out, 0);
        return JNI_FALSE;
    }
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);

    const float* packet = reinterpret_cast<const float*>(payload_raw);
    bool ok = run_solver_step(ctx, packet, nullptr, nullptr, 0, out, timing);

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
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (g_cfg.opencl_enabled && !sync_context_temperature_from_gpu(ctx)) return false;
    if (ctx.temperature.size() != cells) return false;
    std::memcpy(temperature_out, ctx.temperature.data(), cells * sizeof(float));
    return true;
}

static bool native_sample_temperature_point_raw_dims_impl(
    jint nx,
    jint ny,
    jint nz,
    jlong context_key,
    jint sample_x,
    jint sample_y,
    jint sample_z,
    float* out_temperature
) {
    if (!g_cfg.initialized || !out_temperature) return false;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    if (sample_x < 0 || sample_y < 0 || sample_z < 0 || sample_x >= nx || sample_y >= ny || sample_z >= nz) {
        return false;
    }
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    const std::size_t cell = cell_index(sample_x, sample_y, sample_z, nx, ny, nz);
    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (g_cfg.opencl_enabled && ctx.gpu_buffers_ready && ctx.gpu_initialized) {
        cl_mem current_temp = (ctx.step_counter % 2 == 0) ? ctx.d_temp : ctx.d_temp_next;
        const std::size_t offset = cell * sizeof(float);
        float sampled = 0.0f;
        cl_int err = clEnqueueReadBuffer(g_opencl.queue, current_temp, CL_TRUE, offset, sizeof(float), &sampled, 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(sample_temperature_point)", err);
            return false;
        }
        *out_temperature = sampled;
        return true;
    }
    if (ctx.temperature.size() != cells) return false;
    *out_temperature = ctx.temperature[cell];
    return true;
}

static bool native_sample_flow_point_raw_dims_impl(
    jint nx,
    jint ny,
    jint nz,
    jlong context_key,
    jint sample_x,
    jint sample_y,
    jint sample_z,
    float* out_flow
) {
    if (!g_cfg.initialized || !out_flow) return false;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    if (sample_x < 0 || sample_y < 0 || sample_z < 0 || sample_x >= nx || sample_y >= ny || sample_z >= nz) {
        return false;
    }
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    const std::size_t cell = cell_index(sample_x, sample_y, sample_z, nx, ny, nz);
    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);

    auto write_zero = [&]() {
        out_flow[0] = 0.0f;
        out_flow[1] = 0.0f;
        out_flow[2] = 0.0f;
        out_flow[3] = 0.0f;
    };

#if defined(AERO_LBM_OPENCL)
    if (g_cfg.opencl_enabled && ctx.gpu_buffers_ready && ctx.gpu_initialized) {
        float obstacle = 0.0f;
        const std::size_t obstacle_offset =
            (cell * static_cast<std::size_t>(g_cfg.input_channels) + kChannelObstacle) * sizeof(float);
        cl_int err = clEnqueueReadBuffer(
            g_opencl.queue,
            ctx.d_payload,
            CL_TRUE,
            obstacle_offset,
            sizeof(float),
            &obstacle,
            0,
            nullptr,
            nullptr
        );
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(sample_flow_point:obstacle)", err);
            return false;
        }
        if (obstacle > 0.5f) {
            write_zero();
            return true;
        }

        cl_mem current_flow = (ctx.step_counter % 2 == 0) ? ctx.d_f : ctx.d_f_post;
        float rho = 0.0f;
        float ux = 0.0f;
        float uy = 0.0f;
        float uz = 0.0f;
        for (int q = 0; q < kQ; ++q) {
            float fq = 0.0f;
            const std::size_t offset = dist_index(cell, q, cells) * sizeof(float);
            err = clEnqueueReadBuffer(g_opencl.queue, current_flow, CL_TRUE, offset, sizeof(float), &fq, 0, nullptr, nullptr);
            if (err != CL_SUCCESS) {
                g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(sample_flow_point:f)", err);
                return false;
            }
            if (!std::isfinite(fq)) {
                write_zero();
                return true;
            }
            rho += fq;
            ux += fq * static_cast<float>(kCx[q]);
            uy += fq * static_cast<float>(kCy[q]);
            uz += fq * static_cast<float>(kCz[q]);
        }
        if (!std::isfinite(rho) || !std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
            write_zero();
            return true;
        }
        rho = clampf(rho, kRhoMin, kRhoMax);
        if (!std::isfinite(rho) || rho <= 1e-6f) {
            write_zero();
            return true;
        }
        ux /= rho;
        uy /= rho;
        uz /= rho;
        if (!std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
            ux = 0.0f;
            uy = 0.0f;
            uz = 0.0f;
        }
        if (std::fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;
        if (std::fabs(ux) < 1e-7f) ux = 0.0f;
        if (std::fabs(uy) < 1e-7f) uy = 0.0f;
        if (std::fabs(uz) < 1e-7f) uz = 0.0f;

        out_flow[0] = ux;
        out_flow[1] = uy;
        out_flow[2] = uz;
        out_flow[3] = clampf(rho - 1.0f, kPressureMin, kPressureMax);
        return true;
    }
#endif

    if (ctx.f.empty() || ctx.cells == 0 || ctx.obstacle.size() != cells) return false;
    if (ctx.obstacle[cell]) {
        write_zero();
        return true;
    }

    float rho = 0.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    for (int q = 0; q < kQ; ++q) {
        const float fq = ctx.f[dist_index(cell, q, cells)];
        if (!std::isfinite(fq)) {
            write_zero();
            return true;
        }
        rho += fq;
        ux += fq * static_cast<float>(kCx[q]);
        uy += fq * static_cast<float>(kCy[q]);
        uz += fq * static_cast<float>(kCz[q]);
    }
    if (!std::isfinite(rho) || !std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
        write_zero();
        return true;
    }
    rho = clampf(rho, kRhoMin, kRhoMax);
    if (!std::isfinite(rho) || rho <= 1e-6f) {
        write_zero();
        return true;
    }
    ux /= rho;
    uy /= rho;
    uz /= rho;
    if (!std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
        ux = 0.0f;
        uy = 0.0f;
        uz = 0.0f;
    }
    if (std::fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;
    if (std::fabs(ux) < 1e-7f) ux = 0.0f;
    if (std::fabs(uy) < 1e-7f) uy = 0.0f;
    if (std::fabs(uz) < 1e-7f) uz = 0.0f;

    out_flow[0] = ux;
    out_flow[1] = uy;
    out_flow[2] = uz;
    out_flow[3] = clampf(rho - 1.0f, kPressureMin, kPressureMax);
    return true;
}

static bool native_extract_flow_atlas_raw_dims_impl(
    jint nx,
    jint ny,
    jint nz,
    jlong context_key,
    jint stride,
    float* out_flow_atlas,
    jint value_count
) {
    if (!g_cfg.initialized || !out_flow_atlas || stride <= 0) return false;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    const int sx = (nx + stride - 1) / stride;
    const int sy = (ny + stride - 1) / stride;
    const int sz = (nz + stride - 1) / stride;
    const int atlas_cells = sx * sy * sz;
    if (atlas_cells <= 0 || value_count != atlas_cells * 4) return false;

    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);

#if defined(AERO_LBM_OPENCL)
    if (g_cfg.opencl_enabled && ctx.gpu_buffers_ready && ctx.gpu_initialized) {
        cl_mem current_flow = (ctx.step_counter % 2 == 0) ? ctx.d_f : ctx.d_f_post;
        cl_int err = CL_SUCCESS;
        err |= clSetKernelArg(g_opencl.k_output_strided, 0, sizeof(cl_mem), &current_flow);
        err |= clSetKernelArg(g_opencl.k_output_strided, 1, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_output_strided, 2, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_output_strided, 3, sizeof(int), &ctx.nx);
        err |= clSetKernelArg(g_opencl.k_output_strided, 4, sizeof(int), &ctx.ny);
        err |= clSetKernelArg(g_opencl.k_output_strided, 5, sizeof(int), &ctx.nz);
        err |= clSetKernelArg(g_opencl.k_output_strided, 6, sizeof(int), &stride);
        err |= clSetKernelArg(g_opencl.k_output_strided, 7, sizeof(int), &sx);
        err |= clSetKernelArg(g_opencl.k_output_strided, 8, sizeof(int), &sy);
        err |= clSetKernelArg(g_opencl.k_output_strided, 9, sizeof(int), &sz);
        err |= clSetKernelArg(g_opencl.k_output_strided, 10, sizeof(cl_mem), &ctx.d_output);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clSetKernelArg(k_output_strided)", err);
            return false;
        }
        err = enqueue_kernel_1d(g_opencl.k_output_strided, atlas_cells);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueNDRangeKernel(k_output_strided)", err);
            return false;
        }
        const std::size_t bytes = static_cast<std::size_t>(value_count) * sizeof(float);
        err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_output, CL_TRUE, 0, bytes, out_flow_atlas, 0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            g_opencl.error = format_opencl_api_error("clEnqueueReadBuffer(d_output_atlas)", err);
            return false;
        }
        return true;
    }
#endif

    if (ctx.f.empty() || ctx.cells == 0 || ctx.obstacle.size() != cells) return false;
    int dst = 0;
    for (int ax = 0; ax < sx; ++ax) {
        const int gx = std::min(nx - 1, ax * stride);
        for (int ay = 0; ay < sy; ++ay) {
            const int gy = std::min(ny - 1, ay * stride);
            for (int az = 0; az < sz; ++az) {
                const int gz = std::min(nz - 1, az * stride);
                const std::size_t cell = cell_index(gx, gy, gz, nx, ny, nz);
                if (ctx.obstacle[cell]) {
                    out_flow_atlas[dst + 0] = 0.0f;
                    out_flow_atlas[dst + 1] = 0.0f;
                    out_flow_atlas[dst + 2] = 0.0f;
                    out_flow_atlas[dst + 3] = 0.0f;
                    dst += 4;
                    continue;
                }
                float rho = 0.0f;
                float ux = 0.0f;
                float uy = 0.0f;
                float uz = 0.0f;
                bool valid = true;
                for (int q = 0; q < kQ; ++q) {
                    const float fq = ctx.f[dist_index(cell, q, cells)];
                    if (!std::isfinite(fq)) {
                        valid = false;
                        break;
                    }
                    rho += fq;
                    ux += fq * static_cast<float>(kCx[q]);
                    uy += fq * static_cast<float>(kCy[q]);
                    uz += fq * static_cast<float>(kCz[q]);
                }
                if (!valid || !std::isfinite(rho) || !std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
                    out_flow_atlas[dst + 0] = 0.0f;
                    out_flow_atlas[dst + 1] = 0.0f;
                    out_flow_atlas[dst + 2] = 0.0f;
                    out_flow_atlas[dst + 3] = 0.0f;
                    dst += 4;
                    continue;
                }
                rho = clampf(rho, kRhoMin, kRhoMax);
                if (!std::isfinite(rho) || rho <= 1e-6f) {
                    out_flow_atlas[dst + 0] = 0.0f;
                    out_flow_atlas[dst + 1] = 0.0f;
                    out_flow_atlas[dst + 2] = 0.0f;
                    out_flow_atlas[dst + 3] = 0.0f;
                    dst += 4;
                    continue;
                }
                ux /= rho;
                uy /= rho;
                uz /= rho;
                if (!std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
                    ux = 0.0f;
                    uy = 0.0f;
                    uz = 0.0f;
                }
                if (std::fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;
                if (std::fabs(ux) < 1e-7f) ux = 0.0f;
                if (std::fabs(uy) < 1e-7f) uy = 0.0f;
                if (std::fabs(uz) < 1e-7f) uz = 0.0f;
                out_flow_atlas[dst + 0] = ux;
                out_flow_atlas[dst + 1] = uy;
                out_flow_atlas[dst + 2] = uz;
                out_flow_atlas[dst + 3] = clampf(rho - 1.0f, kPressureMin, kPressureMax);
                dst += 4;
            }
        }
    }
    return true;
}

static bool native_copy_flow_temperature_subrect_raw_dims_impl(
    jint nx,
    jint ny,
    jint nz,
    jlong context_key,
    jint offset_x,
    jint offset_y,
    jint offset_z,
    jint copy_nx,
    jint copy_ny,
    jint copy_nz,
    float* out_flow,
    float* out_temperature
) {
    if (!g_cfg.initialized || !out_flow || !out_temperature) return false;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    if (copy_nx <= 0 || copy_ny <= 0 || copy_nz <= 0) return false;
    if (offset_x < 0 || offset_y < 0 || offset_z < 0) return false;
    if (offset_x + copy_nx > nx || offset_y + copy_ny > ny || offset_z + copy_nz > nz) return false;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    const std::size_t copy_cells = static_cast<std::size_t>(copy_nx) * copy_ny * copy_nz;

    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (g_cfg.opencl_enabled && !sync_context_state_from_gpu(ctx)) return false;
    if (ctx.f.empty() || ctx.cells == 0 || ctx.obstacle.size() != cells || ctx.temperature.size() != cells) {
        return false;
    }

    auto write_zero = [](float* flow) {
        flow[0] = 0.0f;
        flow[1] = 0.0f;
        flow[2] = 0.0f;
        flow[3] = 0.0f;
    };

    std::size_t dst_cell = 0;
    for (int x = 0; x < copy_nx; ++x) {
        const int src_x = offset_x + x;
        for (int y = 0; y < copy_ny; ++y) {
            const int src_y = offset_y + y;
            for (int z = 0; z < copy_nz; ++z, ++dst_cell) {
                const int src_z = offset_z + z;
                const std::size_t src_cell = cell_index(src_x, src_y, src_z, nx, ny, nz);
                float* dst_flow = out_flow + dst_cell * 4;
                out_temperature[dst_cell] = ctx.obstacle[src_cell]
                    ? 0.0f
                    : finite_or(clampf(ctx.temperature[src_cell], kThermalMin, kThermalMax), 0.0f);
                if (ctx.obstacle[src_cell]) {
                    write_zero(dst_flow);
                    continue;
                }

                float rho = 0.0f;
                float ux = 0.0f;
                float uy = 0.0f;
                float uz = 0.0f;
                bool valid = true;
                for (int q = 0; q < kQ; ++q) {
                    const float fq = ctx.f[dist_index(src_cell, q, cells)];
                    if (!std::isfinite(fq)) {
                        valid = false;
                        break;
                    }
                    rho += fq;
                    ux += fq * static_cast<float>(kCx[q]);
                    uy += fq * static_cast<float>(kCy[q]);
                    uz += fq * static_cast<float>(kCz[q]);
                }
                if (!valid || !std::isfinite(rho) || !std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
                    write_zero(dst_flow);
                    continue;
                }
                rho = clampf(rho, kRhoMin, kRhoMax);
                if (!std::isfinite(rho) || rho <= 1e-6f) {
                    write_zero(dst_flow);
                    continue;
                }
                ux /= rho;
                uy /= rho;
                uz /= rho;
                if (!std::isfinite(ux) || !std::isfinite(uy) || !std::isfinite(uz)) {
                    ux = 0.0f;
                    uy = 0.0f;
                    uz = 0.0f;
                }
                if (std::fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;
                if (std::fabs(ux) < 1e-7f) ux = 0.0f;
                if (std::fabs(uy) < 1e-7f) uy = 0.0f;
                if (std::fabs(uz) < 1e-7f) uz = 0.0f;
                dst_flow[0] = ux;
                dst_flow[1] = uy;
                dst_flow[2] = uz;
                dst_flow[3] = clampf(rho - 1.0f, kPressureMin, kPressureMax);
            }
        }
    }
    return true;
}

static bool native_get_flow_state_raw_dims_impl(jint nx, jint ny, jint nz, jlong context_key, float* flow_out) {
    if (!g_cfg.initialized || !flow_out) return false;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;
    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (g_cfg.opencl_enabled && !sync_context_state_from_gpu(ctx)) return false;
    if (ctx.f.empty() || ctx.cells == 0) return false;
    write_output(ctx, flow_out, 4);
    return true;
}

static jboolean native_get_temperature_state_impl(
    JNIEnv* env, jclass, jint grid_size, jlong context_key, jfloatArray temperature_state
) {
    if (!g_cfg.initialized || !temperature_state || grid_size != g_cfg.grid_size) return JNI_FALSE;
    const std::size_t cells = configured_cells();
    if (env->GetArrayLength(temperature_state) != static_cast<jsize>(cells)) return JNI_FALSE;

    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return JNI_FALSE;
    ContextState& ctx = *locked_context.ctx;
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

    LockedContext locked_context(context_key, true);
    if (!locked_context.ctx) return JNI_FALSE;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);
    jfloat* temperature_ptr = env->GetFloatArrayElements(temperature_state, nullptr);
    if (!temperature_ptr) return JNI_FALSE;
    assign_temperature_state(ctx, temperature_ptr);
    env->ReleaseFloatArrayElements(temperature_state, temperature_ptr, JNI_ABORT);
    if (g_cfg.opencl_enabled && !sync_context_temperature_to_gpu(ctx)) return JNI_FALSE;
    return JNI_TRUE;
}

static bool native_set_temperature_state_raw_dims_impl(
    jint nx,
    jint ny,
    jint nz,
    jlong context_key,
    const float* temperature_state
) {
    if (!g_cfg.initialized || !temperature_state) return false;
    if (nx <= 0 || ny <= 0 || nz <= 0) return false;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;

    LockedContext locked_context(context_key, true);
    if (!locked_context.ctx) return false;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.nx, ctx.ny, ctx.nz);
    assign_temperature_state(ctx, temperature_state);
    if (g_cfg.opencl_enabled && !sync_context_temperature_to_gpu(ctx)) return false;
    return true;
}

static jboolean native_shift_context_impl(jint grid_size, jlong context_key, jint dx, jint dy, jint dz) {
    if (!g_cfg.initialized || grid_size != g_cfg.grid_size) return JNI_FALSE;
    LockedContext locked_context(context_key, false);
    if (!locked_context.ctx) return JNI_FALSE;
    ContextState& ctx = *locked_context.ctx;
    const std::size_t cells = configured_cells();
    ensure_context_shape(ctx, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (ctx.cells == 0) return JNI_TRUE;
    if (std::abs(dx) >= ctx.nx || std::abs(dy) >= ctx.ny || std::abs(dz) >= ctx.nz) return JNI_FALSE;
    if (g_cfg.opencl_enabled && !sync_context_state_from_gpu(ctx)) return JNI_FALSE;
    if (!shift_context_cpu_state(ctx, dx, dy, dz)) return JNI_FALSE;
    if (g_cfg.opencl_enabled && !sync_context_state_to_gpu(ctx)) return JNI_FALSE;
    return JNI_TRUE;
}

static jboolean native_has_context_impl(jlong context_key) {
    std::lock_guard<SpinMutex> lock(g_contexts_mutex);
    return g_contexts.find(context_key) != g_contexts.end() ? JNI_TRUE : JNI_FALSE;
}

static jboolean native_exchange_halo_impl(
    jint grid_size,
    jlong first_context_key,
    jlong second_context_key,
    jint offset_x,
    jint offset_y,
    jint offset_z
) {
    clear_last_native_error();
    if (!g_cfg.initialized) {
        set_last_native_error("native_exchange_halo: runtime not initialized");
        return JNI_FALSE;
    }
    if (grid_size != g_cfg.grid_size) {
        std::ostringstream oss;
        oss << "native_exchange_halo: grid size mismatch (requested=" << grid_size
            << ", configured=" << g_cfg.grid_size << ")";
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }
    if (offset_x < -1 || offset_x > 1 || offset_y < -1 || offset_y > 1 || offset_z < -1 || offset_z > 1) {
        std::ostringstream oss;
        oss << "native_exchange_halo: invalid offset (" << offset_x << "," << offset_y << "," << offset_z << ")";
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }
    if (offset_x == 0 && offset_y == 0 && offset_z == 0) {
        set_last_native_error("native_exchange_halo: zero offset is invalid");
        return JNI_FALSE;
    }
    LockedContextPair locked_contexts(first_context_key, second_context_key);
    if (!locked_contexts.first || !locked_contexts.second) {
        std::lock_guard<SpinMutex> lock(g_contexts_mutex);
        auto first_it = g_contexts.find(first_context_key);
        auto second_it = g_contexts.find(second_context_key);
        std::ostringstream oss;
        oss << "native_exchange_halo: missing context(s) first=" << static_cast<long long>(first_context_key)
            << (first_it == g_contexts.end() ? " [missing]" : "")
            << ", second=" << static_cast<long long>(second_context_key)
            << (second_it == g_contexts.end() ? " [missing]" : "");
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }
    ContextState& first = *locked_contexts.first;
    ContextState& second = *locked_contexts.second;
    const std::size_t cells = configured_cells();
    ensure_context_shape(first, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    ensure_context_shape(second, g_cfg.nx, g_cfg.ny, g_cfg.nz, cells);
    if (first.cells == 0 || second.cells == 0) {
        clear_last_native_error();
        return JNI_TRUE;
    }
    if (g_cfg.opencl_enabled) {
        const bool ok = exchange_context_halo_gpu(first, second, offset_x, offset_y, offset_z);
        if (!ok && g_last_native_error.empty()) {
            set_last_native_error(g_opencl.error.empty() ? "native_exchange_halo: OpenCL halo exchange failed"
                                                         : g_opencl.error);
        }
        return ok ? JNI_TRUE : JNI_FALSE;
    }
    const bool ok = exchange_context_halo_cpu(first, second, offset_x, offset_y, offset_z);
    if (!ok && g_last_native_error.empty()) {
        std::ostringstream oss;
        oss << "native_exchange_halo: CPU halo exchange failed for offset ("
            << offset_x << "," << offset_y << "," << offset_z << ")";
        set_last_native_error(oss.str());
    }
    return ok ? JNI_TRUE : JNI_FALSE;
}

static jboolean native_exchange_halo_rect_layers_impl(
    jint nx,
    jint ny,
    jint nz,
    jint halo_layers,
    jlong first_context_key,
    jlong second_context_key,
    jint offset_x,
    jint offset_y,
    jint offset_z
) {
    clear_last_native_error();
    if (!g_cfg.initialized) {
        set_last_native_error("native_exchange_halo_rect_layers: runtime not initialized");
        return JNI_FALSE;
    }
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        set_last_native_error("native_exchange_halo_rect_layers: invalid dimensions");
        return JNI_FALSE;
    }
    if (halo_layers <= 0) {
        set_last_native_error("native_exchange_halo_rect_layers: halo_layers must be positive");
        return JNI_FALSE;
    }
    if (offset_x < -1 || offset_x > 1 || offset_y < -1 || offset_y > 1 || offset_z < -1 || offset_z > 1) {
        std::ostringstream oss;
        oss << "native_exchange_halo_rect_layers: invalid offset ("
            << offset_x << "," << offset_y << "," << offset_z << ")";
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }
    if (offset_x == 0 && offset_y == 0 && offset_z == 0) {
        set_last_native_error("native_exchange_halo_rect_layers: zero offset is invalid");
        return JNI_FALSE;
    }
    if (nx != ny || ny != nz) {
        std::ostringstream oss;
        oss << "native_exchange_halo_rect_layers: only cubic contexts are supported, got "
            << nx << "x" << ny << "x" << nz;
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }
    if (halo_layers * 2 >= nx) {
        std::ostringstream oss;
        oss << "native_exchange_halo_rect_layers: invalid halo layout nx=" << nx
            << " halo=" << halo_layers;
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }

    LockedContextPair locked_contexts(first_context_key, second_context_key);
    if (!locked_contexts.first || !locked_contexts.second) {
        std::lock_guard<SpinMutex> lock(g_contexts_mutex);
        auto first_it = g_contexts.find(first_context_key);
        auto second_it = g_contexts.find(second_context_key);
        std::ostringstream oss;
        oss << "native_exchange_halo_rect_layers: missing context(s) first="
            << static_cast<long long>(first_context_key)
            << (first_it == g_contexts.end() ? " [missing]" : "")
            << ", second=" << static_cast<long long>(second_context_key)
            << (second_it == g_contexts.end() ? " [missing]" : "");
        set_last_native_error(oss.str());
        return JNI_FALSE;
    }
    ContextState& first = *locked_contexts.first;
    ContextState& second = *locked_contexts.second;
    const long long cells_ll = static_cast<long long>(nx) * static_cast<long long>(ny) * static_cast<long long>(nz);
    if (cells_ll <= 0 || cells_ll > static_cast<long long>(std::numeric_limits<int>::max())) {
        set_last_native_error("native_exchange_halo_rect_layers: invalid cell count");
        return JNI_FALSE;
    }
    const std::size_t cells = static_cast<std::size_t>(cells_ll);
    ensure_context_shape(first, nx, ny, nz, cells);
    ensure_context_shape(second, nx, ny, nz, cells);
    if (first.cells == 0 || second.cells == 0) {
        clear_last_native_error();
        return JNI_TRUE;
    }
    bool ok = false;
    if (g_cfg.opencl_enabled) {
        ok = exchange_context_halo_gpu_layers(first, second, offset_x, offset_y, offset_z, halo_layers);
        if (!ok && g_last_native_error.empty()) {
            set_last_native_error(g_opencl.error.empty() ? "native_exchange_halo_rect_layers: OpenCL halo exchange failed"
                                                         : g_opencl.error);
        }
    } else {
        ok = exchange_context_halo_cpu_layers(first, second, offset_x, offset_y, offset_z, halo_layers);
        if (!ok && g_last_native_error.empty()) {
            std::ostringstream oss;
            oss << "native_exchange_halo_rect_layers: CPU halo exchange failed for offset ("
                << offset_x << "," << offset_y << "," << offset_z << ")";
            set_last_native_error(oss.str());
        }
    }
    return ok ? JNI_TRUE : JNI_FALSE;
}

static void native_release_context_impl(jlong context_key) {
    std::lock_guard<SpinMutex> lock(g_contexts_mutex);
    auto it = g_contexts.find(context_key);
    if (it != g_contexts.end()) {
        it->second.mutex->lock();
        clear_context(it->second);
        it->second.mutex->unlock();
        g_contexts.erase(it);
    }
}

static void native_shutdown_impl() { reset_runtime_state(); }
static jstring native_runtime_info_impl(JNIEnv* env) {
    return env->NewStringUTF((g_cfg.runtime_info.empty() ? "uninitialized" : g_cfg.runtime_info).c_str());
}
static jstring native_last_error_impl(JNIEnv* env) {
    return env->NewStringUTF(g_last_native_error.c_str());
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

AERO_LBM_CAPI_EXPORT int aero_lbm_step_rect_with_sparse_overlays(
    const float* packet,
    int nx,
    int ny,
    int nz,
    long long context_key,
    const int* overlay_cells,
    const float* overlay_values,
    int overlay_count,
    float* output_flow
) {
    return native_step_raw_dims_with_sparse_overlays_impl(
        packet,
        nx,
        ny,
        nz,
        static_cast<jlong>(context_key),
        overlay_cells,
        overlay_values,
        overlay_count,
        output_flow
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_shift_context(int grid_size, long long context_key, int dx, int dy, int dz) {
    return native_shift_context_impl(grid_size, static_cast<jlong>(context_key), dx, dy, dz) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_has_context(long long context_key) {
    return native_has_context_impl(static_cast<jlong>(context_key)) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_exchange_halo(
    int grid_size,
    long long first_context_key,
    long long second_context_key,
    int offset_x,
    int offset_y,
    int offset_z
) {
    return native_exchange_halo_impl(
        grid_size,
        static_cast<jlong>(first_context_key),
        static_cast<jlong>(second_context_key),
        offset_x,
        offset_y,
        offset_z
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_exchange_halo_layers_rect(
    int nx,
    int ny,
    int nz,
    int halo_layers,
    long long first_context_key,
    long long second_context_key,
    int offset_x,
    int offset_y,
    int offset_z
) {
    return native_exchange_halo_rect_layers_impl(
        nx,
        ny,
        nz,
        halo_layers,
        static_cast<jlong>(first_context_key),
        static_cast<jlong>(second_context_key),
        offset_x,
        offset_y,
        offset_z
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_get_temperature_state_rect(int nx, int ny, int nz, long long context_key, float* out_temperature) {
    return native_get_temperature_state_raw_dims_impl(nx, ny, nz, static_cast<jlong>(context_key), out_temperature) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_sample_temperature_point_rect(
    int nx,
    int ny,
    int nz,
    long long context_key,
    int sample_x,
    int sample_y,
    int sample_z,
    float* out_temperature
) {
    return native_sample_temperature_point_raw_dims_impl(
        nx,
        ny,
        nz,
        static_cast<jlong>(context_key),
        sample_x,
        sample_y,
        sample_z,
        out_temperature
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_sample_flow_point_rect(
    int nx,
    int ny,
    int nz,
    long long context_key,
    int sample_x,
    int sample_y,
    int sample_z,
    float* out_flow
) {
    return native_sample_flow_point_raw_dims_impl(
        nx,
        ny,
        nz,
        static_cast<jlong>(context_key),
        sample_x,
        sample_y,
        sample_z,
        out_flow
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_extract_flow_atlas_rect(
    int nx,
    int ny,
    int nz,
    long long context_key,
    int stride,
    float* out_flow_atlas,
    int value_count
) {
    return native_extract_flow_atlas_raw_dims_impl(
        nx,
        ny,
        nz,
        static_cast<jlong>(context_key),
        stride,
        out_flow_atlas,
        value_count
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_copy_flow_temperature_subrect(
    int nx,
    int ny,
    int nz,
    long long context_key,
    int offset_x,
    int offset_y,
    int offset_z,
    int copy_nx,
    int copy_ny,
    int copy_nz,
    float* out_flow,
    float* out_temperature
) {
    return native_copy_flow_temperature_subrect_raw_dims_impl(
        nx,
        ny,
        nz,
        static_cast<jlong>(context_key),
        offset_x,
        offset_y,
        offset_z,
        copy_nx,
        copy_ny,
        copy_nz,
        out_flow,
        out_temperature
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_get_flow_state_rect(int nx, int ny, int nz, long long context_key, float* out_flow) {
    if (!g_cfg.initialized || !out_flow) return 0;
    if (nx != g_cfg.nx || ny != g_cfg.ny || nz != g_cfg.nz) return 0;
    const std::size_t cells = static_cast<std::size_t>(nx) * ny * nz;

    LockedContext locked_context(static_cast<jlong>(context_key), false);
    if (!locked_context.ctx) return 0;
    ContextState& ctx = *locked_context.ctx;
    ensure_context_shape(ctx, nx, ny, nz, cells);
    if (g_cfg.opencl_enabled && !sync_context_state_from_gpu(ctx)) return 0;
    if (ctx.f.empty() || ctx.cells == 0) return 0;
    write_output(ctx, out_flow, 4);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_set_temperature_state_rect(
    int nx,
    int ny,
    int nz,
    long long context_key,
    const float* temperature
) {
    return native_set_temperature_state_raw_dims_impl(
        nx,
        ny,
        nz,
        static_cast<jlong>(context_key),
        temperature
    ) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_get_last_force(long long context_key, float* out_fx, float* out_fy, float* out_fz) {
    LockedContext locked_context(static_cast<jlong>(context_key), false);
    if (!locked_context.ctx) return 0;
    const ContextState& ctx = *locked_context.ctx;
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

AERO_LBM_CAPI_EXPORT const char* aero_lbm_last_error(void) {
    return g_last_native_error.c_str();
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeShiftContext(
    JNIEnv*, jclass, jint grid_size, jlong context_key, jint dx, jint dy, jint dz
) {
    return native_shift_context_impl(grid_size, context_key, dx, dy, dz);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeShiftContext(
    JNIEnv*, jclass, jint grid_size, jlong context_key, jint dx, jint dy, jint dz
) {
    return native_shift_context_impl(grid_size, context_key, dx, dy, dz);
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeHasContext(
    JNIEnv*, jclass, jlong context_key
) {
    return native_has_context_impl(context_key);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeHasContext(
    JNIEnv*, jclass, jlong context_key
) {
    return native_has_context_impl(context_key);
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeExchangeHalo(
    JNIEnv*, jclass, jint grid_size, jlong first_context_key, jlong second_context_key, jint offset_x, jint offset_y, jint offset_z
) {
    return native_exchange_halo_impl(grid_size, first_context_key, second_context_key, offset_x, offset_y, offset_z);
}
JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeExchangeHalo(
    JNIEnv*, jclass, jint grid_size, jlong first_context_key, jlong second_context_key, jint offset_x, jint offset_y, jint offset_z
) {
    return native_exchange_halo_impl(grid_size, first_context_key, second_context_key, offset_x, offset_y, offset_z);
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

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeLastError(JNIEnv* env, jclass) {
    return native_last_error_impl(env);
}
JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeLastError(JNIEnv* env, jclass) {
    return native_last_error_impl(env);
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeTimingInfo(JNIEnv* env, jclass) {
    return native_timing_info_impl(env);
}
JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeTimingInfo(JNIEnv* env, jclass) {
    return native_timing_info_impl(env);
}

JNIEXPORT jfloatArray JNICALL Java_com_aerodynamics4mc_runtime_MesoscaleNativeBridge_nativeDeriveTransport(
    JNIEnv* env,
    jclass,
    jint nx,
    jint ny,
    jint nz,
    jfloat dx_m,
    jfloat dt_s,
    jfloat molecular_nu_m2_s,
    jfloat prandtl_air,
    jfloat turbulent_prandtl
) {
    AeroLbmMesoscaleConfig cfg{};
    aero_lbm_mesoscale_default_config(&cfg);
    cfg.nx = nx;
    cfg.ny = ny;
    cfg.nz = nz;
    cfg.dx_m = dx_m;
    cfg.dt_s = dt_s;
    cfg.molecular_nu_m2_s = molecular_nu_m2_s;
    cfg.prandtl_air = prandtl_air;
    cfg.turbulent_prandtl = turbulent_prandtl;

    AeroLbmMesoscaleTransport transport{};
    if (!aero_lbm_mesoscale_derive_transport(&cfg, &transport)) {
        return nullptr;
    }
    jfloatArray out = env->NewFloatArray(5);
    if (!out) {
        return nullptr;
    }
    const jfloat values[5] = {
        transport.velocity_scale_m_s_per_lattice,
        transport.nu_molecular_lattice,
        transport.alpha_molecular_lattice,
        transport.tau_shear_molecular,
        transport.tau_thermal_molecular
    };
    env->SetFloatArrayRegion(out, 0, 5, values);
    return out;
}

}  // extern "C"
