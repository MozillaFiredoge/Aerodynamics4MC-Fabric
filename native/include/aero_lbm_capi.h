#ifndef AERO_LBM_CAPI_H
#define AERO_LBM_CAPI_H

#include <stdint.h>

#if defined(_WIN32)
#define AERO_LBM_CAPI_EXPORT __declspec(dllexport)
#else
#define AERO_LBM_CAPI_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    AERO_LBM_BENCHMARK_ABI_VERSION = 1
};

typedef enum AeroLbmBenchmarkPreset {
    AERO_LBM_BENCHMARK_PRESET_NONE = 0,
    AERO_LBM_BENCHMARK_PRESET_TAYLOR_GREEN_3D = 1,
    AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_2D = 2,
    AERO_LBM_BENCHMARK_PRESET_LID_DRIVEN_CAVITY_3D = 3,
    AERO_LBM_BENCHMARK_PRESET_CYLINDER_CROSSFLOW_2D = 4,
    AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_2D = 5,
    AERO_LBM_BENCHMARK_PRESET_DIFFERENTIALLY_HEATED_CAVITY_3D = 6
} AeroLbmBenchmarkPreset;

typedef enum AeroLbmHydrodynamicBoundaryKind {
    AERO_LBM_HYDRO_BOUNDARY_INHERIT_GAME = 0,
    AERO_LBM_HYDRO_BOUNDARY_PERIODIC = 1,
    AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK = 2,
    AERO_LBM_HYDRO_BOUNDARY_MOVING_WALL = 3,
    AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET = 4,
    AERO_LBM_HYDRO_BOUNDARY_PRESSURE_DIRICHLET = 5,
    AERO_LBM_HYDRO_BOUNDARY_CONVECTIVE_OUTFLOW = 6,
    AERO_LBM_HYDRO_BOUNDARY_SYMMETRY = 7
} AeroLbmHydrodynamicBoundaryKind;

typedef enum AeroLbmThermalBoundaryKind {
    AERO_LBM_THERMAL_BOUNDARY_INHERIT_GAME = 0,
    AERO_LBM_THERMAL_BOUNDARY_DISABLED = 1,
    AERO_LBM_THERMAL_BOUNDARY_PERIODIC = 2,
    AERO_LBM_THERMAL_BOUNDARY_ADIABATIC = 3,
    AERO_LBM_THERMAL_BOUNDARY_TEMPERATURE_DIRICHLET = 4,
    AERO_LBM_THERMAL_BOUNDARY_HEAT_FLUX_NEUMANN = 5
} AeroLbmThermalBoundaryKind;

typedef enum AeroLbmBenchmarkFlags {
    AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_FORCING = 1u << 0,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_NOISE = 1u << 1,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_SPONGE = 1u << 2,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_CONVECTIVE_OUTFLOW = 1u << 3,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_OBSTACLE_BOUNCE_BLEND = 1u << 4,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_SGS = 1u << 5,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE = 1u << 6,
    AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY = 1u << 7
} AeroLbmBenchmarkFlags;

typedef struct AeroLbmBoundaryFaceConfig {
    int hydrodynamic_kind;
    int thermal_kind;
    float velocity[3];
    float pressure;
    float temperature;
    float heat_flux;
} AeroLbmBoundaryFaceConfig;

typedef struct AeroLbmBenchmarkConfig {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t flags;
    int enabled;
    int preset;
    int reserved0;
    float reynolds_number;
    float rayleigh_number;
    float prandtl_number;
    float mach_number;
    float reference_density;
    float reference_temperature;
    float reference_length;
    float body_force[3];
    float gravity[3];
    float initial_velocity[3];
    float initial_pressure;
    float initial_temperature;
    AeroLbmBoundaryFaceConfig x_min;
    AeroLbmBoundaryFaceConfig x_max;
    AeroLbmBoundaryFaceConfig y_min;
    AeroLbmBoundaryFaceConfig y_max;
    AeroLbmBoundaryFaceConfig z_min;
    AeroLbmBoundaryFaceConfig z_max;
    uint32_t reserved[8];
} AeroLbmBenchmarkConfig;

typedef struct AeroLbmTimingSnapshot {
    uint64_t ticks;
    double last_payload_copy_ms;
    double last_solver_ms;
    double last_readback_ms;
    double last_total_ms;
    double avg_payload_copy_ms;
    double avg_solver_ms;
    double avg_readback_ms;
    double avg_total_ms;
} AeroLbmTimingSnapshot;

AERO_LBM_CAPI_EXPORT int aero_lbm_init(int grid_size, int input_channels, int output_channels);
AERO_LBM_CAPI_EXPORT int aero_lbm_step(const float* packet, int grid_size, long long context_key, float* output_flow);
AERO_LBM_CAPI_EXPORT int aero_lbm_init_rect(int nx, int ny, int nz, int input_channels, int output_channels);
AERO_LBM_CAPI_EXPORT int aero_lbm_step_rect(const float* packet, int nx, int ny, int nz, long long context_key, float* output_flow);
AERO_LBM_CAPI_EXPORT int aero_lbm_shift_context(int grid_size, long long context_key, int dx, int dy, int dz);
AERO_LBM_CAPI_EXPORT int aero_lbm_exchange_halo(
    int grid_size,
    long long first_context_key,
    long long second_context_key,
    int offset_x,
    int offset_y,
    int offset_z
);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_temperature_state_rect(int nx, int ny, int nz, long long context_key, float* out_temperature);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_last_force(long long context_key, float* out_fx, float* out_fy, float* out_fz);
AERO_LBM_CAPI_EXPORT void aero_lbm_release_context(long long context_key);
AERO_LBM_CAPI_EXPORT void aero_lbm_shutdown(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_runtime_info(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_timing_info(void);
AERO_LBM_CAPI_EXPORT void aero_lbm_reset_timing(void);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_timing_snapshot(AeroLbmTimingSnapshot* out_snapshot);

AERO_LBM_CAPI_EXPORT void aero_lbm_benchmark_default_config(AeroLbmBenchmarkConfig* out_config);
AERO_LBM_CAPI_EXPORT int aero_lbm_benchmark_default_preset_config(int preset, AeroLbmBenchmarkConfig* out_config);
AERO_LBM_CAPI_EXPORT int aero_lbm_benchmark_set_config(const AeroLbmBenchmarkConfig* config);
AERO_LBM_CAPI_EXPORT int aero_lbm_benchmark_get_config(AeroLbmBenchmarkConfig* out_config);
AERO_LBM_CAPI_EXPORT void aero_lbm_benchmark_reset_config(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_benchmark_info(void);

#ifdef __cplusplus
}
#endif

#endif
