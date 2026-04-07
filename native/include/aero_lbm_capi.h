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
    AERO_LBM_BENCHMARK_ABI_VERSION = 1,
    AERO_LBM_MESOSCALE_ABI_VERSION = 1,
    AERO_LBM_SIMULATION_ABI_VERSION = 1,
    AERO_LBM_MESOSCALE_FORCING_CHANNELS = 9,
    AERO_LBM_MESOSCALE_STATE_CHANNELS = 5,
    AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS = 4,
    AERO_LBM_SIMULATION_PACKED_ATLAS_CHANNELS = 4,
    AERO_LBM_SIMULATION_PLAYER_PROBE_CHANNELS = 6
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

typedef struct AeroLbmMesoscaleConfig {
    uint32_t abi_version;
    uint32_t struct_size;
    int nx;
    int ny;
    int nz;
    float dx_m;
    float dt_s;
    float molecular_nu_m2_s;
    float prandtl_air;
    float turbulent_prandtl;
    float reference_density_kg_m3;
    float ambient_air_temperature_k;
    float deep_ground_temperature_k;
    float background_wind_m_s[3];
    uint32_t reserved[8];
} AeroLbmMesoscaleConfig;

typedef struct AeroLbmMesoscaleTransport {
    float velocity_scale_m_s_per_lattice;
    float nu_molecular_lattice;
    float alpha_molecular_lattice;
    float tau_shear_molecular;
    float tau_thermal_molecular;
} AeroLbmMesoscaleTransport;

typedef struct AeroLbmWorldDelta {
    int type;
    int x;
    int y;
    int z;
    int data0;
    int data1;
    int data2;
    int data3;
    float value0;
    float value1;
    float value2;
    float value3;
} AeroLbmWorldDelta;

AERO_LBM_CAPI_EXPORT int aero_lbm_init(int grid_size, int input_channels, int output_channels);
AERO_LBM_CAPI_EXPORT int aero_lbm_step(const float* packet, int grid_size, long long context_key, float* output_flow);
AERO_LBM_CAPI_EXPORT int aero_lbm_init_rect(int nx, int ny, int nz, int input_channels, int output_channels);
AERO_LBM_CAPI_EXPORT int aero_lbm_step_rect(const float* packet, int nx, int ny, int nz, long long context_key, float* output_flow);
AERO_LBM_CAPI_EXPORT int aero_lbm_shift_context(int grid_size, long long context_key, int dx, int dy, int dz);
AERO_LBM_CAPI_EXPORT int aero_lbm_has_context(long long context_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_exchange_halo(
    int grid_size,
    long long first_context_key,
    long long second_context_key,
    int offset_x,
    int offset_y,
    int offset_z
);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_temperature_state_rect(int nx, int ny, int nz, long long context_key, float* out_temperature);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_flow_state_rect(int nx, int ny, int nz, long long context_key, float* out_flow);
AERO_LBM_CAPI_EXPORT int aero_lbm_set_temperature_state_rect(int nx, int ny, int nz, long long context_key, const float* temperature);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_last_force(long long context_key, float* out_fx, float* out_fy, float* out_fz);
AERO_LBM_CAPI_EXPORT void aero_lbm_release_context(long long context_key);
AERO_LBM_CAPI_EXPORT void aero_lbm_shutdown(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_runtime_info(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_last_error(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_timing_info(void);
AERO_LBM_CAPI_EXPORT void aero_lbm_reset_timing(void);
AERO_LBM_CAPI_EXPORT int aero_lbm_get_timing_snapshot(AeroLbmTimingSnapshot* out_snapshot);
AERO_LBM_CAPI_EXPORT void aero_lbm_mesoscale_default_config(AeroLbmMesoscaleConfig* out_config);
AERO_LBM_CAPI_EXPORT int aero_lbm_mesoscale_derive_transport(
    const AeroLbmMesoscaleConfig* config,
    AeroLbmMesoscaleTransport* out_transport
);
AERO_LBM_CAPI_EXPORT int aero_lbm_mesoscale_create_context(
    const AeroLbmMesoscaleConfig* config,
    long long* out_context_key
);
AERO_LBM_CAPI_EXPORT int aero_lbm_mesoscale_step_context(
    long long context_key,
    const AeroLbmMesoscaleConfig* config,
    const float* forcing,
    float* out_state
);
AERO_LBM_CAPI_EXPORT void aero_lbm_mesoscale_release_context(long long context_key);

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_create_service(long long* out_service_key);
AERO_LBM_CAPI_EXPORT void aero_lbm_simulation_release_service(long long service_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_focus(
    long long service_key,
    int block_x,
    int block_y,
    int block_z,
    int radius_blocks
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_submit_world_deltas(
    long long service_key,
    const AeroLbmWorldDelta* deltas,
    int delta_count
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_upload_static_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const uint8_t* obstacle,
    const uint8_t* surface_kind,
    const uint16_t* open_face_mask,
    const float* emitter_power_watts,
    const uint8_t* face_sky_exposure,
    const uint8_t* face_direct_exposure
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_upload_static_region_patch(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    int offset_x,
    int offset_y,
    int offset_z,
    int patch_nx,
    int patch_ny,
    int patch_nz,
    const uint8_t* obstacle,
    const uint8_t* surface_kind,
    const uint16_t* open_face_mask,
    const float* emitter_power_watts,
    const uint8_t* face_sky_exposure,
    const uint8_t* face_direct_exposure
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_activate_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_deactivate_region(long long service_key, long long region_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_has_region(long long service_key, long long region_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_is_region_ready(long long service_key, long long region_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_ensure_l2_runtime(
    long long service_key,
    int nx,
    int ny,
    int nz,
    int input_channels,
    int output_channels
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_has_region_context(long long service_key, long long region_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_upload_region_forcing(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* thermal_source,
    const uint8_t* fan_mask,
    const float* fan_vx,
    const float* fan_vy,
    const float* fan_vz
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_refresh_region_thermal(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float direct_solar_flux_w_m2,
    float diffuse_solar_flux_w_m2,
    float ambient_air_temperature_k,
    float deep_ground_temperature_k,
    float sky_temperature_k,
    float precipitation_temperature_k,
    float precipitation_strength,
    float sun_x,
    float sun_y,
    float sun_z,
    float surface_delta_seconds
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_step_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* packet,
    float* out_flow_state
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_step_region_stored(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float boundary_wind_x,
    float boundary_wind_y,
    float boundary_wind_z,
    float* out_max_speed
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_exchange_region_halo(
    long long service_key,
    long long first_region_key,
    long long second_region_key,
    int grid_size,
    int offset_x,
    int offset_y,
    int offset_z
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_region_temperature_state(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float* out_temperature
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_region_flow_state(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float* out_flow_state
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_region_temperature_state(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* temperature
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_release_region_runtime(long long service_key, long long region_key);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_import_dynamic_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* flow_state,
    const float* air_temperature,
    const float* surface_temperature
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_export_dynamic_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float* out_flow_state,
    float* out_air_temperature,
    float* out_surface_temperature
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_packed_flow_atlas(
    long long service_key,
    long long atlas_key,
    const int16_t* atlas_values,
    int value_count
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_poll_packed_flow_atlas(
    long long service_key,
    long long atlas_key,
    int16_t* out_atlas_values,
    int value_count
);
AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_sample_region_point(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    int sample_x,
    int sample_y,
    int sample_z,
    float* out_probe_values,
    int value_count
);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_simulation_runtime_info(void);
AERO_LBM_CAPI_EXPORT const char* aero_lbm_simulation_last_error(void);

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
