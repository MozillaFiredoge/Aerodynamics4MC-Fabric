#include "aero_lbm_capi.h"

#include <jni.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct StaticRegionData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<uint8_t> obstacle;
    std::vector<uint8_t> surface_kind;
    std::vector<uint16_t> open_face_mask;
    std::vector<float> emitter_power_watts;
};

struct DynamicRegionData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<float> flow_state;
    std::vector<float> air_temperature;
    std::vector<float> surface_temperature;
};

struct AtlasData {
    std::vector<int16_t> values;
};

struct RegionLifecycleData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    bool active = false;
};

struct ServiceState {
    int focus_x = 0;
    int focus_y = 0;
    int focus_z = 0;
    int focus_radius_blocks = 0;
    bool l2_runtime_initialized = false;
    int l2_nx = 0;
    int l2_ny = 0;
    int l2_nz = 0;
    int l2_input_channels = 0;
    int l2_output_channels = 0;
    std::vector<AeroLbmWorldDelta> world_deltas;
    std::unordered_map<long long, RegionLifecycleData> regions;
    std::unordered_map<long long, StaticRegionData> static_regions;
    std::unordered_map<long long, DynamicRegionData> dynamic_regions;
    std::unordered_map<long long, AtlasData> atlases;
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

SpinMutex g_simulation_mutex;
ServiceState g_service_storage;
ServiceState* g_service = nullptr;
long long g_service_key = 0;
std::string g_simulation_last_error;

constexpr int k_default_packed_atlas_stride = 4;
constexpr float k_atlas_velocity_quant_range = 5.6f;
constexpr float k_atlas_pressure_quant_range = 0.03f;

void set_simulation_last_error(const std::string& message) {
    g_simulation_last_error = message;
}

ServiceState* lookup_service(long long service_key) {
    if (!g_service || g_service_key == 0 || service_key != g_service_key) {
        return nullptr;
    }
    return g_service;
}

bool checked_cell_count(int nx, int ny, int nz, int* out_cells) {
    if (nx <= 0 || ny <= 0 || nz <= 0 || !out_cells) {
        return false;
    }
    const long long cells = static_cast<long long>(nx) * static_cast<long long>(ny) * static_cast<long long>(nz);
    if (cells <= 0 || cells > static_cast<long long>(std::numeric_limits<int>::max())) {
        return false;
    }
    *out_cells = static_cast<int>(cells);
    return true;
}

jstring new_java_string(JNIEnv* env, const char* text) {
    if (!env || !text) {
        return nullptr;
    }
    return env->NewStringUTF(text);
}

int16_t quantize_signed(float value, float range) {
    if (!(range > 0.0f) || !std::isfinite(value)) {
        return 0;
    }
    const float normalized = std::clamp(value / range, -1.0f, 1.0f);
    return static_cast<int16_t>(std::lround(normalized * 32767.0f));
}

bool ensure_l2_runtime(ServiceState& service, int nx, int ny, int nz, int input_channels, int output_channels) {
    if (service.l2_runtime_initialized
        && service.l2_nx == nx
        && service.l2_ny == ny
        && service.l2_nz == nz
        && service.l2_input_channels == input_channels
        && service.l2_output_channels == output_channels) {
        return true;
    }
    if (!aero_lbm_init_rect(nx, ny, nz, input_channels, output_channels)) {
        set_simulation_last_error(std::string("simulation_l2_runtime_init failed: ") + aero_lbm_last_error());
        return false;
    }
    service.l2_runtime_initialized = true;
    service.l2_nx = nx;
    service.l2_ny = ny;
    service.l2_nz = nz;
    service.l2_input_channels = input_channels;
    service.l2_output_channels = output_channels;
    return true;
}

void rebuild_default_packed_atlas(ServiceState& service, long long region_key) {
    auto region_it = service.dynamic_regions.find(region_key);
    if (region_it == service.dynamic_regions.end()) {
        service.atlases.erase(region_key);
        return;
    }
    const DynamicRegionData& region = region_it->second;
    if (region.nx <= 0 || region.ny <= 0 || region.nz <= 0 || region.flow_state.empty()) {
        service.atlases.erase(region_key);
        return;
    }
    const int sx = (region.nx + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int sy = (region.ny + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int sz = (region.nz + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    AtlasData& atlas = service.atlases[region_key];
    atlas.values.assign(
        static_cast<size_t>(sx) * static_cast<size_t>(sy) * static_cast<size_t>(sz)
            * AERO_LBM_SIMULATION_PACKED_ATLAS_CHANNELS,
        0
    );
    size_t dst = 0;
    for (int x = 0; x < sx; ++x) {
        const int gx = std::min(region.nx - 1, x * k_default_packed_atlas_stride);
        for (int y = 0; y < sy; ++y) {
            const int gy = std::min(region.ny - 1, y * k_default_packed_atlas_stride);
            for (int z = 0; z < sz; ++z) {
                const int gz = std::min(region.nz - 1, z * k_default_packed_atlas_stride);
                const int cell = (gx * region.ny + gy) * region.nz + gz;
                const size_t src = static_cast<size_t>(cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
                atlas.values[dst] = quantize_signed(region.flow_state[src], k_atlas_velocity_quant_range);
                atlas.values[dst + 1] = quantize_signed(region.flow_state[src + 1], k_atlas_velocity_quant_range);
                atlas.values[dst + 2] = quantize_signed(region.flow_state[src + 2], k_atlas_velocity_quant_range);
                atlas.values[dst + 3] = quantize_signed(region.flow_state[src + 3], k_atlas_pressure_quant_range);
                dst += AERO_LBM_SIMULATION_PACKED_ATLAS_CHANNELS;
            }
        }
    }
}

}  // namespace

extern "C" {

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_create_service(long long* out_service_key) {
    if (!out_service_key) {
        set_simulation_last_error("simulation_create_service: out_service_key is null");
        return 0;
    }
    if (!g_service) {
        g_service = &g_service_storage;
        g_service_key = 1;
    }
    *out_service_key = g_service_key;
    return 1;
}

AERO_LBM_CAPI_EXPORT void aero_lbm_simulation_release_service(long long service_key) {
    if (g_service && service_key == g_service_key) {
        g_service = nullptr;
        g_service_key = 0;
    }
    if (!g_service) {
        aero_lbm_shutdown();
    }
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_focus(
    long long service_key,
    int block_x,
    int block_y,
    int block_z,
    int radius_blocks
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_set_focus: missing service");
        return 0;
    }
    service->focus_x = block_x;
    service->focus_y = block_y;
    service->focus_z = block_z;
    service->focus_radius_blocks = radius_blocks;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_submit_world_deltas(
    long long service_key,
    const AeroLbmWorldDelta* deltas,
    int delta_count
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_submit_world_deltas: missing service");
        return 0;
    }
    if (delta_count < 0) {
        set_simulation_last_error("simulation_submit_world_deltas: negative delta_count");
        return 0;
    }
    if (delta_count > 0 && !deltas) {
        set_simulation_last_error("simulation_submit_world_deltas: deltas is null");
        return 0;
    }
    service->world_deltas.insert(service->world_deltas.end(), deltas, deltas + delta_count);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_upload_static_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const uint8_t* obstacle,
    const uint8_t* surface_kind,
    const uint16_t* open_face_mask,
    const float* emitter_power_watts
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_static_region: invalid dimensions");
        return 0;
    }
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_static_region: null region buffers");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_upload_static_region: missing service");
        return 0;
    }

    StaticRegionData& region = service->static_regions[region_key];
    region.nx = nx;
    region.ny = ny;
    region.nz = nz;
    region.obstacle.assign(obstacle, obstacle + cells);
    region.surface_kind.assign(surface_kind, surface_kind + cells);
    region.open_face_mask.assign(open_face_mask, open_face_mask + cells);
    region.emitter_power_watts.assign(emitter_power_watts, emitter_power_watts + cells);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_activate_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_activate_region: invalid dimensions");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_activate_region: missing service");
        return 0;
    }
    RegionLifecycleData& region = service->regions[region_key];
    region.nx = nx;
    region.ny = ny;
    region.nz = nz;
    region.active = true;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_deactivate_region(long long service_key, long long region_key) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_deactivate_region: missing service");
        return 0;
    }
    auto region_it = service->regions.find(region_key);
    if (region_it == service->regions.end()) {
        return 1;
    }
    region_it->second.active = false;
    aero_lbm_release_context(region_key);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_has_region(long long service_key, long long region_key) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_has_region: missing service");
        return 0;
    }
    return service->regions.find(region_key) != service->regions.end()
        || service->static_regions.find(region_key) != service->static_regions.end()
        || service->dynamic_regions.find(region_key) != service->dynamic_regions.end()
        || service->atlases.find(region_key) != service->atlases.end();
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_is_region_ready(long long service_key, long long region_key) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_is_region_ready: missing service");
        return 0;
    }
    auto lifecycle_it = service->regions.find(region_key);
    if (lifecycle_it == service->regions.end() || !lifecycle_it->second.active) {
        return 0;
    }
    return service->static_regions.find(region_key) != service->static_regions.end();
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_ensure_l2_runtime(
    long long service_key,
    int nx,
    int ny,
    int nz,
    int input_channels,
    int output_channels
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_ensure_l2_runtime: missing service");
        return 0;
    }
    return ensure_l2_runtime(*service, nx, ny, nz, input_channels, output_channels) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_has_region_context(long long service_key, long long region_key) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_has_region_context: missing service");
        return 0;
    }
    return aero_lbm_has_context(region_key);
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_step_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* packet,
    float* out_flow_state
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells) || !packet || !out_flow_state) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_step_region: invalid arguments");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_step_region: missing service");
        return 0;
    }
    auto lifecycle_it = service->regions.find(region_key);
    if (lifecycle_it == service->regions.end() || !lifecycle_it->second.active) {
        set_simulation_last_error("simulation_step_region: inactive region");
        return 0;
    }
    if (!ensure_l2_runtime(*service, nx, ny, nz, 10, 4)) {
        return 0;
    }
    if (!aero_lbm_step_rect(packet, nx, ny, nz, region_key, out_flow_state)) {
        set_simulation_last_error(std::string("simulation_step_region failed: ") + aero_lbm_last_error());
        return 0;
    }
    DynamicRegionData& dynamic = service->dynamic_regions[region_key];
    dynamic.nx = nx;
    dynamic.ny = ny;
    dynamic.nz = nz;
    dynamic.flow_state.assign(
        out_flow_state,
        out_flow_state + static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS
    );
    if (dynamic.air_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.air_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }
    if (dynamic.surface_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.surface_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }
    if (!aero_lbm_get_temperature_state_rect(nx, ny, nz, region_key, dynamic.air_temperature.data())) {
        set_simulation_last_error(std::string("simulation_step_region temperature sync failed: ") + aero_lbm_last_error());
        return 0;
    }
    rebuild_default_packed_atlas(*service, region_key);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_exchange_region_halo(
    long long service_key,
    long long first_region_key,
    long long second_region_key,
    int grid_size,
    int offset_x,
    int offset_y,
    int offset_z
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_exchange_region_halo: missing service");
        return 0;
    }
    if (!aero_lbm_exchange_halo(grid_size, first_region_key, second_region_key, offset_x, offset_y, offset_z)) {
        set_simulation_last_error(std::string("simulation_exchange_region_halo failed: ") + aero_lbm_last_error());
        return 0;
    }
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_region_temperature_state(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float* out_temperature
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells) || !out_temperature) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_get_region_temperature_state: invalid arguments");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_get_region_temperature_state: missing service");
        return 0;
    }
    if (!ensure_l2_runtime(*service, nx, ny, nz, 10, 4)) {
        return 0;
    }
    if (!aero_lbm_get_temperature_state_rect(nx, ny, nz, region_key, out_temperature)) {
        set_simulation_last_error(std::string("simulation_get_region_temperature_state failed: ") + aero_lbm_last_error());
        return 0;
    }
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_region_temperature_state(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* temperature
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells) || !temperature) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_set_region_temperature_state: invalid arguments");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_set_region_temperature_state: missing service");
        return 0;
    }
    if (!ensure_l2_runtime(*service, nx, ny, nz, 10, 4)) {
        return 0;
    }
    if (!aero_lbm_set_temperature_state_rect(nx, ny, nz, region_key, temperature)) {
        set_simulation_last_error(std::string("simulation_set_region_temperature_state failed: ") + aero_lbm_last_error());
        return 0;
    }
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_release_region_runtime(long long service_key, long long region_key) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_release_region_runtime: missing service");
        return 0;
    }
    aero_lbm_release_context(region_key);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_import_dynamic_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    const float* flow_state,
    const float* air_temperature,
    const float* surface_temperature
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_import_dynamic_region: invalid dimensions");
        return 0;
    }
    if (!flow_state || !air_temperature || !surface_temperature) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_import_dynamic_region: null region buffers");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_import_dynamic_region: missing service");
        return 0;
    }

    DynamicRegionData& region = service->dynamic_regions[region_key];
    region.nx = nx;
    region.ny = ny;
    region.nz = nz;
    region.flow_state.assign(flow_state, flow_state + cells * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS);
    region.air_temperature.assign(air_temperature, air_temperature + cells);
    region.surface_temperature.assign(surface_temperature, surface_temperature + cells);
    rebuild_default_packed_atlas(*service, region_key);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_export_dynamic_region(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float* out_flow_state,
    float* out_air_temperature,
    float* out_surface_temperature
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_export_dynamic_region: invalid dimensions");
        return 0;
    }
    if (!out_flow_state || !out_air_temperature || !out_surface_temperature) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_export_dynamic_region: null output buffers");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_export_dynamic_region: missing service");
        return 0;
    }
    auto region_it = service->dynamic_regions.find(region_key);
    if (region_it == service->dynamic_regions.end()) {
        set_simulation_last_error("simulation_export_dynamic_region: missing region");
        return 0;
    }
    const DynamicRegionData& region = region_it->second;
    if (region.nx != nx || region.ny != ny || region.nz != nz) {
        set_simulation_last_error("simulation_export_dynamic_region: dimension mismatch");
        return 0;
    }

    std::copy(region.flow_state.begin(), region.flow_state.end(), out_flow_state);
    std::copy(region.air_temperature.begin(), region.air_temperature.end(), out_air_temperature);
    std::copy(region.surface_temperature.begin(), region.surface_temperature.end(), out_surface_temperature);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_packed_flow_atlas(
    long long service_key,
    long long atlas_key,
    const int16_t* atlas_values,
    int value_count
) {
    if (value_count <= 0 || !atlas_values) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_set_packed_flow_atlas: invalid atlas buffer");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_set_packed_flow_atlas: missing service");
        return 0;
    }
    service->atlases[atlas_key].values.assign(atlas_values, atlas_values + value_count);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_poll_packed_flow_atlas(
    long long service_key,
    long long atlas_key,
    int16_t* out_atlas_values,
    int value_count
) {
    if (value_count <= 0 || !out_atlas_values) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_poll_packed_flow_atlas: invalid output buffer");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_poll_packed_flow_atlas: missing service");
        return 0;
    }
    auto atlas_it = service->atlases.find(atlas_key);
    if (atlas_it == service->atlases.end()) {
        set_simulation_last_error("simulation_poll_packed_flow_atlas: missing atlas");
        return 0;
    }
    if (static_cast<int>(atlas_it->second.values.size()) != value_count) {
        set_simulation_last_error("simulation_poll_packed_flow_atlas: size mismatch");
        return 0;
    }
    std::copy(atlas_it->second.values.begin(), atlas_it->second.values.end(), out_atlas_values);
    return 1;
}

AERO_LBM_CAPI_EXPORT const char* aero_lbm_simulation_runtime_info(void) {
    static std::string text;
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    size_t active_region_count = 0;
    size_t static_region_count = 0;
    size_t dynamic_region_count = 0;
    size_t atlas_count = 0;
    if (g_service) {
        for (const auto& region : g_service->regions) {
            if (region.second.active) {
                ++active_region_count;
            }
        }
        static_region_count = g_service->static_regions.size();
        dynamic_region_count = g_service->dynamic_regions.size();
        atlas_count = g_service->atlases.size();
    }
    text = "simulation_bridge|services=" + std::to_string(g_service ? 1 : 0)
        + "|active_regions=" + std::to_string(active_region_count)
        + "|static_regions=" + std::to_string(static_region_count)
        + "|dynamic_regions=" + std::to_string(dynamic_region_count)
        + "|atlases=" + std::to_string(atlas_count);
    return text.c_str();
}

AERO_LBM_CAPI_EXPORT const char* aero_lbm_simulation_last_error(void) {
    static std::string text;
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    text = g_simulation_last_error;
    return text.c_str();
}

JNIEXPORT jlong JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeCreateService(JNIEnv*, jclass) {
    long long key = 0;
    return aero_lbm_simulation_create_service(&key) ? static_cast<jlong>(key) : 0;
}

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeReleaseService(
    JNIEnv*,
    jclass,
    jlong service_key
) {
    aero_lbm_simulation_release_service(static_cast<long long>(service_key));
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSetFocus(
    JNIEnv*,
    jclass,
    jlong service_key,
    jint block_x,
    jint block_y,
    jint block_z,
    jint radius_blocks
) {
    return aero_lbm_simulation_set_focus(
        static_cast<long long>(service_key),
        block_x,
        block_y,
        block_z,
        radius_blocks
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSubmitWorldDeltas(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jintArray encoded_ints,
    jfloatArray encoded_floats
) {
    if (!encoded_ints || !encoded_floats) {
        return JNI_FALSE;
    }
    const jsize ints_length = env->GetArrayLength(encoded_ints);
    const jsize floats_length = env->GetArrayLength(encoded_floats);
    if (ints_length % 8 != 0 || floats_length % 4 != 0) {
        return JNI_FALSE;
    }
    const int delta_count = static_cast<int>(ints_length / 8);
    if (delta_count != floats_length / 4) {
        return JNI_FALSE;
    }
    jboolean ints_copy = JNI_FALSE;
    jboolean floats_copy = JNI_FALSE;
    jint* ints_ptr = env->GetIntArrayElements(encoded_ints, &ints_copy);
    jfloat* floats_ptr = env->GetFloatArrayElements(encoded_floats, &floats_copy);
    if (!ints_ptr || !floats_ptr) {
        if (ints_ptr) {
            env->ReleaseIntArrayElements(encoded_ints, ints_ptr, JNI_ABORT);
        }
        if (floats_ptr) {
            env->ReleaseFloatArrayElements(encoded_floats, floats_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }

    std::vector<AeroLbmWorldDelta> deltas(delta_count);
    for (int i = 0; i < delta_count; i++) {
        const int int_base = i * 8;
        const int float_base = i * 4;
        AeroLbmWorldDelta& delta = deltas[static_cast<size_t>(i)];
        delta.type = ints_ptr[int_base];
        delta.x = ints_ptr[int_base + 1];
        delta.y = ints_ptr[int_base + 2];
        delta.z = ints_ptr[int_base + 3];
        delta.data0 = ints_ptr[int_base + 4];
        delta.data1 = ints_ptr[int_base + 5];
        delta.data2 = ints_ptr[int_base + 6];
        delta.data3 = ints_ptr[int_base + 7];
        delta.value0 = floats_ptr[float_base];
        delta.value1 = floats_ptr[float_base + 1];
        delta.value2 = floats_ptr[float_base + 2];
        delta.value3 = floats_ptr[float_base + 3];
    }

    const int ok = aero_lbm_simulation_submit_world_deltas(
        static_cast<long long>(service_key),
        deltas.data(),
        delta_count
    );
    env->ReleaseIntArrayElements(encoded_ints, ints_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(encoded_floats, floats_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeUploadStaticRegion(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jbyteArray obstacle,
    jbyteArray surface_kind,
    jshortArray open_face_mask,
    jfloatArray emitter_power_watts
) {
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts) {
        return JNI_FALSE;
    }
    jboolean obstacle_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jboolean open_copy = JNI_FALSE;
    jboolean emitter_copy = JNI_FALSE;
    jbyte* obstacle_ptr = env->GetByteArrayElements(obstacle, &obstacle_copy);
    jbyte* surface_ptr = env->GetByteArrayElements(surface_kind, &surface_copy);
    jshort* open_ptr = env->GetShortArrayElements(open_face_mask, &open_copy);
    jfloat* emitter_ptr = env->GetFloatArrayElements(emitter_power_watts, &emitter_copy);
    if (!obstacle_ptr || !surface_ptr || !open_ptr || !emitter_ptr) {
        if (obstacle_ptr) {
            env->ReleaseByteArrayElements(obstacle, obstacle_ptr, JNI_ABORT);
        }
        if (surface_ptr) {
            env->ReleaseByteArrayElements(surface_kind, surface_ptr, JNI_ABORT);
        }
        if (open_ptr) {
            env->ReleaseShortArrayElements(open_face_mask, open_ptr, JNI_ABORT);
        }
        if (emitter_ptr) {
            env->ReleaseFloatArrayElements(emitter_power_watts, emitter_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_upload_static_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        reinterpret_cast<const uint8_t*>(obstacle_ptr),
        reinterpret_cast<const uint8_t*>(surface_ptr),
        reinterpret_cast<const uint16_t*>(open_ptr),
        emitter_ptr
    );
    env->ReleaseByteArrayElements(obstacle, obstacle_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(surface_kind, surface_ptr, JNI_ABORT);
    env->ReleaseShortArrayElements(open_face_mask, open_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(emitter_power_watts, emitter_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeActivateRegion(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz
) {
    return aero_lbm_simulation_activate_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeDeactivateRegion(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key
) {
    return aero_lbm_simulation_deactivate_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key)
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeHasRegion(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key
) {
    return aero_lbm_simulation_has_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key)
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeIsRegionReady(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key
) {
    return aero_lbm_simulation_is_region_ready(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key)
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeEnsureL2Runtime(
    JNIEnv*,
    jclass,
    jlong service_key,
    jint nx,
    jint ny,
    jint nz,
    jint input_channels,
    jint output_channels
) {
    return aero_lbm_simulation_ensure_l2_runtime(
        static_cast<long long>(service_key),
        nx,
        ny,
        nz,
        input_channels,
        output_channels
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeHasRegionContext(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key
) {
    return aero_lbm_simulation_has_region_context(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key)
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeStepRegion(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jobject payload,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray output_flow
) {
    if (!payload || !output_flow) {
        return JNI_FALSE;
    }
    void* payload_ptr = env->GetDirectBufferAddress(payload);
    if (!payload_ptr) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0 || env->GetArrayLength(output_flow) != static_cast<jsize>(cells * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS)) {
        return JNI_FALSE;
    }
    jboolean flow_copy = JNI_FALSE;
    jfloat* flow_ptr = env->GetFloatArrayElements(output_flow, &flow_copy);
    if (!flow_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_step_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        static_cast<const float*>(payload_ptr),
        flow_ptr
    );
    env->ReleaseFloatArrayElements(output_flow, flow_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeExchangeRegionHalo(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong first_region_key,
    jlong second_region_key,
    jint grid_size,
    jint offset_x,
    jint offset_y,
    jint offset_z
) {
    return aero_lbm_simulation_exchange_region_halo(
        static_cast<long long>(service_key),
        static_cast<long long>(first_region_key),
        static_cast<long long>(second_region_key),
        grid_size,
        offset_x,
        offset_y,
        offset_z
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeGetRegionTemperatureState(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray out_temperature
) {
    if (!out_temperature) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0 || env->GetArrayLength(out_temperature) != static_cast<jsize>(cells)) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jfloat* temperature_ptr = env->GetFloatArrayElements(out_temperature, &copy);
    if (!temperature_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_get_region_temperature_state(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        temperature_ptr
    );
    env->ReleaseFloatArrayElements(out_temperature, temperature_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSetRegionTemperatureState(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray temperature
) {
    if (!temperature) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0 || env->GetArrayLength(temperature) != static_cast<jsize>(cells)) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jfloat* temperature_ptr = env->GetFloatArrayElements(temperature, &copy);
    if (!temperature_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_set_region_temperature_state(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        temperature_ptr
    );
    env->ReleaseFloatArrayElements(temperature, temperature_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeReleaseRegionRuntime(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key
) {
    return aero_lbm_simulation_release_region_runtime(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key)
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeImportDynamicRegion(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray flow_state,
    jfloatArray air_temperature,
    jfloatArray surface_temperature
) {
    if (!flow_state || !air_temperature || !surface_temperature) {
        return JNI_FALSE;
    }
    jboolean flow_copy = JNI_FALSE;
    jboolean air_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jfloat* flow_ptr = env->GetFloatArrayElements(flow_state, &flow_copy);
    jfloat* air_ptr = env->GetFloatArrayElements(air_temperature, &air_copy);
    jfloat* surface_ptr = env->GetFloatArrayElements(surface_temperature, &surface_copy);
    if (!flow_ptr || !air_ptr || !surface_ptr) {
        if (flow_ptr) {
            env->ReleaseFloatArrayElements(flow_state, flow_ptr, JNI_ABORT);
        }
        if (air_ptr) {
            env->ReleaseFloatArrayElements(air_temperature, air_ptr, JNI_ABORT);
        }
        if (surface_ptr) {
            env->ReleaseFloatArrayElements(surface_temperature, surface_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_import_dynamic_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        flow_ptr,
        air_ptr,
        surface_ptr
    );
    env->ReleaseFloatArrayElements(flow_state, flow_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(air_temperature, air_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(surface_temperature, surface_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeExportDynamicRegion(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray out_flow_state,
    jfloatArray out_air_temperature,
    jfloatArray out_surface_temperature
) {
    if (!out_flow_state || !out_air_temperature || !out_surface_temperature) {
        return JNI_FALSE;
    }
    jboolean flow_copy = JNI_FALSE;
    jboolean air_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jfloat* flow_ptr = env->GetFloatArrayElements(out_flow_state, &flow_copy);
    jfloat* air_ptr = env->GetFloatArrayElements(out_air_temperature, &air_copy);
    jfloat* surface_ptr = env->GetFloatArrayElements(out_surface_temperature, &surface_copy);
    if (!flow_ptr || !air_ptr || !surface_ptr) {
        if (flow_ptr) {
            env->ReleaseFloatArrayElements(out_flow_state, flow_ptr, 0);
        }
        if (air_ptr) {
            env->ReleaseFloatArrayElements(out_air_temperature, air_ptr, 0);
        }
        if (surface_ptr) {
            env->ReleaseFloatArrayElements(out_surface_temperature, surface_ptr, 0);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_export_dynamic_region(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        flow_ptr,
        air_ptr,
        surface_ptr
    );
    env->ReleaseFloatArrayElements(out_flow_state, flow_ptr, 0);
    env->ReleaseFloatArrayElements(out_air_temperature, air_ptr, 0);
    env->ReleaseFloatArrayElements(out_surface_temperature, surface_ptr, 0);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSetPackedFlowAtlas(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong atlas_key,
    jshortArray atlas_values
) {
    if (!atlas_values) {
        return JNI_FALSE;
    }
    const jsize length = env->GetArrayLength(atlas_values);
    jboolean copy = JNI_FALSE;
    jshort* values_ptr = env->GetShortArrayElements(atlas_values, &copy);
    if (!values_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_set_packed_flow_atlas(
        static_cast<long long>(service_key),
        static_cast<long long>(atlas_key),
        reinterpret_cast<const int16_t*>(values_ptr),
        static_cast<int>(length)
    );
    env->ReleaseShortArrayElements(atlas_values, values_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativePollPackedFlowAtlas(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong atlas_key,
    jshortArray out_atlas_values
) {
    if (!out_atlas_values) {
        return JNI_FALSE;
    }
    const jsize length = env->GetArrayLength(out_atlas_values);
    jboolean copy = JNI_FALSE;
    jshort* values_ptr = env->GetShortArrayElements(out_atlas_values, &copy);
    if (!values_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_poll_packed_flow_atlas(
        static_cast<long long>(service_key),
        static_cast<long long>(atlas_key),
        reinterpret_cast<int16_t*>(values_ptr),
        static_cast<int>(length)
    );
    env->ReleaseShortArrayElements(out_atlas_values, values_ptr, 0);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeRuntimeInfo(JNIEnv* env, jclass) {
    return new_java_string(env, aero_lbm_simulation_runtime_info());
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeLastError(JNIEnv* env, jclass) {
    return new_java_string(env, aero_lbm_simulation_last_error());
}

}  // extern "C"
