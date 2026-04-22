#include "aero_lbm_capi.h"
#include "aero_lbm_analysis_codec.hpp"

#include <jni.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
    std::vector<uint8_t> face_sky_exposure;
    std::vector<uint8_t> face_direct_exposure;
};

struct DynamicRegionData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<float> flow_state;
    std::vector<float> air_temperature;
    std::vector<float> surface_temperature;
    struct NestedFeedbackBinLayout {
        int cell_x = 0;
        int layer = 0;
        int cell_z = 0;
        int min_x = 0;
        int max_x = 0;
        int min_y = 0;
        int max_y = 0;
        int min_z = 0;
        int max_z = 0;
    };
    struct NestedFeedbackBinAccumulator {
        double volume_sum = 0.0;
        double density_sum = 0.0;
        double momentum_x_sum = 0.0;
        double momentum_z_sum = 0.0;
        double air_temperature_volume_sum = 0.0;
        double surface_temperature_volume_sum = 0.0;
        double bottom_area_sum = 0.0;
        double bottom_mass_flux_sum = 0.0;
        double top_area_sum = 0.0;
        double top_mass_flux_sum = 0.0;
    };
    struct NestedFeedbackData {
        int steps_per_feedback = 0;
        int steps_accumulated = 0;
        int packets_emitted = 0;
        int reset_count = 0;
        std::vector<NestedFeedbackBinLayout> layout;
        std::vector<NestedFeedbackBinAccumulator> accumulators;
        std::vector<float> ready_values;
    } nested_feedback;
};

struct ForcingRegionData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<float> thermal_source;
    std::vector<uint8_t> fan_mask;
    std::vector<float> fan_vx;
    std::vector<float> fan_vy;
    std::vector<float> fan_vz;
};

struct AtlasData {
    std::vector<int16_t> values;
};

struct RegionPacketTemplateData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<float> values;
};

struct RegionLifecycleData {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    bool active = false;
};

struct BrickCoord {
    int x = 0;
    int y = 0;
    int z = 0;
};

inline bool operator==(const BrickCoord& first, const BrickCoord& second) noexcept {
    return first.x == second.x
        && first.y == second.y
        && first.z == second.z;
}

struct BrickCoordHash {
    std::size_t operator()(const BrickCoord& coord) const noexcept {
        std::size_t value = 1469598103934665603ull;
        value = (value ^ static_cast<std::size_t>(coord.x)) * 1099511628211ull;
        value = (value ^ static_cast<std::size_t>(coord.y)) * 1099511628211ull;
        value = (value ^ static_cast<std::size_t>(coord.z)) * 1099511628211ull;
        return value;
    }
};

struct BrickData {
    bool active_hint = false;
    bool active = false;
    bool geometry_dirty = false;
    bool forcing_dirty = false;
    bool pending_reinit = false;
    std::uint64_t last_hint_epoch = 0;
    std::uint64_t last_active_epoch = 0;
    long long context_key = 0;
    std::shared_ptr<const StaticRegionData> static_region;
    std::shared_ptr<DynamicRegionData> dynamic_region;
    std::vector<float> packet_cache;
};

struct FluidWorldRuntime {
    int brick_size = 0;
    float dx_meters = 1.0f;
    float dt_seconds = 0.05f;
    std::uint64_t epoch = 0;
    std::unordered_map<BrickCoord, BrickData, BrickCoordHash> bricks;
    std::unordered_set<BrickCoord, BrickCoordHash> active_hint_closure;
    std::vector<AeroLbmWorldDelta> pending_world_deltas;
};

struct PendingStaticBrickUpload {
    long long world_key = 0;
    BrickCoord coord;
    std::shared_ptr<const StaticRegionData> static_region;
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
    std::unordered_map<long long, std::shared_ptr<const StaticRegionData>> static_regions;
    std::unordered_map<long long, DynamicRegionData> dynamic_regions;
    std::unordered_map<long long, std::shared_ptr<const ForcingRegionData>> forcing_regions;
    std::unordered_map<long long, AtlasData> atlases;
    std::unordered_map<long long, std::shared_ptr<const RegionPacketTemplateData>> packet_templates;
    std::unordered_map<long long, FluidWorldRuntime> brick_world_runtimes;
    long long next_internal_context_key = -1;
    int next_pending_static_brick_upload_id = 1;
    std::unordered_map<int, PendingStaticBrickUpload> pending_static_brick_uploads;
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
constexpr int k_face_count = 6;
constexpr int k_packet_channels = 11;
constexpr int k_channel_obstacle = 0;
constexpr int k_channel_fan_mask = 1;
constexpr int k_channel_fan_vx = 2;
constexpr int k_channel_fan_vy = 3;
constexpr int k_channel_fan_vz = 4;
constexpr int k_channel_state_vx = 5;
constexpr int k_channel_state_vy = 6;
constexpr int k_channel_state_vz = 7;
constexpr int k_channel_state_p = 8;
constexpr int k_channel_thermal_source = 9;
constexpr int k_channel_state_temp = 10;
constexpr int k_sparse_overlay_channels = 6;
constexpr int k_sparse_overlay_state_vx = 0;
constexpr int k_sparse_overlay_state_vy = 1;
constexpr int k_sparse_overlay_state_vz = 2;
constexpr int k_sparse_overlay_state_p = 3;
constexpr int k_sparse_overlay_state_temp = 4;
constexpr int k_sparse_overlay_thermal_source = 5;
constexpr int k_window_edge_stabilization_layers = 8;
constexpr float k_window_edge_stabilization_min_keep = 0.15f;
constexpr int k_nested_boundary_layers = 3;
constexpr float k_nested_boundary_min_keep = 0.25f;
constexpr float k_sponge_relaxation_max = 0.95f;
constexpr float k_runtime_temperature_scale_kelvin = 20.0f;
constexpr int k_tornado_descriptor_floats = 17;
constexpr int k_tornado_desc_center_x = 0;
constexpr int k_tornado_desc_center_y = 1;
constexpr int k_tornado_desc_center_z = 2;
constexpr int k_tornado_desc_translation_x = 3;
constexpr int k_tornado_desc_translation_z = 4;
constexpr int k_tornado_desc_core_radius = 5;
constexpr int k_tornado_desc_influence_radius = 6;
constexpr int k_tornado_desc_tangential_lattice = 7;
constexpr int k_tornado_desc_radial_lattice = 8;
constexpr int k_tornado_desc_updraft_lattice = 9;
constexpr int k_tornado_desc_condensation_bias = 10;
constexpr int k_tornado_desc_intensity = 11;
constexpr int k_tornado_desc_rotation_sign = 12;
constexpr int k_tornado_desc_state_ordinal = 13;
constexpr int k_tornado_desc_lifecycle_envelope = 14;
constexpr int k_nested_feedback_max_bins = 8;
constexpr int k_nested_feedback_layout_ints_per_bin = 9;
constexpr int k_nested_feedback_values_per_bin = 10;
constexpr int k_nested_feedback_status_fields = 6;
constexpr int k_nested_feedback_layout_cell_x = 0;
constexpr int k_nested_feedback_layout_layer = 1;
constexpr int k_nested_feedback_layout_cell_z = 2;
constexpr int k_nested_feedback_layout_min_x = 3;
constexpr int k_nested_feedback_layout_max_x = 4;
constexpr int k_nested_feedback_layout_min_y = 5;
constexpr int k_nested_feedback_layout_max_y = 6;
constexpr int k_nested_feedback_layout_min_z = 7;
constexpr int k_nested_feedback_layout_max_z = 8;
constexpr int k_nested_feedback_value_volume = 0;
constexpr int k_nested_feedback_value_density = 1;
constexpr int k_nested_feedback_value_momentum_x = 2;
constexpr int k_nested_feedback_value_momentum_z = 3;
constexpr int k_nested_feedback_value_air_temperature = 4;
constexpr int k_nested_feedback_value_surface_temperature = 5;
constexpr int k_nested_feedback_value_bottom_area = 6;
constexpr int k_nested_feedback_value_bottom_mass_flux = 7;
constexpr int k_nested_feedback_value_top_area = 8;
constexpr int k_nested_feedback_value_top_mass_flux = 9;
constexpr float k_tornado_lattice_speed_cap = 0.24f;
constexpr float k_cell_face_area_square_meters = 1.0f;
constexpr float k_cell_volume_cubic_meters = 1.0f;
constexpr float k_air_density_kg_per_cubic_meter = 1.225f;
constexpr float k_air_specific_heat_j_per_kg_k = 1005.0f;
constexpr float k_native_thermal_source_max = 0.006f;
constexpr float k_thermal_surface_init_min_k = 220.0f;
constexpr float k_thermal_surface_max_k = 1800.0f;
constexpr int k_brick_face_ghost_layers = 1;
constexpr std::uint64_t k_brick_inactive_retention_epochs = 16;
constexpr int k_brick_face_neighbor_count = 6;
constexpr int k_brick_face_neighbor_offsets[k_brick_face_neighbor_count][3] = {
    {-1, 0, 0}, {1, 0, 0},
    {0, -1, 0}, {0, 1, 0},
    {0, 0, -1}, {0, 0, 1}
};
constexpr int k_brick_face_neighbor_axes[k_brick_face_neighbor_count] = {0, 0, 1, 1, 2, 2};
constexpr bool k_brick_face_neighbor_positive_faces[k_brick_face_neighbor_count] = {
    false, true, false, true, false, true
};
constexpr uint8_t k_surface_kind_none = 0;
constexpr uint8_t k_surface_kind_rock = 1;
constexpr uint8_t k_surface_kind_soil = 2;
constexpr uint8_t k_surface_kind_vegetation = 3;
constexpr uint8_t k_surface_kind_snow_ice = 4;
constexpr uint8_t k_surface_kind_water = 5;
constexpr uint8_t k_surface_kind_molten = 6;
constexpr float k_thermal_stefan_boltzmann = 5.6703744e-8f;

struct SparsePacketOverlayBuilder {
    std::vector<int> lookup;
    std::vector<int> touched_lookup_slots;
    std::vector<int> cells;
    std::vector<float> values;

    void reset(int cell_count) {
        if (cell_count <= 0) {
            lookup.clear();
            touched_lookup_slots.clear();
            cells.clear();
            values.clear();
            return;
        }
        if (lookup.size() != static_cast<size_t>(cell_count)) {
            lookup.assign(static_cast<size_t>(cell_count), -1);
        } else {
            for (int cell : touched_lookup_slots) {
                if (cell >= 0 && cell < cell_count) {
                    lookup[static_cast<size_t>(cell)] = -1;
                }
            }
        }
        touched_lookup_slots.clear();
        cells.clear();
        values.clear();
    }

    float* touch(const float* base_packet, int cell) {
        if (!base_packet || cell < 0 || static_cast<size_t>(cell) >= lookup.size()) {
            return nullptr;
        }
        int& slot = lookup[static_cast<size_t>(cell)];
        if (slot < 0) {
            slot = static_cast<int>(cells.size());
            cells.push_back(cell);
            touched_lookup_slots.push_back(cell);
            values.resize(static_cast<size_t>(slot + 1) * k_sparse_overlay_channels);
            float* entry = values.data() + static_cast<size_t>(slot) * k_sparse_overlay_channels;
            const size_t base = static_cast<size_t>(cell) * k_packet_channels;
            entry[k_sparse_overlay_state_vx] = base_packet[base + k_channel_state_vx];
            entry[k_sparse_overlay_state_vy] = base_packet[base + k_channel_state_vy];
            entry[k_sparse_overlay_state_vz] = base_packet[base + k_channel_state_vz];
            entry[k_sparse_overlay_state_p] = base_packet[base + k_channel_state_p];
            entry[k_sparse_overlay_state_temp] = base_packet[base + k_channel_state_temp];
            entry[k_sparse_overlay_thermal_source] = base_packet[base + k_channel_thermal_source];
            return entry;
        }
        return values.data() + static_cast<size_t>(slot) * k_sparse_overlay_channels;
    }
};

struct ThermalMaterialProperties {
    float solar_absorptivity;
    float emissivity;
    float surface_heat_capacity_jm2k;
    float convective_exchange_coefficient_wm2k;
    float bulk_conductance_wm2k;
    float rain_exchange_coefficient_wm2k;
};

void set_simulation_last_error(const std::string& message) {
    g_simulation_last_error = message;
}

ServiceState* lookup_service(long long service_key) {
    if (!g_service || g_service_key == 0 || service_key != g_service_key) {
        return nullptr;
    }
    return g_service;
}

FluidWorldRuntime* ensure_brick_world_runtime(
    ServiceState& service,
    long long world_key,
    int brick_size,
    float dx_meters,
    float dt_seconds
) {
    if (world_key == 0) {
        set_simulation_last_error("simulation_brick_world_runtime: invalid world key");
        return nullptr;
    }
    if (brick_size <= 0 || !std::isfinite(dx_meters) || dx_meters <= 0.0f || !std::isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        set_simulation_last_error("simulation_brick_world_runtime: invalid runtime config");
        return nullptr;
    }
    auto [iterator, inserted] = service.brick_world_runtimes.try_emplace(world_key);
    FluidWorldRuntime& runtime = iterator->second;
    if (inserted || runtime.brick_size != brick_size || runtime.dx_meters != dx_meters || runtime.dt_seconds != dt_seconds) {
        runtime = FluidWorldRuntime{};
        runtime.brick_size = brick_size;
        runtime.dx_meters = dx_meters;
        runtime.dt_seconds = dt_seconds;
    }
    return &runtime;
}

int count_active_hint_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (entry.second.active_hint) {
            ++count;
        }
    }
    return count;
}

int count_active_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (entry.second.active) {
            ++count;
        }
    }
    return count;
}

int count_geometry_dirty_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (entry.second.geometry_dirty) {
            ++count;
        }
    }
    return count;
}

int count_forcing_dirty_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (entry.second.forcing_dirty) {
            ++count;
        }
    }
    return count;
}

int count_pending_reinit_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (entry.second.pending_reinit) {
            ++count;
        }
    }
    return count;
}

constexpr int k_world_delta_brick_static_cell_patch = 8;
constexpr int k_world_delta_brick_static_brick_upload = 9;

bool brick_is_resident_for_windows(const BrickData& brick) {
    return brick.active_hint
        || brick.active
        || brick.geometry_dirty
        || brick.forcing_dirty
        || brick.pending_reinit;
}

int count_resident_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (brick_is_resident_for_windows(entry.second)) {
            ++count;
        }
    }
    return count;
}

int count_active_window_bricks(const FluidWorldRuntime& runtime) {
    int count = 0;
    for (const auto& entry : runtime.bricks) {
        if (entry.second.active) {
            ++count;
        }
    }
    return count;
}

int copy_resident_bricks(const FluidWorldRuntime& runtime, int* out_coords, int brick_capacity) {
    if ((brick_capacity > 0) && !out_coords) {
        return -1;
    }
    int written = 0;
    for (const auto& entry : runtime.bricks) {
        if (!brick_is_resident_for_windows(entry.second)) {
            continue;
        }
        if (written >= brick_capacity) {
            return -1;
        }
        const int base = written * 3;
        out_coords[base] = entry.first.x;
        out_coords[base + 1] = entry.first.y;
        out_coords[base + 2] = entry.first.z;
        ++written;
    }
    return written;
}

int copy_active_bricks(const FluidWorldRuntime& runtime, int* out_coords, int brick_capacity) {
    if ((brick_capacity > 0) && !out_coords) {
        return -1;
    }
    int written = 0;
    for (const auto& entry : runtime.bricks) {
        if (!entry.second.active) {
            continue;
        }
        if (written >= brick_capacity) {
            return -1;
        }
        const int base = written * 3;
        out_coords[base] = entry.first.x;
        out_coords[base + 1] = entry.first.y;
        out_coords[base + 2] = entry.first.z;
        ++written;
    }
    return written;
}

int floor_div_int(int value, int divisor) {
    if (divisor <= 0) {
        return 0;
    }
    const int quotient = value / divisor;
    const int remainder = value % divisor;
    return (remainder != 0 && value < 0) ? (quotient - 1) : quotient;
}

BrickCoord brick_coord_for_block(int x, int y, int z, int brick_size) {
    return BrickCoord{
        floor_div_int(x, brick_size),
        floor_div_int(y, brick_size),
        floor_div_int(z, brick_size)
    };
}

void recompute_runtime_active_flags(FluidWorldRuntime& runtime) {
    for (auto& entry : runtime.bricks) {
        const bool hinted = runtime.active_hint_closure.find(entry.first) != runtime.active_hint_closure.end();
        entry.second.active = hinted
            || entry.second.geometry_dirty
            || entry.second.forcing_dirty
            || entry.second.pending_reinit;
        if (entry.second.active) {
            entry.second.last_active_epoch = runtime.epoch;
        }
    }
}

long long allocate_internal_context_key(ServiceState& service) {
    return service.next_internal_context_key--;
}

void release_brick_context(BrickData& brick) {
    if (brick.context_key != 0) {
        aero_lbm_release_context(brick.context_key);
        brick.context_key = 0;
    }
}

void prune_inactive_bricks(FluidWorldRuntime& runtime) {
    for (auto iterator = runtime.bricks.begin(); iterator != runtime.bricks.end();) {
        const BrickData& brick = iterator->second;
        const bool stale = !brick.active
            && !brick.geometry_dirty
            && !brick.forcing_dirty
            && !brick.pending_reinit
            && runtime.epoch > brick.last_active_epoch
            && (runtime.epoch - brick.last_active_epoch) > k_brick_inactive_retention_epochs;
        if (!stale) {
            ++iterator;
            continue;
        }
        release_brick_context(iterator->second);
        runtime.active_hint_closure.erase(iterator->first);
        iterator = runtime.bricks.erase(iterator);
    }
}

bool brick_dynamic_region_valid(const FluidWorldRuntime& runtime, const DynamicRegionData* dynamic);
void ensure_brick_dynamic_region_storage(FluidWorldRuntime& runtime, BrickData& brick);
void reset_brick_dynamic_region_state(FluidWorldRuntime& runtime, BrickData& brick);
void apply_brick_static_constraints(FluidWorldRuntime& runtime, BrickData& brick);
float temperature_source_from_power_watts(float thermal_power_watts);
struct BrickMeanState;
BrickMeanState compute_brick_mean_state(const DynamicRegionData& dynamic);
void seed_brick_from_neighbor_means(
    const FluidWorldRuntime& runtime,
    const BrickCoord& coord,
    const std::unordered_map<BrickCoord, std::shared_ptr<const DynamicRegionData>, BrickCoordHash>& snapshots,
    DynamicRegionData& dynamic
);
void copy_brick_face_ghost_from_neighbor(
    const FluidWorldRuntime& runtime,
    const DynamicRegionData& neighbor,
    int axis,
    bool positive_face,
    DynamicRegionData& dynamic
);
bool exchange_brick_face_context_halo(
    const FluidWorldRuntime& runtime,
    BrickData& first,
    BrickData& second,
    int offset_x,
    int offset_y,
    int offset_z
);
bool build_brick_step_packet(
    const FluidWorldRuntime& runtime,
    const BrickData& brick,
    const DynamicRegionData& dynamic,
    std::vector<float>& packet
);
bool sync_brick_dynamic_from_context(const FluidWorldRuntime& runtime, BrickData& brick);
bool step_brick_actual(
    ServiceState& service,
    FluidWorldRuntime& runtime,
    BrickData& brick,
    DynamicRegionData& dynamic
);
void apply_pending_world_deltas(ServiceState& service, long long world_key, FluidWorldRuntime& runtime);

void step_brick_world_runtime(ServiceState& service, long long world_key, FluidWorldRuntime& runtime, int step_count) {
    if (step_count <= 0) {
        return;
    }
    apply_pending_world_deltas(service, world_key, runtime);
    for (int step = 0; step < step_count; ++step) {
        runtime.epoch++;
        recompute_runtime_active_flags(runtime);
        std::unordered_map<BrickCoord, bool, BrickCoordHash> needs_seed;
        for (auto& entry : runtime.bricks) {
            BrickData& brick = entry.second;
            if (!brick.active) {
                continue;
            }
            const bool had_dynamic = brick.dynamic_region != nullptr;
            if ((brick.pending_reinit || !had_dynamic) && brick.context_key != 0) {
                release_brick_context(brick);
            }
            if ((brick.pending_reinit || !had_dynamic) && brick.static_region) {
                reset_brick_dynamic_region_state(runtime, brick);
                apply_brick_static_constraints(runtime, brick);
                needs_seed[entry.first] = true;
            } else if (brick.dynamic_region) {
                ensure_brick_dynamic_region_storage(runtime, brick);
            } else {
                ensure_brick_dynamic_region_storage(runtime, brick);
                needs_seed[entry.first] = true;
            }
        }

        std::unordered_map<BrickCoord, std::shared_ptr<const DynamicRegionData>, BrickCoordHash> snapshots;
        snapshots.reserve(runtime.bricks.size());
        for (const auto& entry : runtime.bricks) {
            if (!entry.second.active || !brick_dynamic_region_valid(runtime, entry.second.dynamic_region.get())) {
                continue;
            }
            snapshots.emplace(entry.first, entry.second.dynamic_region);
        }

        for (auto& entry : runtime.bricks) {
            BrickData& brick = entry.second;
            if (!brick.active || brick.context_key == 0) {
                continue;
            }
            for (int i = 0; i < k_brick_face_neighbor_count; ++i) {
                if (!k_brick_face_neighbor_positive_faces[i]) {
                    continue;
                }
                BrickCoord neighbor_coord{
                    entry.first.x + k_brick_face_neighbor_offsets[i][0],
                    entry.first.y + k_brick_face_neighbor_offsets[i][1],
                    entry.first.z + k_brick_face_neighbor_offsets[i][2]
                };
                auto neighbor_it = runtime.bricks.find(neighbor_coord);
                if (neighbor_it == runtime.bricks.end()) {
                    continue;
                }
                BrickData& neighbor = neighbor_it->second;
                if (!neighbor.active || neighbor.context_key == 0) {
                    continue;
                }
                exchange_brick_face_context_halo(
                    runtime,
                    brick,
                    neighbor,
                    k_brick_face_neighbor_offsets[i][0],
                    k_brick_face_neighbor_offsets[i][1],
                    k_brick_face_neighbor_offsets[i][2]
                );
            }
        }

        for (auto& entry : runtime.bricks) {
            BrickData& brick = entry.second;
            if (!brick.active || !brick.dynamic_region) {
                continue;
            }
            ensure_brick_dynamic_region_storage(runtime, brick);
            auto next_dynamic = std::make_shared<DynamicRegionData>(*brick.dynamic_region);
            const bool bootstrap_macro_ghost =
                needs_seed.find(entry.first) != needs_seed.end() || brick.context_key == 0;
            if (bootstrap_macro_ghost) {
                seed_brick_from_neighbor_means(runtime, entry.first, snapshots, *next_dynamic);
                for (int i = 0; i < k_brick_face_neighbor_count; ++i) {
                    BrickCoord neighbor_coord{
                        entry.first.x + k_brick_face_neighbor_offsets[i][0],
                        entry.first.y + k_brick_face_neighbor_offsets[i][1],
                        entry.first.z + k_brick_face_neighbor_offsets[i][2]
                    };
                    auto neighbor_it = snapshots.find(neighbor_coord);
                    if (neighbor_it == snapshots.end() || !neighbor_it->second) {
                        continue;
                    }
                    copy_brick_face_ghost_from_neighbor(
                        runtime,
                        *neighbor_it->second,
                        k_brick_face_neighbor_axes[i],
                        k_brick_face_neighbor_positive_faces[i],
                        *next_dynamic
                    );
                }
            }
            BrickData constrained_brick = brick;
            constrained_brick.dynamic_region = next_dynamic;
            apply_brick_static_constraints(runtime, constrained_brick);
            if (!step_brick_actual(service, runtime, constrained_brick, *constrained_brick.dynamic_region)) {
                brick.dynamic_region = std::move(constrained_brick.dynamic_region);
            } else {
                brick.context_key = constrained_brick.context_key;
                brick.packet_cache = std::move(constrained_brick.packet_cache);
                brick.dynamic_region = std::move(constrained_brick.dynamic_region);
            }
            if (brick.pending_reinit) {
                brick.pending_reinit = false;
            }
            if (brick.geometry_dirty) {
                brick.geometry_dirty = false;
            }
            if (brick.forcing_dirty) {
                brick.forcing_dirty = false;
            }
        }
        recompute_runtime_active_flags(runtime);
        prune_inactive_bricks(runtime);
    }
}

bool brick_dynamic_region_valid(const FluidWorldRuntime& runtime, const DynamicRegionData* dynamic) {
    if (!dynamic) {
        return false;
    }
    if (dynamic->nx != runtime.brick_size || dynamic->ny != runtime.brick_size || dynamic->nz != runtime.brick_size) {
        return false;
    }
    const int cells = runtime.brick_size > 0
        ? runtime.brick_size * runtime.brick_size * runtime.brick_size
        : 0;
    return cells > 0
        && dynamic->flow_state.size() == static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS
        && dynamic->air_temperature.size() == static_cast<size_t>(cells)
        && dynamic->surface_temperature.size() == static_cast<size_t>(cells);
}

void ensure_brick_dynamic_region_storage(FluidWorldRuntime& runtime, BrickData& brick) {
    if (brick_dynamic_region_valid(runtime, brick.dynamic_region.get())) {
        return;
    }
    const int cells = runtime.brick_size * runtime.brick_size * runtime.brick_size;
    auto dynamic = std::make_shared<DynamicRegionData>();
    dynamic->nx = runtime.brick_size;
    dynamic->ny = runtime.brick_size;
    dynamic->nz = runtime.brick_size;
    dynamic->flow_state.assign(static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS, 0.0f);
    dynamic->air_temperature.assign(static_cast<size_t>(cells), 0.0f);
    dynamic->surface_temperature.assign(static_cast<size_t>(cells), 0.0f);
    brick.dynamic_region = std::move(dynamic);
}

void reset_brick_dynamic_region_state(FluidWorldRuntime& runtime, BrickData& brick) {
    ensure_brick_dynamic_region_storage(runtime, brick);
    DynamicRegionData& dynamic = *brick.dynamic_region;
    std::fill(dynamic.flow_state.begin(), dynamic.flow_state.end(), 0.0f);
    std::fill(dynamic.air_temperature.begin(), dynamic.air_temperature.end(), 0.0f);
    std::fill(dynamic.surface_temperature.begin(), dynamic.surface_temperature.end(), 0.0f);
}

void apply_brick_static_constraints(FluidWorldRuntime& runtime, BrickData& brick) {
    if (!brick.static_region || !brick.dynamic_region) {
        return;
    }
    const StaticRegionData& stat = *brick.static_region;
    DynamicRegionData& dynamic = *brick.dynamic_region;
    if (!brick_dynamic_region_valid(runtime, &dynamic)
        || stat.nx != runtime.brick_size
        || stat.ny != runtime.brick_size
        || stat.nz != runtime.brick_size
        || stat.obstacle.size() != dynamic.air_temperature.size()
        || stat.surface_kind.size() != dynamic.air_temperature.size()
        || stat.open_face_mask.size() != dynamic.air_temperature.size()
        || stat.emitter_power_watts.size() != dynamic.air_temperature.size()) {
        return;
    }
    const size_t cells = dynamic.air_temperature.size();
    for (size_t cell = 0; cell < cells; ++cell) {
        const bool obstacle = stat.obstacle[cell] != 0;
        const size_t flow_base = cell * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        if (obstacle) {
            std::fill_n(
                dynamic.flow_state.begin() + static_cast<std::ptrdiff_t>(flow_base),
                AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS,
                0.0f
            );
            dynamic.air_temperature[cell] = 0.0f;
            dynamic.surface_temperature[cell] = 0.0f;
            continue;
        }
        if (stat.open_face_mask[cell] == 0
            && stat.surface_kind[cell] == 0
            && !(stat.emitter_power_watts[cell] > 0.0f)) {
            dynamic.surface_temperature[cell] = 0.0f;
        }
    }
}

struct BrickMeanState {
    float flow[AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
    float air_temperature = 0.0f;
    float surface_temperature = 0.0f;
    bool valid = false;
};

BrickMeanState compute_brick_mean_state(const DynamicRegionData& dynamic) {
    BrickMeanState mean;
    const size_t cells = dynamic.air_temperature.size();
    if (cells == 0 || dynamic.flow_state.size() != cells * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS) {
        return mean;
    }
    for (size_t cell = 0; cell < cells; ++cell) {
        const size_t base = cell * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        for (int channel = 0; channel < AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS; ++channel) {
            mean.flow[channel] += dynamic.flow_state[base + channel];
        }
        mean.air_temperature += dynamic.air_temperature[cell];
        mean.surface_temperature += dynamic.surface_temperature[cell];
    }
    const float inv_cells = 1.0f / static_cast<float>(cells);
    for (int channel = 0; channel < AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS; ++channel) {
        mean.flow[channel] *= inv_cells;
    }
    mean.air_temperature *= inv_cells;
    mean.surface_temperature *= inv_cells;
    mean.valid = true;
    return mean;
}

void seed_brick_from_neighbor_means(
    const FluidWorldRuntime& runtime,
    const BrickCoord& coord,
    const std::unordered_map<BrickCoord, std::shared_ptr<const DynamicRegionData>, BrickCoordHash>& snapshots,
    DynamicRegionData& dynamic
) {
    BrickMeanState accum;
    int contributing_neighbors = 0;
    for (const auto& offset : k_brick_face_neighbor_offsets) {
        BrickCoord neighbor_coord{coord.x + offset[0], coord.y + offset[1], coord.z + offset[2]};
        auto it = snapshots.find(neighbor_coord);
        if (it == snapshots.end() || !brick_dynamic_region_valid(runtime, it->second.get())) {
            continue;
        }
        BrickMeanState mean = compute_brick_mean_state(*it->second);
        if (!mean.valid) {
            continue;
        }
        for (int channel = 0; channel < AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS; ++channel) {
            accum.flow[channel] += mean.flow[channel];
        }
        accum.air_temperature += mean.air_temperature;
        accum.surface_temperature += mean.surface_temperature;
        contributing_neighbors++;
    }
    if (contributing_neighbors <= 0) {
        return;
    }
    const float inv_neighbors = 1.0f / static_cast<float>(contributing_neighbors);
    for (int channel = 0; channel < AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS; ++channel) {
        accum.flow[channel] *= inv_neighbors;
    }
    accum.air_temperature *= inv_neighbors;
    accum.surface_temperature *= inv_neighbors;
    const size_t cells = dynamic.air_temperature.size();
    for (size_t cell = 0; cell < cells; ++cell) {
        const size_t base = cell * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        for (int channel = 0; channel < AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS; ++channel) {
            dynamic.flow_state[base + channel] = accum.flow[channel];
        }
        dynamic.air_temperature[cell] = accum.air_temperature;
        dynamic.surface_temperature[cell] = accum.surface_temperature;
    }
}

void copy_brick_face_ghost_from_neighbor(
    const FluidWorldRuntime& runtime,
    const DynamicRegionData& neighbor,
    int axis,
    bool positive_face,
    DynamicRegionData& dynamic
) {
    if (!brick_dynamic_region_valid(runtime, &dynamic) || !brick_dynamic_region_valid(runtime, &neighbor)) {
        return;
    }
    const int size = runtime.brick_size;
    const int clamped_layers = std::max(1, std::min(k_brick_face_ghost_layers, size));
    auto cell_index = [size](int x, int y, int z) {
        return (x * size + y) * size + z;
    };
    auto copy_cell = [&](int dst_x, int dst_y, int dst_z, int src_x, int src_y, int src_z) {
        const int dst_cell = cell_index(dst_x, dst_y, dst_z);
        const int src_cell = cell_index(src_x, src_y, src_z);
        const size_t dst_base = static_cast<size_t>(dst_cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        const size_t src_base = static_cast<size_t>(src_cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        for (int channel = 0; channel < AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS; ++channel) {
            dynamic.flow_state[dst_base + channel] = neighbor.flow_state[src_base + channel];
        }
        dynamic.air_temperature[static_cast<size_t>(dst_cell)] =
            neighbor.air_temperature[static_cast<size_t>(src_cell)];
        dynamic.surface_temperature[static_cast<size_t>(dst_cell)] =
            neighbor.surface_temperature[static_cast<size_t>(src_cell)];
    };

    for (int layer = 0; layer < clamped_layers; ++layer) {
        const int dst_face = positive_face ? size - 1 - layer : layer;
        const int src_face = positive_face ? layer : size - 1 - layer;
        if (axis == 0) {
            for (int y = 0; y < size; ++y) {
                for (int z = 0; z < size; ++z) {
                    copy_cell(dst_face, y, z, src_face, y, z);
                }
            }
        } else if (axis == 1) {
            for (int x = 0; x < size; ++x) {
                for (int z = 0; z < size; ++z) {
                    copy_cell(x, dst_face, z, x, src_face, z);
                }
            }
        } else {
            for (int x = 0; x < size; ++x) {
                for (int y = 0; y < size; ++y) {
                    copy_cell(x, y, dst_face, x, y, src_face);
                }
            }
        }
    }
}

bool exchange_brick_face_context_halo(
    const FluidWorldRuntime& runtime,
    BrickData& first,
    BrickData& second,
    int offset_x,
    int offset_y,
    int offset_z
) {
    if (runtime.brick_size <= 2 * k_brick_face_ghost_layers
        || first.context_key == 0
        || second.context_key == 0) {
        return false;
    }
    return aero_lbm_exchange_halo_layers_rect(
        runtime.brick_size,
        runtime.brick_size,
        runtime.brick_size,
        k_brick_face_ghost_layers,
        first.context_key,
        second.context_key,
        offset_x,
        offset_y,
        offset_z
    ) != 0;
}

bool build_brick_step_packet(
    const FluidWorldRuntime& runtime,
    const BrickData& brick,
    const DynamicRegionData& dynamic,
    std::vector<float>& packet
) {
    if (!brick.static_region || !brick_dynamic_region_valid(runtime, &dynamic)) {
        return false;
    }
    const StaticRegionData& stat = *brick.static_region;
    if (stat.nx != runtime.brick_size
        || stat.ny != runtime.brick_size
        || stat.nz != runtime.brick_size
        || stat.obstacle.size() != dynamic.air_temperature.size()
        || stat.surface_kind.size() != dynamic.air_temperature.size()
        || stat.open_face_mask.size() != dynamic.air_temperature.size()
        || stat.emitter_power_watts.size() != dynamic.air_temperature.size()) {
        return false;
    }
    const size_t cells = dynamic.air_temperature.size();
    const size_t packet_values = cells * k_packet_channels;
    if (packet.size() != packet_values) {
        packet.assign(packet_values, 0.0f);
    } else {
        std::fill(packet.begin(), packet.end(), 0.0f);
    }
    for (size_t cell = 0; cell < cells; ++cell) {
        const size_t packet_base = cell * k_packet_channels;
        const size_t flow_base = cell * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        const bool obstacle = stat.obstacle[cell] != 0;
        packet[packet_base + k_channel_obstacle] = obstacle ? 1.0f : 0.0f;
        packet[packet_base + k_channel_fan_mask] = 0.0f;
        packet[packet_base + k_channel_fan_vx] = 0.0f;
        packet[packet_base + k_channel_fan_vy] = 0.0f;
        packet[packet_base + k_channel_fan_vz] = 0.0f;
        if (!obstacle) {
            packet[packet_base + k_channel_state_vx] = dynamic.flow_state[flow_base];
            packet[packet_base + k_channel_state_vy] = dynamic.flow_state[flow_base + 1];
            packet[packet_base + k_channel_state_vz] = dynamic.flow_state[flow_base + 2];
            packet[packet_base + k_channel_state_p] = dynamic.flow_state[flow_base + 3];
            packet[packet_base + k_channel_state_temp] = dynamic.air_temperature[cell];
            if (stat.emitter_power_watts[cell] > 0.0f) {
                packet[packet_base + k_channel_thermal_source] =
                    temperature_source_from_power_watts(stat.emitter_power_watts[cell]);
            }
        }
    }
    return true;
}

bool sync_brick_dynamic_from_context(const FluidWorldRuntime& runtime, BrickData& brick) {
    if (!brick.dynamic_region || brick.context_key == 0) {
        return false;
    }
    DynamicRegionData& dynamic = *brick.dynamic_region;
    if (!brick_dynamic_region_valid(runtime, &dynamic)) {
        return false;
    }
    const int size = runtime.brick_size;
    if (!aero_lbm_get_flow_state_rect(size, size, size, brick.context_key, dynamic.flow_state.data())) {
        return false;
    }
    if (!aero_lbm_get_temperature_state_rect(size, size, size, brick.context_key, dynamic.air_temperature.data())) {
        return false;
    }
    return true;
}

bool step_brick_actual(
    ServiceState& service,
    FluidWorldRuntime& runtime,
    BrickData& brick,
    DynamicRegionData& dynamic
) {
    if (!build_brick_step_packet(runtime, brick, dynamic, brick.packet_cache)) {
        return false;
    }
    if (brick.context_key == 0) {
        brick.context_key = allocate_internal_context_key(service);
    }
    const int size = runtime.brick_size;
    if (!aero_lbm_step_rect(brick.packet_cache.data(), size, size, size, brick.context_key, nullptr)) {
        release_brick_context(brick);
        return false;
    }
    if (!sync_brick_dynamic_from_context(runtime, brick)) {
        release_brick_context(brick);
        return false;
    }
    apply_brick_static_constraints(runtime, brick);
    return true;
}

void mark_brick_state(
    FluidWorldRuntime& runtime,
    const BrickCoord& coord,
    bool geometry_dirty,
    bool forcing_dirty,
    bool pending_reinit
) {
    BrickData& brick = runtime.bricks[coord];
    brick.geometry_dirty = brick.geometry_dirty || geometry_dirty;
    brick.forcing_dirty = brick.forcing_dirty || forcing_dirty;
    brick.pending_reinit = brick.pending_reinit || pending_reinit;
    brick.active = true;
    brick.last_active_epoch = runtime.epoch;
}

void mark_brick_and_neighbor_reinit(
    FluidWorldRuntime& runtime,
    const BrickCoord& center,
    bool geometry_dirty,
    bool forcing_dirty
) {
    mark_brick_state(runtime, center, geometry_dirty, forcing_dirty, true);
    for (const auto& offset : k_brick_face_neighbor_offsets) {
        mark_brick_state(
            runtime,
            BrickCoord{center.x + offset[0], center.y + offset[1], center.z + offset[2]},
            false,
            false,
            true
        );
    }
}

void apply_brick_world_delta(FluidWorldRuntime& runtime, const AeroLbmWorldDelta& delta) {
    const BrickCoord coord = brick_coord_for_block(delta.x, delta.y, delta.z, runtime.brick_size);
    switch (delta.type) {
        case k_world_delta_brick_static_cell_patch: {
            auto brick_it = runtime.bricks.find(coord);
            if (brick_it == runtime.bricks.end()) {
                break;
            }
            BrickData& brick = brick_it->second;
            if (!brick.static_region
                || brick.static_region->nx != runtime.brick_size
                || brick.static_region->ny != runtime.brick_size
                || brick.static_region->nz != runtime.brick_size) {
                break;
            }
            const int local_x = delta.x - coord.x * runtime.brick_size;
            const int local_y = delta.y - coord.y * runtime.brick_size;
            const int local_z = delta.z - coord.z * runtime.brick_size;
            if (local_x < 0 || local_y < 0 || local_z < 0
                || local_x >= runtime.brick_size
                || local_y >= runtime.brick_size
                || local_z >= runtime.brick_size) {
                break;
            }
            const int cell = (local_x * runtime.brick_size + local_y) * runtime.brick_size + local_z;
            auto updated_static = std::make_shared<StaticRegionData>(*brick.static_region);
            StaticRegionData& stat = *updated_static;
            stat.obstacle[static_cast<size_t>(cell)] = (delta.data1 & 0x1) != 0 ? 1u : 0u;
            stat.surface_kind[static_cast<size_t>(cell)] = static_cast<uint8_t>((delta.data1 >> 8) & 0xFF);
            stat.open_face_mask[static_cast<size_t>(cell)] = static_cast<uint16_t>(delta.data2 & 0xFF);
            stat.emitter_power_watts[static_cast<size_t>(cell)] = std::isfinite(delta.value0) ? delta.value0 : 0.0f;
            const size_t face_base = static_cast<size_t>(cell) * k_face_count;
            std::fill_n(stat.face_sky_exposure.begin() + static_cast<std::ptrdiff_t>(face_base), k_face_count, 0u);
            std::fill_n(stat.face_direct_exposure.begin() + static_cast<std::ptrdiff_t>(face_base), k_face_count, 0u);
            brick.static_region = std::move(updated_static);
            if (brick.dynamic_region) {
                apply_brick_static_constraints(runtime, brick);
            }
            break;
        }
        case 1:
        case 2:
        case 3:
            mark_brick_and_neighbor_reinit(runtime, coord, true, false);
            break;
        case 4:
        case 5:
            mark_brick_and_neighbor_reinit(runtime, coord, false, true);
            break;
        default:
            break;
    }
}

void apply_pending_world_deltas(ServiceState& service, long long world_key, FluidWorldRuntime& runtime) {
    if (runtime.pending_world_deltas.empty()) {
        return;
    }
    std::vector<AeroLbmWorldDelta> pending;
    pending.swap(runtime.pending_world_deltas);
    std::vector<int> consumed_upload_ids;
    consumed_upload_ids.reserve(pending.size());
    bool touched_runtime_state = false;
    for (const AeroLbmWorldDelta& delta : pending) {
        if (delta.type == k_world_delta_brick_static_brick_upload) {
            auto upload_it = service.pending_static_brick_uploads.find(delta.data1);
            if (upload_it == service.pending_static_brick_uploads.end()) {
                continue;
            }
            const PendingStaticBrickUpload& upload = upload_it->second;
            if (upload.world_key != world_key || !upload.static_region) {
                consumed_upload_ids.push_back(delta.data1);
                continue;
            }
            BrickData& brick = runtime.bricks[upload.coord];
            brick.static_region = upload.static_region;
            brick.geometry_dirty = true;
            brick.pending_reinit = true;
            brick.active = true;
            brick.last_active_epoch = runtime.epoch;
            consumed_upload_ids.push_back(delta.data1);
            touched_runtime_state = true;
            continue;
        }
        apply_brick_world_delta(runtime, delta);
        touched_runtime_state = true;
    }
    for (int upload_id : consumed_upload_ids) {
        service.pending_static_brick_uploads.erase(upload_id);
    }
    if (touched_runtime_state) {
        recompute_runtime_active_flags(runtime);
        prune_inactive_bricks(runtime);
    }
}

void apply_world_delta_to_brick_runtimes(ServiceState& service, const AeroLbmWorldDelta& delta) {
    long long world_key = static_cast<long long>(delta.data0);
    if (world_key == 0) {
        world_key = 1;
    }
    if (delta.type == 6) {
        service.brick_world_runtimes.erase(world_key);
        return;
    }
    auto runtime_iterator = service.brick_world_runtimes.find(world_key);
    if (runtime_iterator == service.brick_world_runtimes.end()) {
        return;
    }
    runtime_iterator->second.pending_world_deltas.push_back(delta);
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

void rebuild_region_packet_template(ServiceState& service, long long region_key);
int grid_cell_index(int ny, int nz, int x, int y, int z);
bool region_core_bounds_valid(
    int region_nx,
    int region_ny,
    int region_nz,
    int core_offset_x,
    int core_offset_y,
    int core_offset_z,
    int core_nx,
    int core_ny,
    int core_nz
) {
    return core_offset_x >= 0
        && core_offset_y >= 0
        && core_offset_z >= 0
        && core_nx > 0
        && core_ny > 0
        && core_nz > 0
        && core_offset_x + core_nx <= region_nx
        && core_offset_y + core_ny <= region_ny
        && core_offset_z + core_nz <= region_nz;
}

void clear_nested_feedback_accumulators(DynamicRegionData::NestedFeedbackData& nested_feedback) {
    nested_feedback.steps_accumulated = 0;
    nested_feedback.accumulators.assign(
        nested_feedback.layout.size(),
        DynamicRegionData::NestedFeedbackBinAccumulator{}
    );
}

void reset_nested_feedback_progress(DynamicRegionData::NestedFeedbackData& nested_feedback) {
    nested_feedback.ready_values.clear();
    nested_feedback.reset_count++;
    clear_nested_feedback_accumulators(nested_feedback);
}

bool configure_nested_feedback_layout(
    DynamicRegionData& dynamic,
    int steps_per_feedback,
    const int* layout_values,
    int value_count
) {
    if (steps_per_feedback <= 0 || !layout_values || value_count <= 0
        || (value_count % k_nested_feedback_layout_ints_per_bin) != 0
        || value_count > k_nested_feedback_max_bins * k_nested_feedback_layout_ints_per_bin) {
        set_simulation_last_error("simulation_nested_feedback_layout: invalid layout arguments");
        return false;
    }
    const int bin_count = value_count / k_nested_feedback_layout_ints_per_bin;
    std::vector<DynamicRegionData::NestedFeedbackBinLayout> layout;
    layout.reserve(static_cast<std::size_t>(bin_count));
    for (int index = 0; index < bin_count; ++index) {
        const int base = index * k_nested_feedback_layout_ints_per_bin;
        DynamicRegionData::NestedFeedbackBinLayout bin{};
        bin.cell_x = layout_values[base + k_nested_feedback_layout_cell_x];
        bin.layer = layout_values[base + k_nested_feedback_layout_layer];
        bin.cell_z = layout_values[base + k_nested_feedback_layout_cell_z];
        bin.min_x = layout_values[base + k_nested_feedback_layout_min_x];
        bin.max_x = layout_values[base + k_nested_feedback_layout_max_x];
        bin.min_y = layout_values[base + k_nested_feedback_layout_min_y];
        bin.max_y = layout_values[base + k_nested_feedback_layout_max_y];
        bin.min_z = layout_values[base + k_nested_feedback_layout_min_z];
        bin.max_z = layout_values[base + k_nested_feedback_layout_max_z];
        if (bin.min_x < 0 || bin.min_y < 0 || bin.min_z < 0
            || bin.max_x > dynamic.nx || bin.max_y > dynamic.ny || bin.max_z > dynamic.nz
            || bin.min_x >= bin.max_x || bin.min_y >= bin.max_y || bin.min_z >= bin.max_z) {
            set_simulation_last_error("simulation_nested_feedback_layout: layout bounds out of range");
            return false;
        }
        layout.push_back(bin);
    }
    dynamic.nested_feedback.steps_per_feedback = steps_per_feedback;
    dynamic.nested_feedback.layout = std::move(layout);
    reset_nested_feedback_progress(dynamic.nested_feedback);
    return true;
}

void accumulate_nested_feedback(
    const StaticRegionData* stat,
    DynamicRegionData& dynamic
) {
    DynamicRegionData::NestedFeedbackData& nested_feedback = dynamic.nested_feedback;
    if (!stat
        || nested_feedback.layout.empty()
        || nested_feedback.steps_per_feedback <= 0
        || !nested_feedback.ready_values.empty()) {
        return;
    }
    if (dynamic.flow_state.empty()
        || dynamic.air_temperature.empty()
        || dynamic.surface_temperature.empty()) {
        return;
    }

    for (std::size_t index = 0; index < nested_feedback.layout.size(); ++index) {
        const auto& bin = nested_feedback.layout[index];
        auto& accumulator = nested_feedback.accumulators[index];
        for (int x = bin.min_x; x < bin.max_x; ++x) {
            for (int y = bin.min_y; y < bin.max_y; ++y) {
                for (int z = bin.min_z; z < bin.max_z; ++z) {
                    const int cell = grid_cell_index(dynamic.ny, dynamic.nz, x, y, z);
                    if (stat->obstacle[static_cast<std::size_t>(cell)] != 0) {
                        continue;
                    }
                    const std::size_t flow_base = static_cast<std::size_t>(cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
                    const float vx = dynamic.flow_state[flow_base];
                    const float vy = dynamic.flow_state[flow_base + 1];
                    const float vz = dynamic.flow_state[flow_base + 2];
                    const float pressure = dynamic.flow_state[flow_base + 3];
                    const float air_temperature = dynamic.air_temperature[static_cast<std::size_t>(cell)];
                    const float surface_temperature = dynamic.surface_temperature[static_cast<std::size_t>(cell)];
                    if (!std::isfinite(vx)
                        || !std::isfinite(vy)
                        || !std::isfinite(vz)
                        || !std::isfinite(pressure)
                        || !std::isfinite(air_temperature)
                        || !std::isfinite(surface_temperature)) {
                        continue;
                    }
                    const float rho = std::max(1.0e-6f, 1.0f + pressure);
                    accumulator.volume_sum += k_cell_volume_cubic_meters;
                    accumulator.density_sum += rho;
                    accumulator.momentum_x_sum += rho * vx;
                    accumulator.momentum_z_sum += rho * vz;
                    accumulator.air_temperature_volume_sum += air_temperature;
                    accumulator.surface_temperature_volume_sum += surface_temperature;
                    if (y == bin.min_y) {
                        accumulator.bottom_area_sum += k_cell_face_area_square_meters;
                        accumulator.bottom_mass_flux_sum += rho * vy;
                    }
                    if (y == bin.max_y - 1) {
                        accumulator.top_area_sum += k_cell_face_area_square_meters;
                        accumulator.top_mass_flux_sum += rho * vy;
                    }
                }
            }
        }
    }

    nested_feedback.steps_accumulated++;
    if (nested_feedback.steps_accumulated < nested_feedback.steps_per_feedback) {
        return;
    }

    nested_feedback.ready_values.assign(
        nested_feedback.layout.size() * k_nested_feedback_values_per_bin,
        0.0f
    );
    for (std::size_t index = 0; index < nested_feedback.layout.size(); ++index) {
        const auto& accumulator = nested_feedback.accumulators[index];
        const std::size_t base = index * k_nested_feedback_values_per_bin;
        nested_feedback.ready_values[base + k_nested_feedback_value_volume] =
            static_cast<float>(accumulator.volume_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_density] =
            static_cast<float>(accumulator.density_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_momentum_x] =
            static_cast<float>(accumulator.momentum_x_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_momentum_z] =
            static_cast<float>(accumulator.momentum_z_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_air_temperature] =
            static_cast<float>(accumulator.air_temperature_volume_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_surface_temperature] =
            static_cast<float>(accumulator.surface_temperature_volume_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_bottom_area] =
            static_cast<float>(accumulator.bottom_area_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_bottom_mass_flux] =
            static_cast<float>(accumulator.bottom_mass_flux_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_top_area] =
            static_cast<float>(accumulator.top_area_sum);
        nested_feedback.ready_values[base + k_nested_feedback_value_top_mass_flux] =
            static_cast<float>(accumulator.top_mass_flux_sum);
    }
    nested_feedback.packets_emitted++;
    clear_nested_feedback_accumulators(nested_feedback);
}

void ensure_region_buffers(ServiceState& service, long long region_key, int nx, int ny, int nz, int cells) {
    DynamicRegionData& dynamic = service.dynamic_regions[region_key];
    if (dynamic.nx != nx || dynamic.ny != ny || dynamic.nz != nz
        || dynamic.flow_state.size() != static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS
        || dynamic.air_temperature.size() != static_cast<size_t>(cells)
        || dynamic.surface_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.nx = nx;
        dynamic.ny = ny;
        dynamic.nz = nz;
        dynamic.flow_state.assign(static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS, 0.0f);
        dynamic.air_temperature.assign(static_cast<size_t>(cells), 0.0f);
        dynamic.surface_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }

    auto forcing_it = service.forcing_regions.find(region_key);
    if (forcing_it == service.forcing_regions.end()
        || !forcing_it->second
        || forcing_it->second->nx != nx || forcing_it->second->ny != ny || forcing_it->second->nz != nz
        || forcing_it->second->thermal_source.size() != static_cast<size_t>(cells)
        || forcing_it->second->fan_mask.size() != static_cast<size_t>(cells)
        || forcing_it->second->fan_vx.size() != static_cast<size_t>(cells)
        || forcing_it->second->fan_vy.size() != static_cast<size_t>(cells)
        || forcing_it->second->fan_vz.size() != static_cast<size_t>(cells)) {
        auto forcing = std::make_shared<ForcingRegionData>();
        forcing->nx = nx;
        forcing->ny = ny;
        forcing->nz = nz;
        forcing->thermal_source.assign(static_cast<size_t>(cells), 0.0f);
        forcing->fan_mask.assign(static_cast<size_t>(cells), 0);
        forcing->fan_vx.assign(static_cast<size_t>(cells), 0.0f);
        forcing->fan_vy.assign(static_cast<size_t>(cells), 0.0f);
        forcing->fan_vz.assign(static_cast<size_t>(cells), 0.0f);
        service.forcing_regions[region_key] = std::move(forcing);
    }
    rebuild_region_packet_template(service, region_key);
}

std::shared_ptr<const RegionPacketTemplateData> build_region_packet_template(
    const StaticRegionData& stat,
    const ForcingRegionData& forcing,
    int nx,
    int ny,
    int nz
) {
    if (stat.nx != nx || stat.ny != ny || stat.nz != nz
        || forcing.nx != nx || forcing.ny != ny || forcing.nz != nz) {
        set_simulation_last_error("simulation_packet_template: region dimension mismatch");
        return nullptr;
    }
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        set_simulation_last_error("simulation_packet_template: invalid dimensions");
        return nullptr;
    }
    auto packet = std::make_shared<RegionPacketTemplateData>();
    packet->nx = nx;
    packet->ny = ny;
    packet->nz = nz;
    packet->values.assign(static_cast<size_t>(cells) * k_packet_channels, 0.0f);
    for (int cell = 0; cell < cells; ++cell) {
        size_t packet_base = static_cast<size_t>(cell) * k_packet_channels;
        const bool obstacle = stat.obstacle[static_cast<size_t>(cell)] != 0;
        packet->values[packet_base + k_channel_obstacle] = obstacle ? 1.0f : 0.0f;
        if (obstacle) {
            continue;
        }
        packet->values[packet_base + k_channel_fan_mask] = forcing.fan_mask[static_cast<size_t>(cell)] != 0 ? 1.0f : 0.0f;
        packet->values[packet_base + k_channel_fan_vx] = forcing.fan_vx[static_cast<size_t>(cell)];
        packet->values[packet_base + k_channel_fan_vy] = forcing.fan_vy[static_cast<size_t>(cell)];
        packet->values[packet_base + k_channel_fan_vz] = forcing.fan_vz[static_cast<size_t>(cell)];
        packet->values[packet_base + k_channel_thermal_source] = forcing.thermal_source[static_cast<size_t>(cell)];
    }
    return packet;
}

void rebuild_region_packet_template(ServiceState& service, long long region_key) {
    auto static_it = service.static_regions.find(region_key);
    auto forcing_it = service.forcing_regions.find(region_key);
    if (static_it == service.static_regions.end() || forcing_it == service.forcing_regions.end()) {
        service.packet_templates.erase(region_key);
        return;
    }
    const StaticRegionData& stat = *static_it->second;
    const ForcingRegionData& forcing = *forcing_it->second;
    std::shared_ptr<const RegionPacketTemplateData> packet = build_region_packet_template(
        stat,
        forcing,
        stat.nx,
        stat.ny,
        stat.nz
    );
    if (packet) {
        service.packet_templates[region_key] = std::move(packet);
    } else {
        service.packet_templates.erase(region_key);
    }
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

int local_face_index(int cell, int direction_index) {
    return cell * k_face_count + direction_index;
}

bool has_face_bit(uint16_t mask, int direction_index) {
    return (mask & (1u << direction_index)) != 0;
}

int count_open_faces(uint16_t mask) {
    int count = 0;
    for (int direction = 0; direction < k_face_count; ++direction) {
        if (has_face_bit(mask, direction)) {
            ++count;
        }
    }
    return count;
}

float dequantize_unit_float(uint8_t value) {
    return value / 255.0f;
}

float temperature_source_from_power_watts(float thermal_power_watts) {
    const float scalar = thermal_power_watts
        / (k_air_density_kg_per_cubic_meter * k_air_specific_heat_j_per_kg_k
            * k_cell_volume_cubic_meters * k_runtime_temperature_scale_kelvin);
    return std::clamp(scalar, -k_native_thermal_source_max, k_native_thermal_source_max);
}

float temperature_source_from_surface_flux(float heat_flux_watts_per_square_meter) {
    const float scalar = heat_flux_watts_per_square_meter * k_cell_face_area_square_meters
        / (k_air_density_kg_per_cubic_meter * k_air_specific_heat_j_per_kg_k
            * k_cell_volume_cubic_meters * k_runtime_temperature_scale_kelvin);
    return std::clamp(scalar, -k_native_thermal_source_max, k_native_thermal_source_max);
}

const ThermalMaterialProperties* thermal_material_properties(uint8_t kind) {
    static constexpr ThermalMaterialProperties rock{0.78f, 0.92f, 1.60e5f, 8.0f, 2.4f, 20.0f};
    static constexpr ThermalMaterialProperties soil{0.88f, 0.94f, 1.35e5f, 7.0f, 1.7f, 24.0f};
    static constexpr ThermalMaterialProperties vegetation{0.64f, 0.96f, 1.90e5f, 9.0f, 1.2f, 32.0f};
    static constexpr ThermalMaterialProperties snow_ice{0.22f, 0.98f, 2.40e5f, 6.0f, 1.0f, 18.0f};
    static constexpr ThermalMaterialProperties water{0.93f, 0.96f, 1.00e6f, 10.0f, 0.6f, 40.0f};
    static constexpr ThermalMaterialProperties molten{0.95f, 0.95f, 3.50e5f, 14.0f, 4.0f, 18.0f};
    switch (kind) {
        case k_surface_kind_rock: return &rock;
        case k_surface_kind_soil: return &soil;
        case k_surface_kind_vegetation: return &vegetation;
        case k_surface_kind_snow_ice: return &snow_ice;
        case k_surface_kind_water: return &water;
        case k_surface_kind_molten: return &molten;
        default: return nullptr;
    }
}

bool in_bounds(int nx, int ny, int nz, int x, int y, int z) {
    return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
}

int grid_cell_index(int ny, int nz, int x, int y, int z) {
    return (x * ny + y) * nz + z;
}

int boundary_face_index(int face, int u, int v, int resolution) {
    return (face * resolution + u) * resolution + v;
}

float sample_boundary_face_scalar(
    const float* values,
    int face,
    int resolution,
    double u,
    double v
) {
    if (!values || resolution <= 0) {
        return 0.0f;
    }
    double clamped_u = std::clamp(u, 0.0, static_cast<double>(resolution - 1));
    double clamped_v = std::clamp(v, 0.0, static_cast<double>(resolution - 1));
    int u0 = static_cast<int>(std::floor(clamped_u));
    int v0 = static_cast<int>(std::floor(clamped_v));
    int u1 = std::min(resolution - 1, u0 + 1);
    int v1 = std::min(resolution - 1, v0 + 1);
    float fu = static_cast<float>(clamped_u - u0);
    float fv = static_cast<float>(clamped_v - v0);
    float c00 = values[boundary_face_index(face, u0, v0, resolution)];
    float c10 = values[boundary_face_index(face, u1, v0, resolution)];
    float c01 = values[boundary_face_index(face, u0, v1, resolution)];
    float c11 = values[boundary_face_index(face, u1, v1, resolution)];
    float c0 = c00 + (c10 - c00) * fu;
    float c1 = c01 + (c11 - c01) * fu;
    return c0 + (c1 - c0) * fv;
}

bool use_boundary_face_sample(
    int external_face_mask,
    int face,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k
) {
    return boundary_face_resolution > 0
        && (external_face_mask & (1 << face)) != 0
        && boundary_wind_face_x
        && boundary_wind_face_y
        && boundary_wind_face_z
        && boundary_air_temperature_k;
}

void sample_boundary_face_target(
    int face,
    int boundary_face_resolution,
    int external_face_mask,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k,
    float fallback_boundary_wind_x,
    float fallback_boundary_wind_y,
    float fallback_boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    double u,
    double v,
    float& out_vx,
    float& out_vy,
    float& out_vz,
    float& out_air_k
) {
    const bool use_face = use_boundary_face_sample(
        external_face_mask,
        face,
        boundary_face_resolution,
        boundary_wind_face_x,
        boundary_wind_face_y,
        boundary_wind_face_z,
        boundary_air_temperature_k
    );
    out_vx = use_face
        ? sample_boundary_face_scalar(boundary_wind_face_x, face, boundary_face_resolution, u, v)
        : fallback_boundary_wind_x;
    out_vy = use_face
        ? sample_boundary_face_scalar(boundary_wind_face_y, face, boundary_face_resolution, u, v)
        : fallback_boundary_wind_y;
    out_vz = use_face
        ? sample_boundary_face_scalar(boundary_wind_face_z, face, boundary_face_resolution, u, v)
        : fallback_boundary_wind_z;
    out_air_k = use_face
        ? sample_boundary_face_scalar(boundary_air_temperature_k, face, boundary_face_resolution, u, v)
        : fallback_boundary_air_temperature_k;
}

float neighbor_air_temperature_kelvin(
    const DynamicRegionData& dynamic,
    int x,
    int y,
    int z,
    float ambient_air_temperature_kelvin
) {
    if (!in_bounds(dynamic.nx, dynamic.ny, dynamic.nz, x, y, z)
        || dynamic.air_temperature.empty()) {
        return ambient_air_temperature_kelvin;
    }
    return ambient_air_temperature_kelvin
        + dynamic.air_temperature[grid_cell_index(dynamic.ny, dynamic.nz, x, y, z)] * k_runtime_temperature_scale_kelvin;
}

float initialize_surface_temperature_kelvin(
    const ThermalMaterialProperties& material,
    float ambient_air_temperature_kelvin,
    float deep_ground_temperature_kelvin,
    float emitter_power_watts,
    int open_faces
) {
    const float exposed_area = std::max(1, open_faces) * k_cell_face_area_square_meters;
    const float ambient = 0.70f * ambient_air_temperature_kelvin + 0.30f * deep_ground_temperature_kelvin;
    const float denominator = std::max(
        1.0f,
        material.convective_exchange_coefficient_wm2k * exposed_area
            + material.bulk_conductance_wm2k * exposed_area
    );
    return std::clamp(
        ambient + emitter_power_watts / denominator,
        k_thermal_surface_init_min_k,
        k_thermal_surface_max_k
    );
}

void add_thermal_source(std::vector<float>& thermal_source, int cell, float source) {
    if (cell < 0 || static_cast<size_t>(cell) >= thermal_source.size() || source == 0.0f) {
        return;
    }
    thermal_source[static_cast<size_t>(cell)] = std::clamp(
        thermal_source[static_cast<size_t>(cell)] + source,
        -k_native_thermal_source_max,
        k_native_thermal_source_max
    );
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

void rebuild_default_packed_atlas(const DynamicRegionData& region, AtlasData& atlas) {
    if (region.nx <= 0 || region.ny <= 0 || region.nz <= 0 || region.flow_state.empty()) {
        atlas.values.clear();
        return;
    }
    const int sx = (region.nx + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int sy = (region.ny + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int sz = (region.nz + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
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

void rebuild_default_packed_atlas(ServiceState& service, long long region_key) {
    auto region_it = service.dynamic_regions.find(region_key);
    if (region_it == service.dynamic_regions.end()) {
        service.atlases.erase(region_key);
        return;
    }
    rebuild_default_packed_atlas(region_it->second, service.atlases[region_key]);
}

float rebuild_default_packed_atlas_from_flow_samples(
    const float* flow_samples,
    int sample_cell_count,
    AtlasData& atlas
) {
    if (!flow_samples || sample_cell_count <= 0) {
        atlas.values.clear();
        return 0.0f;
    }
    atlas.values.assign(
        static_cast<size_t>(sample_cell_count) * AERO_LBM_SIMULATION_PACKED_ATLAS_CHANNELS,
        0
    );
    float max_speed = 0.0f;
    for (int cell = 0; cell < sample_cell_count; ++cell) {
        const size_t src = static_cast<size_t>(cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        const size_t dst = static_cast<size_t>(cell) * AERO_LBM_SIMULATION_PACKED_ATLAS_CHANNELS;
        const float vx = flow_samples[src];
        const float vy = flow_samples[src + 1];
        const float vz = flow_samples[src + 2];
        const float pressure = flow_samples[src + 3];
        atlas.values[dst] = quantize_signed(vx, k_atlas_velocity_quant_range);
        atlas.values[dst + 1] = quantize_signed(vy, k_atlas_velocity_quant_range);
        atlas.values[dst + 2] = quantize_signed(vz, k_atlas_velocity_quant_range);
        atlas.values[dst + 3] = quantize_signed(pressure, k_atlas_pressure_quant_range);
        const float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (std::isfinite(speed) && speed > max_speed) {
            max_speed = speed;
        }
    }
    return max_speed;
}

float sync_dynamic_region_from_native(long long region_key, DynamicRegionData& dynamic, AtlasData* atlas) {
    int cells = 0;
    if (!checked_cell_count(dynamic.nx, dynamic.ny, dynamic.nz, &cells)) {
        return 0.0f;
    }
    if (dynamic.flow_state.size() != static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS) {
        dynamic.flow_state.assign(static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS, 0.0f);
    }
    if (dynamic.air_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.air_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }
    if (!aero_lbm_get_flow_state_rect(dynamic.nx, dynamic.ny, dynamic.nz, region_key, dynamic.flow_state.data())) {
        return -1.0f;
    }
    if (!aero_lbm_get_temperature_state_rect(dynamic.nx, dynamic.ny, dynamic.nz, region_key, dynamic.air_temperature.data())) {
        return -1.0f;
    }
    float max_speed = 0.0f;
    for (int cell = 0; cell < cells; ++cell) {
        size_t base = static_cast<size_t>(cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        float vx = dynamic.flow_state[base];
        float vy = dynamic.flow_state[base + 1];
        float vz = dynamic.flow_state[base + 2];
        float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (std::isfinite(speed) && speed > max_speed) {
            max_speed = speed;
        }
    }
    if (atlas) {
        rebuild_default_packed_atlas(dynamic, *atlas);
    }
    return max_speed;
}

bool sync_dynamic_region_temperature_from_native(long long region_key, DynamicRegionData& dynamic) {
    int cells = 0;
    if (!checked_cell_count(dynamic.nx, dynamic.ny, dynamic.nz, &cells)) {
        return false;
    }
    if (dynamic.air_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.air_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }
    return aero_lbm_get_temperature_state_rect(dynamic.nx, dynamic.ny, dynamic.nz, region_key, dynamic.air_temperature.data()) != 0;
}

float compute_max_speed_from_flow_state(const DynamicRegionData& dynamic) {
    const size_t cells = dynamic.flow_state.size() / AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
    float max_speed = 0.0f;
    for (size_t cell = 0; cell < cells; ++cell) {
        size_t base = cell * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
        float vx = dynamic.flow_state[base];
        float vy = dynamic.flow_state[base + 1];
        float vz = dynamic.flow_state[base + 2];
        float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (std::isfinite(speed) && speed > max_speed) {
            max_speed = speed;
        }
    }
    return max_speed;
}

bool sync_dynamic_region_flow_from_native(long long region_key, DynamicRegionData& dynamic, AtlasData* atlas, float* out_max_speed) {
    int cells = 0;
    if (!checked_cell_count(dynamic.nx, dynamic.ny, dynamic.nz, &cells)) {
        return false;
    }
    if (dynamic.flow_state.size() != static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS) {
        dynamic.flow_state.assign(static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS, 0.0f);
    }
    if (!aero_lbm_get_flow_state_rect(dynamic.nx, dynamic.ny, dynamic.nz, region_key, dynamic.flow_state.data())) {
        return false;
    }
    if (atlas) {
        rebuild_default_packed_atlas(dynamic, *atlas);
    }
    if (out_max_speed) {
        *out_max_speed = compute_max_speed_from_flow_state(dynamic);
    }
    return true;
}

float sync_dynamic_region_from_native(ServiceState& service, long long region_key) {
    auto dynamic_it = service.dynamic_regions.find(region_key);
    if (dynamic_it == service.dynamic_regions.end()) {
        return 0.0f;
    }
    return sync_dynamic_region_from_native(region_key, dynamic_it->second, &service.atlases[region_key]);
}

void apply_boundary_wind(
    std::vector<float>& packet,
    int nx,
    int ny,
    int nz,
    float boundary_wind_x,
    float boundary_wind_y,
    float boundary_wind_z
) {
    if (packet.empty() || nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    for (int x = 0; x < nx; ++x) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                int edge_distance = std::min(
                    std::min(std::min(x, y), z),
                    std::min(std::min(nx - 1 - x, ny - 1 - y), nz - 1 - z)
                );
                if (edge_distance >= k_window_edge_stabilization_layers) {
                    continue;
                }
                int cell = (x * ny + y) * nz + z;
                size_t base = static_cast<size_t>(cell) * k_packet_channels;
                if (packet[base + k_channel_obstacle] > 0.5f) {
                    continue;
                }
                float eta = (k_window_edge_stabilization_layers - edge_distance)
                    / static_cast<float>(k_window_edge_stabilization_layers);
                float keep = k_window_edge_stabilization_min_keep
                    + (1.0f - k_window_edge_stabilization_min_keep) * (1.0f - eta * eta);
                float relax = 1.0f - keep;
                packet[base + k_channel_state_vx] = packet[base + k_channel_state_vx] * keep + boundary_wind_x * relax;
                packet[base + k_channel_state_vy] = packet[base + k_channel_state_vy] * keep + boundary_wind_y * relax;
                packet[base + k_channel_state_vz] = packet[base + k_channel_state_vz] * keep + boundary_wind_z * relax;
                packet[base + k_channel_state_p] *= keep;
            }
        }
    }
}

void apply_nested_boundary_fields(
    std::vector<float>& packet,
    int nx,
    int ny,
    int nz,
    float fallback_boundary_wind_x,
    float fallback_boundary_wind_y,
    float fallback_boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    int external_face_mask,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k
) {
    if (packet.empty() || nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    auto apply_face = [&](int face, int x, int y, int z, int layer_index, double u, double v) {
        int cell = grid_cell_index(ny, nz, x, y, z);
        size_t base = static_cast<size_t>(cell) * k_packet_channels;
        if (packet[base + k_channel_obstacle] > 0.5f) {
            return;
        }
        float eta = (k_nested_boundary_layers - layer_index) / static_cast<float>(k_nested_boundary_layers);
        float keep = k_nested_boundary_min_keep + (1.0f - k_nested_boundary_min_keep) * (1.0f - eta * eta);
        float relax = 1.0f - keep;
        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        float air_k = 0.0f;
        sample_boundary_face_target(
            face,
            boundary_face_resolution,
            external_face_mask,
            boundary_wind_face_x,
            boundary_wind_face_y,
            boundary_wind_face_z,
            boundary_air_temperature_k,
            fallback_boundary_wind_x,
            fallback_boundary_wind_y,
            fallback_boundary_wind_z,
            fallback_boundary_air_temperature_k,
            u,
            v,
            vx,
            vy,
            vz,
            air_k
        );
        packet[base + k_channel_state_vx] = packet[base + k_channel_state_vx] * keep + vx * relax;
        packet[base + k_channel_state_vy] = packet[base + k_channel_state_vy] * keep + vy * relax;
        packet[base + k_channel_state_vz] = packet[base + k_channel_state_vz] * keep + vz * relax;
        packet[base + k_channel_state_p] *= keep;
        packet[base + k_channel_state_temp] = packet[base + k_channel_state_temp] * keep
            + ((air_k - fallback_boundary_air_temperature_k) / k_runtime_temperature_scale_kelvin) * relax;
    };

    for (int layer = 0; layer < k_nested_boundary_layers; ++layer) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                double u = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                apply_face(4, layer, y, z, layer, u, v);
                apply_face(5, nx - 1 - layer, y, z, layer, u, v);
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                double v = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                apply_face(2, x, y, layer, layer, u, v);
                apply_face(3, x, y, nz - 1 - layer, layer, u, v);
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int z = 0; z < nz; ++z) {
                double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                apply_face(0, x, layer, z, layer, u, v);
                apply_face(1, x, ny - 1 - layer, z, layer, u, v);
            }
        }
    }
}

void apply_sponge_boundary_fields(
    std::vector<float>& packet,
    int nx,
    int ny,
    int nz,
    float fallback_boundary_wind_x,
    float fallback_boundary_wind_y,
    float fallback_boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    int external_face_mask,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k,
    int sponge_thickness_cells,
    float sponge_velocity_relaxation,
    float sponge_temperature_relaxation
) {
    if (packet.empty() || nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    if (sponge_thickness_cells <= 0 || external_face_mask == 0) {
        return;
    }
    const float velocity_relaxation = std::max(0.0f, sponge_velocity_relaxation);
    const float temperature_relaxation = std::max(0.0f, sponge_temperature_relaxation);
    if (velocity_relaxation <= 0.0f && temperature_relaxation <= 0.0f) {
        return;
    }

    auto apply_face = [&](int face, int x, int y, int z, int layer_index, double u, double v) {
        if (layer_index < 0 || layer_index >= sponge_thickness_cells) {
            return;
        }
        const int cell = grid_cell_index(ny, nz, x, y, z);
        const size_t base = static_cast<size_t>(cell) * k_packet_channels;
        if (packet[base + k_channel_obstacle] > 0.5f) {
            return;
        }
        const float normalized = 1.0f - (layer_index / static_cast<float>(sponge_thickness_cells));
        const float weight = normalized * normalized;
        const float velocity_relax = std::clamp(velocity_relaxation * weight, 0.0f, k_sponge_relaxation_max);
        const float temperature_relax = std::clamp(temperature_relaxation * weight, 0.0f, k_sponge_relaxation_max);
        if (velocity_relax <= 0.0f && temperature_relax <= 0.0f) {
            return;
        }

        float target_vx = 0.0f;
        float target_vy = 0.0f;
        float target_vz = 0.0f;
        float target_air_k = 0.0f;
        sample_boundary_face_target(
            face,
            boundary_face_resolution,
            external_face_mask,
            boundary_wind_face_x,
            boundary_wind_face_y,
            boundary_wind_face_z,
            boundary_air_temperature_k,
            fallback_boundary_wind_x,
            fallback_boundary_wind_y,
            fallback_boundary_wind_z,
            fallback_boundary_air_temperature_k,
            u,
            v,
            target_vx,
            target_vy,
            target_vz,
            target_air_k
        );

        if (velocity_relax > 0.0f) {
            const float velocity_keep = 1.0f - velocity_relax;
            packet[base + k_channel_state_vx] = packet[base + k_channel_state_vx] * velocity_keep + target_vx * velocity_relax;
            packet[base + k_channel_state_vy] = packet[base + k_channel_state_vy] * velocity_keep + target_vy * velocity_relax;
            packet[base + k_channel_state_vz] = packet[base + k_channel_state_vz] * velocity_keep + target_vz * velocity_relax;
        }
        if (temperature_relax > 0.0f) {
            const float temperature_keep = 1.0f - temperature_relax;
            const float target_temp = (target_air_k - fallback_boundary_air_temperature_k) / k_runtime_temperature_scale_kelvin;
            packet[base + k_channel_state_temp] = packet[base + k_channel_state_temp] * temperature_keep
                + target_temp * temperature_relax;
        }
    };

    for (int layer = 0; layer < sponge_thickness_cells; ++layer) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const double u = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                const double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                if ((external_face_mask & (1 << 4)) != 0) {
                    apply_face(4, layer, y, z, layer, u, v);
                }
                if ((external_face_mask & (1 << 5)) != 0) {
                    apply_face(5, nx - 1 - layer, y, z, layer, u, v);
                }
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                const double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                const double v = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                if ((external_face_mask & (1 << 2)) != 0) {
                    apply_face(2, x, y, layer, layer, u, v);
                }
                if ((external_face_mask & (1 << 3)) != 0) {
                    apply_face(3, x, y, nz - 1 - layer, layer, u, v);
                }
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int z = 0; z < nz; ++z) {
                const double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                const double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                if ((external_face_mask & (1 << 0)) != 0) {
                    apply_face(0, x, layer, z, layer, u, v);
                }
                if ((external_face_mask & (1 << 1)) != 0) {
                    apply_face(1, x, ny - 1 - layer, z, layer, u, v);
                }
            }
        }
    }
}

void apply_tornado_vortex_descriptors(
    std::vector<float>& packet,
    int nx,
    int ny,
    int nz,
    int tornado_descriptor_count,
    const float* tornado_descriptors
) {
    if (packet.empty() || nx <= 0 || ny <= 0 || nz <= 0 || tornado_descriptor_count <= 0 || !tornado_descriptors) {
        return;
    }
    for (int descriptor_index = 0; descriptor_index < tornado_descriptor_count; ++descriptor_index) {
        const float* descriptor = tornado_descriptors + descriptor_index * k_tornado_descriptor_floats;
        const float center_x = descriptor[k_tornado_desc_center_x];
        const float center_y = descriptor[k_tornado_desc_center_y];
        const float center_z = descriptor[k_tornado_desc_center_z];
        const float core_radius = std::max(1.0f, descriptor[k_tornado_desc_core_radius]);
        const float influence_radius = std::max(core_radius, descriptor[k_tornado_desc_influence_radius]);
        const float tangential_lattice = descriptor[k_tornado_desc_tangential_lattice];
        const float radial_lattice = descriptor[k_tornado_desc_radial_lattice];
        const float updraft_lattice = descriptor[k_tornado_desc_updraft_lattice];
        const float condensation_bias = descriptor[k_tornado_desc_condensation_bias];
        const float intensity = std::max(0.0f, descriptor[k_tornado_desc_intensity]);
        const float rotation_sign = descriptor[k_tornado_desc_rotation_sign] >= 0.0f ? 1.0f : -1.0f;
        const float lifecycle_envelope = std::clamp(descriptor[k_tornado_desc_lifecycle_envelope], 0.0f, 1.0f);
        const float effective_intensity = intensity * lifecycle_envelope;
        if (effective_intensity <= 1.0e-4f) {
            continue;
        }

        const int min_x = std::max(0, static_cast<int>(std::floor(center_x - influence_radius - 1.0f)));
        const int max_x = std::min(nx - 1, static_cast<int>(std::ceil(center_x + influence_radius + 1.0f)));
        const int min_z = std::max(0, static_cast<int>(std::floor(center_z - influence_radius - 1.0f)));
        const int max_z = std::min(nz - 1, static_cast<int>(std::ceil(center_z + influence_radius + 1.0f)));
        const int min_y = std::max(0, static_cast<int>(std::floor(center_y - core_radius * 0.75f - 1.0f)));
        const int max_y = std::min(ny - 1, static_cast<int>(std::ceil(center_y + influence_radius * 1.25f + 1.0f)));

        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                for (int z = min_z; z <= max_z; ++z) {
                    const int cell = grid_cell_index(ny, nz, x, y, z);
                    const size_t base = static_cast<size_t>(cell) * k_packet_channels;
                    if (packet[base + k_channel_obstacle] > 0.5f) {
                        continue;
                    }
                    const float px = static_cast<float>(x) + 0.5f;
                    const float py = static_cast<float>(y) + 0.5f;
                    const float pz = static_cast<float>(z) + 0.5f;
                    const float dx = px - center_x;
                    const float dy = py - center_y;
                    const float dz = pz - center_z;
                    const float horizontal_distance_sq = dx * dx + dz * dz;
                    if (horizontal_distance_sq >= influence_radius * influence_radius) {
                        continue;
                    }

                    const float horizontal_distance = std::max(1.0e-3f, std::sqrt(horizontal_distance_sq));
                    const float outer_norm = horizontal_distance / std::max(1.0f, influence_radius);
                    const float core_norm = horizontal_distance / std::max(1.0f, core_radius);
                    const float outer_envelope = std::exp(-outer_norm * outer_norm * 1.2f);
                    const float core_envelope = std::exp(-core_norm * core_norm * 2.2f);
                    const float above_envelope = dy <= 0.0f
                        ? 1.0f
                        : std::exp(-(dy / std::max(1.0f, influence_radius * 0.80f)) * (dy / std::max(1.0f, influence_radius * 0.80f)));
                    const float below_envelope = dy >= 0.0f
                        ? 1.0f
                        : std::exp(-(dy / std::max(1.0f, core_radius * 0.75f)) * (dy / std::max(1.0f, core_radius * 0.75f)));
                    const float vertical_envelope = above_envelope * below_envelope;
                    const float envelope = effective_intensity * vertical_envelope;
                    if (envelope <= 1.0e-4f) {
                        continue;
                    }

                    const float tangent_x = (-dz / horizontal_distance) * rotation_sign;
                    const float tangent_z = (dx / horizontal_distance) * rotation_sign;
                    const float radial_x = -dx / horizontal_distance;
                    const float radial_z = -dz / horizontal_distance;
                    const float swirl = tangential_lattice * envelope * (0.30f * outer_envelope + 1.10f * core_envelope);
                    const float inflow = radial_lattice * envelope * (0.55f * outer_envelope + 0.35f * core_envelope);
                    const float updraft = updraft_lattice * envelope * (0.30f * outer_envelope + 0.90f * core_envelope);
                    const float heating = 0.0012f * condensation_bias * envelope * (0.20f * outer_envelope + 0.65f * core_envelope);

                    packet[base + k_channel_state_vx] = std::clamp(
                        packet[base + k_channel_state_vx] + tangent_x * swirl + radial_x * inflow,
                        -k_tornado_lattice_speed_cap,
                        k_tornado_lattice_speed_cap
                    );
                    packet[base + k_channel_state_vy] = std::clamp(
                        packet[base + k_channel_state_vy] + updraft,
                        -k_tornado_lattice_speed_cap,
                        k_tornado_lattice_speed_cap
                    );
                    packet[base + k_channel_state_vz] = std::clamp(
                        packet[base + k_channel_state_vz] + tangent_z * swirl + radial_z * inflow,
                        -k_tornado_lattice_speed_cap,
                        k_tornado_lattice_speed_cap
                    );
                    packet[base + k_channel_thermal_source] = std::clamp(
                        packet[base + k_channel_thermal_source] + heating,
                        -k_native_thermal_source_max,
                        k_native_thermal_source_max
                    );
                }
            }
        }
    }
}

void apply_boundary_wind_overlay(
    SparsePacketOverlayBuilder& overlay,
    const float* base_packet,
    int nx,
    int ny,
    int nz,
    float boundary_wind_x,
    float boundary_wind_y,
    float boundary_wind_z
) {
    if (!base_packet || nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    for (int x = 0; x < nx; ++x) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const int edge_distance = std::min(
                    std::min(std::min(x, y), z),
                    std::min(std::min(nx - 1 - x, ny - 1 - y), nz - 1 - z)
                );
                if (edge_distance >= k_window_edge_stabilization_layers) {
                    continue;
                }
                const int cell = grid_cell_index(ny, nz, x, y, z);
                const size_t base = static_cast<size_t>(cell) * k_packet_channels;
                if (base_packet[base + k_channel_obstacle] > 0.5f) {
                    continue;
                }
                float* entry = overlay.touch(base_packet, cell);
                if (!entry) {
                    continue;
                }
                const float eta = (k_window_edge_stabilization_layers - edge_distance)
                    / static_cast<float>(k_window_edge_stabilization_layers);
                const float keep = k_window_edge_stabilization_min_keep
                    + (1.0f - k_window_edge_stabilization_min_keep) * (1.0f - eta * eta);
                const float relax = 1.0f - keep;
                entry[k_sparse_overlay_state_vx] = entry[k_sparse_overlay_state_vx] * keep + boundary_wind_x * relax;
                entry[k_sparse_overlay_state_vy] = entry[k_sparse_overlay_state_vy] * keep + boundary_wind_y * relax;
                entry[k_sparse_overlay_state_vz] = entry[k_sparse_overlay_state_vz] * keep + boundary_wind_z * relax;
                entry[k_sparse_overlay_state_p] *= keep;
            }
        }
    }
}

void apply_nested_boundary_field_overlay(
    SparsePacketOverlayBuilder& overlay,
    const float* base_packet,
    int nx,
    int ny,
    int nz,
    float fallback_boundary_wind_x,
    float fallback_boundary_wind_y,
    float fallback_boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    int external_face_mask,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k
) {
    if (!base_packet || nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    auto apply_face = [&](int face, int x, int y, int z, int layer_index, double u, double v) {
        const int cell = grid_cell_index(ny, nz, x, y, z);
        const size_t base = static_cast<size_t>(cell) * k_packet_channels;
        if (base_packet[base + k_channel_obstacle] > 0.5f) {
            return;
        }
        float* entry = overlay.touch(base_packet, cell);
        if (!entry) {
            return;
        }
        const float eta = (k_nested_boundary_layers - layer_index) / static_cast<float>(k_nested_boundary_layers);
        const float keep = k_nested_boundary_min_keep + (1.0f - k_nested_boundary_min_keep) * (1.0f - eta * eta);
        const float relax = 1.0f - keep;
        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        float air_k = 0.0f;
        sample_boundary_face_target(
            face,
            boundary_face_resolution,
            external_face_mask,
            boundary_wind_face_x,
            boundary_wind_face_y,
            boundary_wind_face_z,
            boundary_air_temperature_k,
            fallback_boundary_wind_x,
            fallback_boundary_wind_y,
            fallback_boundary_wind_z,
            fallback_boundary_air_temperature_k,
            u,
            v,
            vx,
            vy,
            vz,
            air_k
        );
        entry[k_sparse_overlay_state_vx] = entry[k_sparse_overlay_state_vx] * keep + vx * relax;
        entry[k_sparse_overlay_state_vy] = entry[k_sparse_overlay_state_vy] * keep + vy * relax;
        entry[k_sparse_overlay_state_vz] = entry[k_sparse_overlay_state_vz] * keep + vz * relax;
        entry[k_sparse_overlay_state_p] *= keep;
        entry[k_sparse_overlay_state_temp] = entry[k_sparse_overlay_state_temp] * keep
            + ((air_k - fallback_boundary_air_temperature_k) / k_runtime_temperature_scale_kelvin) * relax;
    };

    for (int layer = 0; layer < k_nested_boundary_layers; ++layer) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const double u = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                const double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                apply_face(4, layer, y, z, layer, u, v);
                apply_face(5, nx - 1 - layer, y, z, layer, u, v);
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                const double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                const double v = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                apply_face(2, x, y, layer, layer, u, v);
                apply_face(3, x, y, nz - 1 - layer, layer, u, v);
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int z = 0; z < nz; ++z) {
                const double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                const double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                apply_face(0, x, layer, z, layer, u, v);
                apply_face(1, x, ny - 1 - layer, z, layer, u, v);
            }
        }
    }
}

void apply_sponge_boundary_field_overlay(
    SparsePacketOverlayBuilder& overlay,
    const float* base_packet,
    int nx,
    int ny,
    int nz,
    float fallback_boundary_wind_x,
    float fallback_boundary_wind_y,
    float fallback_boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    int external_face_mask,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k,
    int sponge_thickness_cells,
    float sponge_velocity_relaxation,
    float sponge_temperature_relaxation
) {
    if (!base_packet || nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    if (sponge_thickness_cells <= 0 || external_face_mask == 0) {
        return;
    }
    const float velocity_relaxation = std::max(0.0f, sponge_velocity_relaxation);
    const float temperature_relaxation = std::max(0.0f, sponge_temperature_relaxation);
    if (velocity_relaxation <= 0.0f && temperature_relaxation <= 0.0f) {
        return;
    }

    auto apply_face = [&](int face, int x, int y, int z, int layer_index, double u, double v) {
        if (layer_index < 0 || layer_index >= sponge_thickness_cells) {
            return;
        }
        const int cell = grid_cell_index(ny, nz, x, y, z);
        const size_t base = static_cast<size_t>(cell) * k_packet_channels;
        if (base_packet[base + k_channel_obstacle] > 0.5f) {
            return;
        }
        float* entry = overlay.touch(base_packet, cell);
        if (!entry) {
            return;
        }
        const float normalized = 1.0f - (layer_index / static_cast<float>(sponge_thickness_cells));
        const float weight = normalized * normalized;
        const float velocity_relax = std::clamp(velocity_relaxation * weight, 0.0f, k_sponge_relaxation_max);
        const float temperature_relax = std::clamp(temperature_relaxation * weight, 0.0f, k_sponge_relaxation_max);
        if (velocity_relax <= 0.0f && temperature_relax <= 0.0f) {
            return;
        }

        float target_vx = 0.0f;
        float target_vy = 0.0f;
        float target_vz = 0.0f;
        float target_air_k = 0.0f;
        sample_boundary_face_target(
            face,
            boundary_face_resolution,
            external_face_mask,
            boundary_wind_face_x,
            boundary_wind_face_y,
            boundary_wind_face_z,
            boundary_air_temperature_k,
            fallback_boundary_wind_x,
            fallback_boundary_wind_y,
            fallback_boundary_wind_z,
            fallback_boundary_air_temperature_k,
            u,
            v,
            target_vx,
            target_vy,
            target_vz,
            target_air_k
        );

        if (velocity_relax > 0.0f) {
            const float velocity_keep = 1.0f - velocity_relax;
            entry[k_sparse_overlay_state_vx] = entry[k_sparse_overlay_state_vx] * velocity_keep + target_vx * velocity_relax;
            entry[k_sparse_overlay_state_vy] = entry[k_sparse_overlay_state_vy] * velocity_keep + target_vy * velocity_relax;
            entry[k_sparse_overlay_state_vz] = entry[k_sparse_overlay_state_vz] * velocity_keep + target_vz * velocity_relax;
        }
        if (temperature_relax > 0.0f) {
            const float temperature_keep = 1.0f - temperature_relax;
            const float target_temp = (target_air_k - fallback_boundary_air_temperature_k) / k_runtime_temperature_scale_kelvin;
            entry[k_sparse_overlay_state_temp] = entry[k_sparse_overlay_state_temp] * temperature_keep
                + target_temp * temperature_relax;
        }
    };

    for (int layer = 0; layer < sponge_thickness_cells; ++layer) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const double u = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                const double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                if ((external_face_mask & (1 << 4)) != 0) {
                    apply_face(4, layer, y, z, layer, u, v);
                }
                if ((external_face_mask & (1 << 5)) != 0) {
                    apply_face(5, nx - 1 - layer, y, z, layer, u, v);
                }
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                const double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                const double v = ((y + 0.5) / ny) * std::max(1, boundary_face_resolution - 1);
                if ((external_face_mask & (1 << 2)) != 0) {
                    apply_face(2, x, y, layer, layer, u, v);
                }
                if ((external_face_mask & (1 << 3)) != 0) {
                    apply_face(3, x, y, nz - 1 - layer, layer, u, v);
                }
            }
        }
        for (int x = 0; x < nx; ++x) {
            for (int z = 0; z < nz; ++z) {
                const double u = ((x + 0.5) / nx) * std::max(1, boundary_face_resolution - 1);
                const double v = ((z + 0.5) / nz) * std::max(1, boundary_face_resolution - 1);
                if ((external_face_mask & (1 << 0)) != 0) {
                    apply_face(0, x, layer, z, layer, u, v);
                }
                if ((external_face_mask & (1 << 1)) != 0) {
                    apply_face(1, x, ny - 1 - layer, z, layer, u, v);
                }
            }
        }
    }
}

void apply_tornado_vortex_descriptor_overlay(
    SparsePacketOverlayBuilder& overlay,
    const float* base_packet,
    int nx,
    int ny,
    int nz,
    int tornado_descriptor_count,
    const float* tornado_descriptors
) {
    if (!base_packet || nx <= 0 || ny <= 0 || nz <= 0 || tornado_descriptor_count <= 0 || !tornado_descriptors) {
        return;
    }
    for (int descriptor_index = 0; descriptor_index < tornado_descriptor_count; ++descriptor_index) {
        const float* descriptor = tornado_descriptors + descriptor_index * k_tornado_descriptor_floats;
        const float center_x = descriptor[k_tornado_desc_center_x];
        const float center_y = descriptor[k_tornado_desc_center_y];
        const float center_z = descriptor[k_tornado_desc_center_z];
        const float core_radius = std::max(1.0f, descriptor[k_tornado_desc_core_radius]);
        const float influence_radius = std::max(core_radius, descriptor[k_tornado_desc_influence_radius]);
        const float tangential_lattice = descriptor[k_tornado_desc_tangential_lattice];
        const float radial_lattice = descriptor[k_tornado_desc_radial_lattice];
        const float updraft_lattice = descriptor[k_tornado_desc_updraft_lattice];
        const float condensation_bias = descriptor[k_tornado_desc_condensation_bias];
        const float intensity = std::max(0.0f, descriptor[k_tornado_desc_intensity]);
        const float rotation_sign = descriptor[k_tornado_desc_rotation_sign] >= 0.0f ? 1.0f : -1.0f;
        const float lifecycle_envelope = std::clamp(descriptor[k_tornado_desc_lifecycle_envelope], 0.0f, 1.0f);
        const float effective_intensity = intensity * lifecycle_envelope;
        if (effective_intensity <= 1.0e-4f) {
            continue;
        }

        const int min_x = std::max(0, static_cast<int>(std::floor(center_x - influence_radius - 1.0f)));
        const int max_x = std::min(nx - 1, static_cast<int>(std::ceil(center_x + influence_radius + 1.0f)));
        const int min_z = std::max(0, static_cast<int>(std::floor(center_z - influence_radius - 1.0f)));
        const int max_z = std::min(nz - 1, static_cast<int>(std::ceil(center_z + influence_radius + 1.0f)));
        const int min_y = std::max(0, static_cast<int>(std::floor(center_y - core_radius * 0.75f - 1.0f)));
        const int max_y = std::min(ny - 1, static_cast<int>(std::ceil(center_y + influence_radius * 1.25f + 1.0f)));

        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                for (int z = min_z; z <= max_z; ++z) {
                    const int cell = grid_cell_index(ny, nz, x, y, z);
                    const size_t base = static_cast<size_t>(cell) * k_packet_channels;
                    if (base_packet[base + k_channel_obstacle] > 0.5f) {
                        continue;
                    }
                    const float px = static_cast<float>(x) + 0.5f;
                    const float py = static_cast<float>(y) + 0.5f;
                    const float pz = static_cast<float>(z) + 0.5f;
                    const float dx = px - center_x;
                    const float dy = py - center_y;
                    const float dz = pz - center_z;
                    const float horizontal_distance_sq = dx * dx + dz * dz;
                    if (horizontal_distance_sq >= influence_radius * influence_radius) {
                        continue;
                    }

                    const float horizontal_distance = std::max(1.0e-3f, std::sqrt(horizontal_distance_sq));
                    const float outer_norm = horizontal_distance / std::max(1.0f, influence_radius);
                    const float core_norm = horizontal_distance / std::max(1.0f, core_radius);
                    const float outer_envelope = std::exp(-outer_norm * outer_norm * 1.2f);
                    const float core_envelope = std::exp(-core_norm * core_norm * 2.2f);
                    const float above_envelope = dy <= 0.0f
                        ? 1.0f
                        : std::exp(-(dy / std::max(1.0f, influence_radius * 0.80f)) * (dy / std::max(1.0f, influence_radius * 0.80f)));
                    const float below_envelope = dy >= 0.0f
                        ? 1.0f
                        : std::exp(-(dy / std::max(1.0f, core_radius * 0.75f)) * (dy / std::max(1.0f, core_radius * 0.75f)));
                    const float vertical_envelope = above_envelope * below_envelope;
                    const float envelope = effective_intensity * vertical_envelope;
                    if (envelope <= 1.0e-4f) {
                        continue;
                    }

                    float* entry = overlay.touch(base_packet, cell);
                    if (!entry) {
                        continue;
                    }
                    const float tangent_x = (-dz / horizontal_distance) * rotation_sign;
                    const float tangent_z = (dx / horizontal_distance) * rotation_sign;
                    const float radial_x = -dx / horizontal_distance;
                    const float radial_z = -dz / horizontal_distance;
                    const float swirl = tangential_lattice * envelope * (0.30f * outer_envelope + 1.10f * core_envelope);
                    const float inflow = radial_lattice * envelope * (0.55f * outer_envelope + 0.35f * core_envelope);
                    const float updraft = updraft_lattice * envelope * (0.30f * outer_envelope + 0.90f * core_envelope);
                    const float heating = 0.0012f * condensation_bias * envelope * (0.20f * outer_envelope + 0.65f * core_envelope);

                    entry[k_sparse_overlay_state_vx] = std::clamp(
                        entry[k_sparse_overlay_state_vx] + tangent_x * swirl + radial_x * inflow,
                        -k_tornado_lattice_speed_cap,
                        k_tornado_lattice_speed_cap
                    );
                    entry[k_sparse_overlay_state_vy] = std::clamp(
                        entry[k_sparse_overlay_state_vy] + updraft,
                        -k_tornado_lattice_speed_cap,
                        k_tornado_lattice_speed_cap
                    );
                    entry[k_sparse_overlay_state_vz] = std::clamp(
                        entry[k_sparse_overlay_state_vz] + tangent_z * swirl + radial_z * inflow,
                        -k_tornado_lattice_speed_cap,
                        k_tornado_lattice_speed_cap
                    );
                    entry[k_sparse_overlay_thermal_source] = std::clamp(
                        entry[k_sparse_overlay_thermal_source] + heating,
                        -k_native_thermal_source_max,
                        k_native_thermal_source_max
                    );
                }
            }
        }
    }
}

bool build_region_packet_from_template(
    const RegionPacketTemplateData& packet_template,
    const DynamicRegionData& dynamic,
    bool include_dynamic_state,
    int nx,
    int ny,
    int nz,
    float boundary_wind_x,
    float boundary_wind_y,
    float boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    int external_face_mask,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k,
    int sponge_thickness_cells,
    int tornado_descriptor_count,
    const float* tornado_descriptors,
    std::vector<float>& packet
) {
    if (packet_template.nx != nx || packet_template.ny != ny || packet_template.nz != nz
        || dynamic.nx != nx || dynamic.ny != ny || dynamic.nz != nz) {
        set_simulation_last_error("simulation_step_region_stored: region dimension mismatch");
        return false;
    }
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        set_simulation_last_error("simulation_step_region_stored: invalid dimensions");
        return false;
    }
    const size_t packet_values = static_cast<size_t>(cells) * k_packet_channels;
    if (packet.size() != packet_values) {
        packet.resize(packet_values);
    }
    std::copy(packet_template.values.begin(), packet_template.values.end(), packet.begin());
    if (include_dynamic_state) {
        for (int cell = 0; cell < cells; ++cell) {
            size_t packet_base = static_cast<size_t>(cell) * k_packet_channels;
            size_t flow_base = static_cast<size_t>(cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
            packet[packet_base + k_channel_state_vx] = dynamic.flow_state[flow_base];
            packet[packet_base + k_channel_state_vy] = dynamic.flow_state[flow_base + 1];
            packet[packet_base + k_channel_state_vz] = dynamic.flow_state[flow_base + 2];
            packet[packet_base + k_channel_state_p] = dynamic.flow_state[flow_base + 3];
        }
    }
    if (include_dynamic_state && dynamic.air_temperature.size() == static_cast<size_t>(cells)) {
        for (int cell = 0; cell < cells; ++cell) {
            size_t packet_base = static_cast<size_t>(cell) * k_packet_channels;
            packet[packet_base + k_channel_state_temp] = dynamic.air_temperature[static_cast<size_t>(cell)];
        }
    }
    if (sponge_thickness_cells <= 0 && external_face_mask != 0) {
        apply_boundary_wind(packet, nx, ny, nz, boundary_wind_x, boundary_wind_y, boundary_wind_z);
    }
    apply_tornado_vortex_descriptors(packet, nx, ny, nz, tornado_descriptor_count, tornado_descriptors);
    return true;
}

bool refresh_region_thermal(
    ServiceState& service,
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
) {
    auto static_it = service.static_regions.find(region_key);
    auto dynamic_it = service.dynamic_regions.find(region_key);
    auto forcing_it = service.forcing_regions.find(region_key);
    if (static_it == service.static_regions.end()) {
        set_simulation_last_error("simulation_refresh_region_thermal: missing static region");
        return false;
    }
    if (dynamic_it == service.dynamic_regions.end()) {
        set_simulation_last_error("simulation_refresh_region_thermal: missing dynamic region");
        return false;
    }
    if (forcing_it == service.forcing_regions.end()) {
        set_simulation_last_error("simulation_refresh_region_thermal: missing forcing region");
        return false;
    }
    const StaticRegionData& stat = *static_it->second;
    DynamicRegionData& dynamic = dynamic_it->second;
    ForcingRegionData forcing = *forcing_it->second;
    if (stat.nx != nx || stat.ny != ny || stat.nz != nz
        || dynamic.nx != nx || dynamic.ny != ny || dynamic.nz != nz
        || forcing.nx != nx || forcing.ny != ny || forcing.nz != nz) {
        set_simulation_last_error("simulation_refresh_region_thermal: region dimension mismatch");
        return false;
    }
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        set_simulation_last_error("simulation_refresh_region_thermal: invalid dimensions");
        return false;
    }
    if (dynamic.surface_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.surface_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }
    if (dynamic.air_temperature.size() != static_cast<size_t>(cells)) {
        dynamic.air_temperature.assign(static_cast<size_t>(cells), 0.0f);
    }
    if (aero_lbm_has_context(region_key)) {
        if (!sync_dynamic_region_temperature_from_native(region_key, dynamic)) {
            set_simulation_last_error(std::string("simulation_refresh_region_thermal temperature sync failed: ") + aero_lbm_last_error());
            return false;
        }
    }
    if (forcing.thermal_source.size() != static_cast<size_t>(cells)) {
        forcing.thermal_source.assign(static_cast<size_t>(cells), 0.0f);
    } else {
        std::fill(forcing.thermal_source.begin(), forcing.thermal_source.end(), 0.0f);
    }

    static constexpr int dir_x[k_face_count] = {0, 0, 0, 0, -1, 1};
    static constexpr int dir_y[k_face_count] = {-1, 1, 0, 0, 0, 0};
    static constexpr int dir_z[k_face_count] = {0, 0, -1, 1, 0, 0};
    static constexpr float diffuse_weight[k_face_count] = {0.05f, 1.0f, 0.42f, 0.42f, 0.42f, 0.42f};
    static constexpr float sky_weight[k_face_count] = {0.08f, 1.0f, 0.55f, 0.55f, 0.55f, 0.55f};
    static constexpr float rain_weight[k_face_count] = {0.0f, 1.0f, 0.35f, 0.35f, 0.35f, 0.35f};

    for (int x = 0; x < nx; ++x) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const int cell = grid_cell_index(ny, nz, x, y, z);
                const uint8_t surface_kind = stat.surface_kind[static_cast<size_t>(cell)];
                const uint16_t open_face_mask = stat.open_face_mask[static_cast<size_t>(cell)];
                const float emitter_power_watts = stat.emitter_power_watts[static_cast<size_t>(cell)];
                const ThermalMaterialProperties* material = thermal_material_properties(surface_kind);

                if (!material) {
                    if (emitter_power_watts > 0.0f && open_face_mask != 0) {
                        const bool self_blocked = stat.obstacle[static_cast<size_t>(cell)] != 0;
                        const float self_weight = self_blocked ? 0.0f : 0.30f;
                        float face_weights[k_face_count] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                        float total_weight = self_weight;
                        for (int direction = 0; direction < k_face_count; ++direction) {
                            if (!has_face_bit(open_face_mask, direction)) {
                                continue;
                            }
                            const float weight = direction == 1 ? 0.55f : (direction == 0 ? 0.03f : 0.12f);
                            face_weights[direction] = weight;
                            total_weight += weight;
                        }
                        if (total_weight > 1.0e-6f) {
                            const float scalar_source = temperature_source_from_power_watts(emitter_power_watts);
                            if (self_weight > 0.0f) {
                                add_thermal_source(forcing.thermal_source, cell, scalar_source * (self_weight / total_weight));
                            }
                            for (int direction = 0; direction < k_face_count; ++direction) {
                                const float weight = face_weights[direction];
                                if (weight <= 0.0f) {
                                    continue;
                                }
                                const int nx_cell = x + dir_x[direction];
                                const int ny_cell = y + dir_y[direction];
                                const int nz_cell = z + dir_z[direction];
                                if (!in_bounds(nx, ny, nz, nx_cell, ny_cell, nz_cell)) {
                                    continue;
                                }
                                add_thermal_source(
                                    forcing.thermal_source,
                                    grid_cell_index(ny, nz, nx_cell, ny_cell, nz_cell),
                                    scalar_source * (weight / total_weight)
                                );
                            }
                        }
                    } else {
                        dynamic.surface_temperature[static_cast<size_t>(cell)] = 0.0f;
                    }
                    continue;
                }

                if (open_face_mask == 0) {
                    dynamic.surface_temperature[static_cast<size_t>(cell)] = 0.0f;
                    continue;
                }

                const int open_faces = count_open_faces(open_face_mask);
                float current_surface_temperature = dynamic.surface_temperature[static_cast<size_t>(cell)];
                if (!std::isfinite(current_surface_temperature) || current_surface_temperature <= 0.0f) {
                    current_surface_temperature = initialize_surface_temperature_kelvin(
                        *material,
                        ambient_air_temperature_k,
                        deep_ground_temperature_k,
                        emitter_power_watts,
                        open_faces
                    );
                }

                float solar_watts = 0.0f;
                float longwave_watts = 0.0f;
                float rain_watts = 0.0f;
                float convective_watts = 0.0f;
                for (int direction = 0; direction < k_face_count; ++direction) {
                    if (!has_face_bit(open_face_mask, direction)) {
                        continue;
                    }
                    const int face_index = local_face_index(cell, direction);
                    const float diffuse_sky = dequantize_unit_float(stat.face_sky_exposure[static_cast<size_t>(face_index)]);
                    const float direct_sky = dequantize_unit_float(stat.face_direct_exposure[static_cast<size_t>(face_index)]);
                    const float sun_dot = std::max(
                        0.0f,
                        dir_x[direction] * sun_x + dir_y[direction] * sun_y + dir_z[direction] * sun_z
                    );
                    const float air_temperature_kelvin = neighbor_air_temperature_kelvin(
                        dynamic,
                        x + dir_x[direction],
                        y + dir_y[direction],
                        z + dir_z[direction],
                        ambient_air_temperature_k
                    );
                    solar_watts += material->solar_absorptivity
                        * k_cell_face_area_square_meters
                        * (direct_solar_flux_w_m2 * direct_sky * sun_dot
                            + diffuse_solar_flux_w_m2 * diffuse_sky * diffuse_weight[direction]);
                    const float surface_temp_sq = current_surface_temperature * current_surface_temperature;
                    const float sky_temp_sq = sky_temperature_k * sky_temperature_k;
                    longwave_watts += material->emissivity
                        * k_thermal_stefan_boltzmann
                        * k_cell_face_area_square_meters
                        * (sky_temp_sq * sky_temp_sq - surface_temp_sq * surface_temp_sq)
                        * diffuse_sky
                        * sky_weight[direction];
                    rain_watts += material->rain_exchange_coefficient_wm2k
                        * k_cell_face_area_square_meters
                        * precipitation_strength
                        * rain_weight[direction]
                        * (precipitation_temperature_k - current_surface_temperature);
                    convective_watts += material->convective_exchange_coefficient_wm2k
                        * k_cell_face_area_square_meters
                        * (air_temperature_kelvin - current_surface_temperature);
                }

                const float exposed_area = open_faces * k_cell_face_area_square_meters;
                const float bulk_watts = material->bulk_conductance_wm2k
                    * exposed_area
                    * (deep_ground_temperature_k - current_surface_temperature);
                const float thermal_mass_j_per_k = std::max(1.0f, material->surface_heat_capacity_jm2k * exposed_area);
                const float updated_surface_temperature = std::clamp(
                    current_surface_temperature
                        + (solar_watts + longwave_watts + rain_watts + bulk_watts + convective_watts + emitter_power_watts)
                            * surface_delta_seconds
                            / thermal_mass_j_per_k,
                    k_thermal_surface_init_min_k,
                    k_thermal_surface_max_k
                );
                dynamic.surface_temperature[static_cast<size_t>(cell)] = updated_surface_temperature;

                for (int direction = 0; direction < k_face_count; ++direction) {
                    if (!has_face_bit(open_face_mask, direction)) {
                        continue;
                    }
                    const int nx_cell = x + dir_x[direction];
                    const int ny_cell = y + dir_y[direction];
                    const int nz_cell = z + dir_z[direction];
                    if (!in_bounds(nx, ny, nz, nx_cell, ny_cell, nz_cell)) {
                        continue;
                    }
                    const float air_temperature_kelvin = neighbor_air_temperature_kelvin(
                        dynamic,
                        nx_cell,
                        ny_cell,
                        nz_cell,
                        ambient_air_temperature_k
                    );
                    const float convective_power_watts = material->convective_exchange_coefficient_wm2k
                        * k_cell_face_area_square_meters
                        * (updated_surface_temperature - air_temperature_kelvin);
                    add_thermal_source(
                        forcing.thermal_source,
                        grid_cell_index(ny, nz, nx_cell, ny_cell, nz_cell),
                        temperature_source_from_power_watts(convective_power_watts)
                    );
                }
            }
        }
    }
    service.forcing_regions[region_key] = std::make_shared<ForcingRegionData>(std::move(forcing));
    rebuild_region_packet_template(service, region_key);
    return true;
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
    for (int index = 0; index < delta_count; ++index) {
        apply_world_delta_to_brick_runtimes(*service, deltas[index]);
    }
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_ensure_brick_world_runtime(
    long long service_key,
    long long world_key,
    int brick_size,
    float dx_meters,
    float dt_seconds
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_ensure_brick_world_runtime: missing service");
        return 0;
    }
    return ensure_brick_world_runtime(*service, world_key, brick_size, dx_meters, dt_seconds) ? 1 : 0;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_brick_world_active_hints(
    long long service_key,
    long long world_key,
    int brick_size,
    const int* brick_coords,
    int brick_count
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_set_brick_world_active_hints: missing service");
        return 0;
    }
    if (brick_count < 0 || (brick_count > 0 && !brick_coords)) {
        set_simulation_last_error("simulation_set_brick_world_active_hints: invalid hint payload");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_set_brick_world_active_hints: missing brick runtime");
        return 0;
    }
    FluidWorldRuntime& runtime = runtime_iterator->second;
    if (runtime.brick_size != brick_size) {
        set_simulation_last_error("simulation_set_brick_world_active_hints: brick size mismatch");
        return 0;
    }
    for (auto& entry : runtime.bricks) {
        entry.second.active_hint = false;
    }
    runtime.active_hint_closure.clear();
    runtime.epoch++;
    for (int index = 0; index < brick_count; ++index) {
        const int base = index * 3;
        BrickCoord coord{
            brick_coords[base],
            brick_coords[base + 1],
            brick_coords[base + 2]
        };
        BrickData& brick = runtime.bricks[coord];
        brick.active_hint = true;
        brick.last_hint_epoch = runtime.epoch;
        brick.last_active_epoch = runtime.epoch;
        runtime.active_hint_closure.insert(coord);
        runtime.active_hint_closure.insert(BrickCoord{coord.x - 1, coord.y, coord.z});
        runtime.active_hint_closure.insert(BrickCoord{coord.x + 1, coord.y, coord.z});
        runtime.active_hint_closure.insert(BrickCoord{coord.x, coord.y - 1, coord.z});
        runtime.active_hint_closure.insert(BrickCoord{coord.x, coord.y + 1, coord.z});
        runtime.active_hint_closure.insert(BrickCoord{coord.x, coord.y, coord.z - 1});
        runtime.active_hint_closure.insert(BrickCoord{coord.x, coord.y, coord.z + 1});
    }
    for (const BrickCoord& coord : runtime.active_hint_closure) {
        runtime.bricks.try_emplace(coord);
    }
    recompute_runtime_active_flags(runtime);
    prune_inactive_bricks(runtime);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_brick_world_runtime_status(
    long long service_key,
    long long world_key,
    int* out_status,
    int status_count
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_get_brick_world_runtime_status: missing service");
        return 0;
    }
    if (!out_status || status_count < AERO_LBM_SIMULATION_BRICK_RUNTIME_STATUS_FIELDS) {
        set_simulation_last_error("simulation_get_brick_world_runtime_status: invalid output buffer");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_get_brick_world_runtime_status: missing brick runtime");
        return 0;
    }
    const FluidWorldRuntime& runtime = runtime_iterator->second;
    out_status[0] = runtime.brick_size;
    out_status[1] = static_cast<int>(runtime.bricks.size());
    out_status[2] = count_active_hint_bricks(runtime);
    out_status[3] = count_active_bricks(runtime);
    out_status[4] = count_geometry_dirty_bricks(runtime);
    out_status[5] = count_forcing_dirty_bricks(runtime);
    out_status[6] = count_pending_reinit_bricks(runtime);
    out_status[7] = static_cast<int>(std::min<std::uint64_t>(
        runtime.epoch,
        static_cast<std::uint64_t>(std::numeric_limits<int>::max())
    ));
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_step_brick_world_runtime(
    long long service_key,
    long long world_key,
    int step_count
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_step_brick_world_runtime: missing service");
        return 0;
    }
    if (step_count <= 0) {
        set_simulation_last_error("simulation_step_brick_world_runtime: invalid step count");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_step_brick_world_runtime: missing brick runtime");
        return 0;
    }
    step_brick_world_runtime(*service, world_key, runtime_iterator->second, step_count);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_brick_world_resident_brick_count(
    long long service_key,
    long long world_key
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_get_brick_world_resident_brick_count: missing service");
        return -1;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        return 0;
    }
    return count_resident_bricks(runtime_iterator->second);
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_copy_brick_world_resident_bricks(
    long long service_key,
    long long world_key,
    int* out_coords,
    int brick_capacity
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_copy_brick_world_resident_bricks: missing service");
        return -1;
    }
    if (brick_capacity < 0) {
        set_simulation_last_error("simulation_copy_brick_world_resident_bricks: invalid brick capacity");
        return -1;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        return 0;
    }
    const int copied = copy_resident_bricks(runtime_iterator->second, out_coords, brick_capacity);
    if (copied < 0) {
        set_simulation_last_error("simulation_copy_brick_world_resident_bricks: output buffer too small");
    }
    return copied;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_brick_world_active_brick_count(
    long long service_key,
    long long world_key
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_get_brick_world_active_brick_count: missing service");
        return -1;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        return 0;
    }
    return count_active_window_bricks(runtime_iterator->second);
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_copy_brick_world_active_bricks(
    long long service_key,
    long long world_key,
    int* out_coords,
    int brick_capacity
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_copy_brick_world_active_bricks: missing service");
        return -1;
    }
    if (brick_capacity < 0) {
        set_simulation_last_error("simulation_copy_brick_world_active_bricks: invalid brick capacity");
        return -1;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        return 0;
    }
    const int copied = copy_active_bricks(runtime_iterator->second, out_coords, brick_capacity);
    if (copied < 0) {
        set_simulation_last_error("simulation_copy_brick_world_active_bricks: output buffer too small");
    }
    return copied;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_upload_brick_world_static_brick(
    long long service_key,
    long long world_key,
    int brick_size,
    int brick_x,
    int brick_y,
    int brick_z,
    const uint8_t* obstacle,
    const uint8_t* surface_kind,
    const uint16_t* open_face_mask,
    const float* emitter_power_watts,
    const uint8_t* face_sky_exposure,
    const uint8_t* face_direct_exposure
) {
    int cells = 0;
    if (!checked_cell_count(brick_size, brick_size, brick_size, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_brick_world_static_brick: invalid brick size");
        return 0;
    }
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_brick_world_static_brick: null brick buffers");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_upload_brick_world_static_brick: missing service");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_upload_brick_world_static_brick: missing brick runtime");
        return 0;
    }
    FluidWorldRuntime& runtime = runtime_iterator->second;
    if (runtime.brick_size != brick_size) {
        set_simulation_last_error("simulation_upload_brick_world_static_brick: brick size mismatch");
        return 0;
    }

    auto region = std::make_shared<StaticRegionData>();
    region->nx = brick_size;
    region->ny = brick_size;
    region->nz = brick_size;
    region->obstacle.assign(obstacle, obstacle + cells);
    region->surface_kind.assign(surface_kind, surface_kind + cells);
    region->open_face_mask.assign(open_face_mask, open_face_mask + cells);
    region->emitter_power_watts.assign(emitter_power_watts, emitter_power_watts + cells);
    region->face_sky_exposure.assign(face_sky_exposure, face_sky_exposure + static_cast<size_t>(cells) * k_face_count);
    region->face_direct_exposure.assign(face_direct_exposure, face_direct_exposure + static_cast<size_t>(cells) * k_face_count);

    BrickCoord coord{brick_x, brick_y, brick_z};
    BrickData& brick = runtime.bricks[coord];
    brick.static_region = std::move(region);
    brick.geometry_dirty = true;
    brick.pending_reinit = true;
    brick.active = true;
    brick.last_active_epoch = runtime.epoch;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_queue_brick_world_static_brick_upload(
    long long service_key,
    long long world_key,
    int brick_size,
    int brick_x,
    int brick_y,
    int brick_z,
    const uint8_t* obstacle,
    const uint8_t* surface_kind,
    const uint16_t* open_face_mask,
    const float* emitter_power_watts,
    const uint8_t* face_sky_exposure,
    const uint8_t* face_direct_exposure
) {
    int cells = 0;
    if (!checked_cell_count(brick_size, brick_size, brick_size, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_queue_brick_world_static_brick_upload: invalid brick size");
        return 0;
    }
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_queue_brick_world_static_brick_upload: null brick buffers");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_queue_brick_world_static_brick_upload: missing service");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_queue_brick_world_static_brick_upload: missing brick runtime");
        return 0;
    }
    FluidWorldRuntime& runtime = runtime_iterator->second;
    if (runtime.brick_size != brick_size) {
        set_simulation_last_error("simulation_queue_brick_world_static_brick_upload: brick size mismatch");
        return 0;
    }

    auto region = std::make_shared<StaticRegionData>();
    region->nx = brick_size;
    region->ny = brick_size;
    region->nz = brick_size;
    region->obstacle.assign(obstacle, obstacle + cells);
    region->surface_kind.assign(surface_kind, surface_kind + cells);
    region->open_face_mask.assign(open_face_mask, open_face_mask + cells);
    region->emitter_power_watts.assign(emitter_power_watts, emitter_power_watts + cells);
    region->face_sky_exposure.assign(face_sky_exposure, face_sky_exposure + static_cast<size_t>(cells) * k_face_count);
    region->face_direct_exposure.assign(face_direct_exposure, face_direct_exposure + static_cast<size_t>(cells) * k_face_count);

    int upload_id = service->next_pending_static_brick_upload_id++;
    if (upload_id <= 0) {
        upload_id = service->next_pending_static_brick_upload_id = 1;
    }
    service->pending_static_brick_uploads[upload_id] = PendingStaticBrickUpload{
        world_key,
        BrickCoord{brick_x, brick_y, brick_z},
        std::move(region)
    };
    runtime.pending_world_deltas.push_back(AeroLbmWorldDelta{
        k_world_delta_brick_static_brick_upload,
        brick_x,
        brick_y,
        brick_z,
        static_cast<int>(world_key),
        upload_id,
        0,
        0,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    });
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_sync_region_core_to_brick_world(
    long long service_key,
    long long region_key,
    long long world_key,
    int region_nx,
    int region_ny,
    int region_nz,
    int core_offset_x,
    int core_offset_y,
    int core_offset_z,
    int core_nx,
    int core_ny,
    int core_nz,
    int brick_x,
    int brick_y,
    int brick_z
) {
    int region_cells = 0;
    int core_cells = 0;
    if (!checked_cell_count(region_nx, region_ny, region_nz, &region_cells)
        || !checked_cell_count(core_nx, core_ny, core_nz, &core_cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: invalid dimensions");
        return 0;
    }
    if (!region_core_bounds_valid(
        region_nx,
        region_ny,
        region_nz,
        core_offset_x,
        core_offset_y,
        core_offset_z,
        core_nx,
        core_ny,
        core_nz
    )) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: invalid core bounds");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: missing service");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: missing brick runtime");
        return 0;
    }
    FluidWorldRuntime& runtime = runtime_iterator->second;
    if (runtime.brick_size != core_nx || runtime.brick_size != core_ny || runtime.brick_size != core_nz) {
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: brick size mismatch");
        return 0;
    }
    auto dynamic_region_iterator = service->dynamic_regions.find(region_key);
    if (dynamic_region_iterator == service->dynamic_regions.end()) {
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: missing dynamic region");
        return 0;
    }
    const DynamicRegionData& source_dynamic = dynamic_region_iterator->second;
    if (source_dynamic.nx != region_nx || source_dynamic.ny != region_ny || source_dynamic.nz != region_nz) {
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: region dimension mismatch");
        return 0;
    }
    if (aero_lbm_has_context(region_key) == 0) {
        set_simulation_last_error("simulation_sync_region_core_to_brick_world: missing native context");
        return 0;
    }

    std::vector<float> core_flow(static_cast<size_t>(core_cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS);
    std::vector<float> core_air_temperature(static_cast<size_t>(core_cells));
    if (!aero_lbm_copy_flow_temperature_subrect(
        region_nx,
        region_ny,
        region_nz,
        region_key,
        core_offset_x,
        core_offset_y,
        core_offset_z,
        core_nx,
        core_ny,
        core_nz,
        core_flow.data(),
        core_air_temperature.data()
    )) {
        set_simulation_last_error(std::string("simulation_sync_region_core_to_brick_world core sync failed: ") + aero_lbm_last_error());
        return 0;
    }

    auto brick_dynamic = std::make_shared<DynamicRegionData>();
    brick_dynamic->nx = core_nx;
    brick_dynamic->ny = core_ny;
    brick_dynamic->nz = core_nz;
    brick_dynamic->flow_state = std::move(core_flow);
    brick_dynamic->air_temperature = std::move(core_air_temperature);
    brick_dynamic->surface_temperature.resize(static_cast<size_t>(core_cells));

    for (int x = 0; x < core_nx; ++x) {
        for (int y = 0; y < core_ny; ++y) {
            for (int z = 0; z < core_nz; ++z) {
                const int src_x = core_offset_x + x;
                const int src_y = core_offset_y + y;
                const int src_z = core_offset_z + z;
                const int src_cell = grid_cell_index(region_ny, region_nz, src_x, src_y, src_z);
                const int dst_cell = grid_cell_index(core_ny, core_nz, x, y, z);
                brick_dynamic->surface_temperature[static_cast<size_t>(dst_cell)] =
                    source_dynamic.surface_temperature[static_cast<size_t>(src_cell)];
            }
        }
    }

    BrickData& brick = runtime.bricks[BrickCoord{brick_x, brick_y, brick_z}];
    brick.dynamic_region = std::move(brick_dynamic);
    brick.active = true;
    brick.last_active_epoch = runtime.epoch;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_copy_brick_world_dynamic_brick(
    long long service_key,
    long long world_key,
    int brick_size,
    int brick_x,
    int brick_y,
    int brick_z,
    float* out_flow_state,
    float* out_air_temperature,
    float* out_surface_temperature
) {
    int cells = 0;
    if (!checked_cell_count(brick_size, brick_size, brick_size, &cells)
        || !out_flow_state
        || !out_air_temperature
        || !out_surface_temperature) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_copy_brick_world_dynamic_brick: invalid arguments");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_copy_brick_world_dynamic_brick: missing service");
        return 0;
    }
    auto runtime_iterator = service->brick_world_runtimes.find(world_key);
    if (runtime_iterator == service->brick_world_runtimes.end()) {
        set_simulation_last_error("simulation_copy_brick_world_dynamic_brick: missing brick runtime");
        return 0;
    }
    const FluidWorldRuntime& runtime = runtime_iterator->second;
    if (runtime.brick_size != brick_size) {
        set_simulation_last_error("simulation_copy_brick_world_dynamic_brick: brick size mismatch");
        return 0;
    }
    auto brick_iterator = runtime.bricks.find(BrickCoord{brick_x, brick_y, brick_z});
    if (brick_iterator == runtime.bricks.end() || !brick_iterator->second.dynamic_region) {
        return 0;
    }
    const DynamicRegionData& dynamic = *brick_iterator->second.dynamic_region;
    if (!brick_dynamic_region_valid(runtime, &dynamic)) {
        set_simulation_last_error("simulation_copy_brick_world_dynamic_brick: invalid stored brick dynamic state");
        return 0;
    }
    std::copy(dynamic.flow_state.begin(), dynamic.flow_state.end(), out_flow_state);
    std::copy(dynamic.air_temperature.begin(), dynamic.air_temperature.end(), out_air_temperature);
    std::copy(dynamic.surface_temperature.begin(), dynamic.surface_temperature.end(), out_surface_temperature);
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
    const float* emitter_power_watts,
    const uint8_t* face_sky_exposure,
    const uint8_t* face_direct_exposure
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_static_region: invalid dimensions");
        return 0;
    }
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
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

    auto region = std::make_shared<StaticRegionData>();
    region->nx = nx;
    region->ny = ny;
    region->nz = nz;
    region->obstacle.assign(obstacle, obstacle + cells);
    region->surface_kind.assign(surface_kind, surface_kind + cells);
    region->open_face_mask.assign(open_face_mask, open_face_mask + cells);
    region->emitter_power_watts.assign(emitter_power_watts, emitter_power_watts + cells);
    region->face_sky_exposure.assign(face_sky_exposure, face_sky_exposure + static_cast<size_t>(cells) * k_face_count);
    region->face_direct_exposure.assign(face_direct_exposure, face_direct_exposure + static_cast<size_t>(cells) * k_face_count);
    service->static_regions[region_key] = std::move(region);
    rebuild_region_packet_template(*service, region_key);
    return 1;
}

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
) {
    int patch_cells = 0;
    if (!checked_cell_count(patch_nx, patch_ny, patch_nz, &patch_cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_static_region_patch: invalid patch dimensions");
        return 0;
    }
    if (offset_x < 0 || offset_y < 0 || offset_z < 0
        || patch_nx <= 0 || patch_ny <= 0 || patch_nz <= 0
        || offset_x + patch_nx > nx
        || offset_y + patch_ny > ny
        || offset_z + patch_nz > nz) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_static_region_patch: patch out of bounds");
        return 0;
    }
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_static_region_patch: null patch buffers");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_upload_static_region_patch: missing service");
        return 0;
    }
    auto region_it = service->static_regions.find(region_key);
    if (region_it == service->static_regions.end()) {
        set_simulation_last_error("simulation_upload_static_region_patch: missing static region");
        return 0;
    }
    StaticRegionData region = *region_it->second;
    if (region.nx != nx || region.ny != ny || region.nz != nz) {
        set_simulation_last_error("simulation_upload_static_region_patch: region dimension mismatch");
        return 0;
    }

    for (int px = 0; px < patch_nx; ++px) {
        for (int py = 0; py < patch_ny; ++py) {
            for (int pz = 0; pz < patch_nz; ++pz) {
                const int patch_cell = grid_cell_index(patch_ny, patch_nz, px, py, pz);
                const int cell = grid_cell_index(
                    region.ny,
                    region.nz,
                    offset_x + px,
                    offset_y + py,
                    offset_z + pz
                );
                region.obstacle[static_cast<size_t>(cell)] = obstacle[patch_cell];
                region.surface_kind[static_cast<size_t>(cell)] = surface_kind[patch_cell];
                region.open_face_mask[static_cast<size_t>(cell)] = open_face_mask[patch_cell];
                region.emitter_power_watts[static_cast<size_t>(cell)] = emitter_power_watts[patch_cell];
                const size_t face_base = static_cast<size_t>(cell) * k_face_count;
                const size_t patch_face_base = static_cast<size_t>(patch_cell) * k_face_count;
                std::copy_n(face_sky_exposure + patch_face_base, k_face_count, region.face_sky_exposure.begin() + static_cast<std::ptrdiff_t>(face_base));
                std::copy_n(face_direct_exposure + patch_face_base, k_face_count, region.face_direct_exposure.begin() + static_cast<std::ptrdiff_t>(face_base));
            }
        }
    }
    service->static_regions[region_key] = std::make_shared<StaticRegionData>(std::move(region));
    rebuild_region_packet_template(*service, region_key);
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
    ensure_region_buffers(*service, region_key, nx, ny, nz, cells);
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
    return service->static_regions.find(region_key) != service->static_regions.end()
        && service->forcing_regions.find(region_key) != service->forcing_regions.end()
        && service->dynamic_regions.find(region_key) != service->dynamic_regions.end();
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

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_set_region_nested_feedback_layout(
    long long service_key,
    long long region_key,
    int steps_per_feedback,
    const int* layout_values,
    int value_count
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_set_region_nested_feedback_layout: missing service");
        return 0;
    }
    auto lifecycle_it = service->regions.find(region_key);
    if (lifecycle_it == service->regions.end()) {
        set_simulation_last_error("simulation_set_region_nested_feedback_layout: missing region lifecycle");
        return 0;
    }
    int cells = 0;
    if (!checked_cell_count(lifecycle_it->second.nx, lifecycle_it->second.ny, lifecycle_it->second.nz, &cells)) {
        set_simulation_last_error("simulation_set_region_nested_feedback_layout: invalid region dimensions");
        return 0;
    }
    ensure_region_buffers(
        *service,
        region_key,
        lifecycle_it->second.nx,
        lifecycle_it->second.ny,
        lifecycle_it->second.nz,
        cells
    );
    DynamicRegionData& dynamic = service->dynamic_regions[region_key];
    return configure_nested_feedback_layout(dynamic, steps_per_feedback, layout_values, value_count) ? 1 : 0;
}

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
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)
        || !fan_mask
        || !fan_vx
        || !fan_vy
        || !fan_vz) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_upload_region_forcing: invalid arguments");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_upload_region_forcing: missing service");
        return 0;
    }
    auto forcing = std::make_shared<ForcingRegionData>();
    forcing->nx = nx;
    forcing->ny = ny;
    forcing->nz = nz;
    if (!thermal_source) {
        forcing->thermal_source.assign(static_cast<size_t>(cells), 0.0f);
    } else {
        forcing->thermal_source.assign(thermal_source, thermal_source + cells);
    }
    forcing->fan_mask.assign(fan_mask, fan_mask + cells);
    forcing->fan_vx.assign(fan_vx, fan_vx + cells);
    forcing->fan_vy.assign(fan_vy, fan_vy + cells);
    forcing->fan_vz.assign(fan_vz, fan_vz + cells);
    service->forcing_regions[region_key] = std::move(forcing);
    rebuild_region_packet_template(*service, region_key);
    return 1;
}

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
) {
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_refresh_region_thermal: missing service");
        return 0;
    }
    return refresh_region_thermal(
        *service,
        region_key,
        nx,
        ny,
        nz,
        direct_solar_flux_w_m2,
        diffuse_solar_flux_w_m2,
        ambient_air_temperature_k,
        deep_ground_temperature_k,
        sky_temperature_k,
        precipitation_temperature_k,
        precipitation_strength,
        sun_x,
        sun_y,
        sun_z,
        surface_delta_seconds
    ) ? 1 : 0;
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
    if (!ensure_l2_runtime(*service, nx, ny, nz, 11, 4)) {
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
    if (!sync_dynamic_region_temperature_from_native(region_key, dynamic)) {
        set_simulation_last_error(std::string("simulation_step_region temperature sync failed: ") + aero_lbm_last_error());
        return 0;
    }
    rebuild_default_packed_atlas(dynamic, service->atlases[region_key]);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_step_region_stored(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float boundary_wind_x,
    float boundary_wind_y,
    float boundary_wind_z,
    float fallback_boundary_air_temperature_k,
    int external_face_mask,
    int boundary_face_resolution,
    const float* boundary_wind_face_x,
    const float* boundary_wind_face_y,
    const float* boundary_wind_face_z,
    const float* boundary_air_temperature_k,
    int sponge_thickness_cells,
    float sponge_velocity_relaxation,
    float sponge_temperature_relaxation,
    int tornado_descriptor_count,
    const float* tornado_descriptors,
    float* out_max_speed
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells) || !out_max_speed) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_step_region_stored: invalid arguments");
        return 0;
    }

    const bool context_ready = aero_lbm_has_context(region_key) != 0;
    DynamicRegionData dynamic_region;
    std::shared_ptr<const RegionPacketTemplateData> packet_template;
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_step_region_stored: missing service");
            return 0;
        }
        auto lifecycle_it = service->regions.find(region_key);
        if (lifecycle_it == service->regions.end() || !lifecycle_it->second.active) {
            set_simulation_last_error("simulation_step_region_stored: inactive region");
            return 0;
        }
        if (!ensure_l2_runtime(*service, nx, ny, nz, 11, 4)) {
            return 0;
        }
        auto packet_it = service->packet_templates.find(region_key);
        if (packet_it == service->packet_templates.end()) {
            set_simulation_last_error("simulation_step_region_stored: missing packet template");
            return 0;
        }
        auto dynamic_it = service->dynamic_regions.find(region_key);
        if (dynamic_it == service->dynamic_regions.end()) {
            DynamicRegionData& dynamic = service->dynamic_regions[region_key];
            dynamic.nx = nx;
            dynamic.ny = ny;
            dynamic.nz = nz;
            dynamic.flow_state.assign(static_cast<size_t>(cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS, 0.0f);
            dynamic.air_temperature.assign(static_cast<size_t>(cells), 0.0f);
            dynamic.surface_temperature.assign(static_cast<size_t>(cells), 0.0f);
            dynamic_it = service->dynamic_regions.find(region_key);
        }
        packet_template = packet_it->second;
        if (context_ready) {
            // The native context is authoritative on the hot path. Only carry mutable
            // coarse-side fields that are still consumed outside the solver step.
            dynamic_region.nx = dynamic_it->second.nx;
            dynamic_region.ny = dynamic_it->second.ny;
            dynamic_region.nz = dynamic_it->second.nz;
            dynamic_region.surface_temperature = dynamic_it->second.surface_temperature;
            dynamic_region.nested_feedback = dynamic_it->second.nested_feedback;
        } else {
            dynamic_region = dynamic_it->second;
        }
    }
    thread_local std::vector<float> packet;
    if (!packet_template
        || packet_template->values.size() != static_cast<size_t>(cells) * k_packet_channels) {
        set_simulation_last_error("simulation_step_region_stored: invalid packet template");
        return 0;
    }
    const bool needs_dynamic_seed = !context_ready;
    const bool has_boundary_override = boundary_face_resolution > 0 && external_face_mask != 0;
    const bool has_global_edge_stabilization = sponge_thickness_cells <= 0 && external_face_mask != 0;
    const bool has_tornado_override = tornado_descriptor_count > 0 && tornado_descriptors != nullptr;
    const bool needs_materialized_packet = needs_dynamic_seed;
    const float* packet_data = packet_template->values.data();
    if (needs_materialized_packet) {
        if (!build_region_packet_from_template(
            *packet_template,
            dynamic_region,
            needs_dynamic_seed,
            nx,
            ny,
            nz,
            boundary_wind_x,
            boundary_wind_y,
            boundary_wind_z,
            fallback_boundary_air_temperature_k,
            external_face_mask,
            boundary_face_resolution,
            boundary_wind_face_x,
            boundary_wind_face_y,
            boundary_wind_face_z,
            boundary_air_temperature_k,
            sponge_thickness_cells,
            tornado_descriptor_count,
            tornado_descriptors,
            packet
        )) {
            return 0;
        }
        if (has_boundary_override) {
            if (sponge_thickness_cells > 0) {
                apply_sponge_boundary_fields(
                    packet,
                    nx,
                    ny,
                    nz,
                    boundary_wind_x,
                    boundary_wind_y,
                    boundary_wind_z,
                    fallback_boundary_air_temperature_k,
                    external_face_mask,
                    boundary_face_resolution,
                    boundary_wind_face_x,
                    boundary_wind_face_y,
                    boundary_wind_face_z,
                    boundary_air_temperature_k,
                    sponge_thickness_cells,
                    sponge_velocity_relaxation,
                    sponge_temperature_relaxation
                );
            } else {
                apply_nested_boundary_fields(
                    packet,
                    nx,
                    ny,
                    nz,
                    boundary_wind_x,
                    boundary_wind_y,
                    boundary_wind_z,
                    fallback_boundary_air_temperature_k,
                    external_face_mask,
                    boundary_face_resolution,
                    boundary_wind_face_x,
                    boundary_wind_face_y,
                    boundary_wind_face_z,
                    boundary_air_temperature_k
                );
            }
        }
        packet_data = packet.data();
    }
    thread_local SparsePacketOverlayBuilder overlay_builder;
    overlay_builder.reset(cells);
    if (!needs_materialized_packet) {
        if (has_global_edge_stabilization) {
            apply_boundary_wind_overlay(
                overlay_builder,
                packet_data,
                nx,
                ny,
                nz,
                boundary_wind_x,
                boundary_wind_y,
                boundary_wind_z
            );
        }
        if (has_tornado_override) {
            apply_tornado_vortex_descriptor_overlay(
                overlay_builder,
                packet_data,
                nx,
                ny,
                nz,
                tornado_descriptor_count,
                tornado_descriptors
            );
        }
        if (has_boundary_override) {
            if (sponge_thickness_cells > 0) {
                apply_sponge_boundary_field_overlay(
                    overlay_builder,
                    packet_data,
                    nx,
                    ny,
                    nz,
                    boundary_wind_x,
                    boundary_wind_y,
                    boundary_wind_z,
                    fallback_boundary_air_temperature_k,
                    external_face_mask,
                    boundary_face_resolution,
                    boundary_wind_face_x,
                    boundary_wind_face_y,
                    boundary_wind_face_z,
                    boundary_air_temperature_k,
                    sponge_thickness_cells,
                    sponge_velocity_relaxation,
                    sponge_temperature_relaxation
                );
            } else {
                apply_nested_boundary_field_overlay(
                    overlay_builder,
                    packet_data,
                    nx,
                    ny,
                    nz,
                    boundary_wind_x,
                    boundary_wind_y,
                    boundary_wind_z,
                    fallback_boundary_air_temperature_k,
                    external_face_mask,
                    boundary_face_resolution,
                    boundary_wind_face_x,
                    boundary_wind_face_y,
                    boundary_wind_face_z,
                    boundary_air_temperature_k
                );
            }
        }
    }
    // Hot path keeps the live L2 state inside the native context. Full flow-state
    // caches are refreshed only on explicit sync/export paths.
    const bool step_ok = overlay_builder.cells.empty()
        ? aero_lbm_step_rect(packet_data, nx, ny, nz, region_key, nullptr) != 0
        : aero_lbm_step_rect_with_sparse_overlays(
            packet_data,
            nx,
            ny,
            nz,
            region_key,
            overlay_builder.cells.data(),
            overlay_builder.values.data(),
            static_cast<int>(overlay_builder.cells.size()),
            nullptr
        ) != 0;
    if (!step_ok) {
        set_simulation_last_error(std::string("simulation_step_region_stored failed: ") + aero_lbm_last_error());
        return 0;
    }
    const int atlas_sx = (nx + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int atlas_sy = (ny + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int atlas_sz = (nz + k_default_packed_atlas_stride - 1) / k_default_packed_atlas_stride;
    const int atlas_cells = atlas_sx * atlas_sy * atlas_sz;
    thread_local std::vector<float> atlas_flow;
    atlas_flow.assign(static_cast<size_t>(atlas_cells) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS, 0.0f);
    if (!aero_lbm_extract_flow_atlas_rect(
        nx,
        ny,
        nz,
        region_key,
        k_default_packed_atlas_stride,
        atlas_flow.data(),
        static_cast<int>(atlas_flow.size())
    )) {
        set_simulation_last_error(std::string("simulation_step_region_stored atlas extract failed: ") + aero_lbm_last_error());
        return 0;
    }
    AtlasData atlas;
    // Region max speed now tracks the published stride atlas rather than a full-field
    // readback; this keeps the publish path cheap at the cost of being a sampled max.
    const float max_speed = rebuild_default_packed_atlas_from_flow_samples(atlas_flow.data(), atlas_cells, atlas);
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_step_region_stored: missing service after step");
            return 0;
        }
        DynamicRegionData& stored = service->dynamic_regions[region_key];
        stored.surface_temperature = std::move(dynamic_region.surface_temperature);
        stored.nested_feedback = std::move(dynamic_region.nested_feedback);
        service->atlases[region_key] = std::move(atlas);
    }
    *out_max_speed = max_speed;
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
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_exchange_region_halo: missing service");
            return 0;
        }
    }
    if (!aero_lbm_exchange_halo(grid_size, first_region_key, second_region_key, offset_x, offset_y, offset_z)) {
        set_simulation_last_error(std::string("simulation_exchange_region_halo failed: ") + aero_lbm_last_error());
        return 0;
    }
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_exchange_region_halo_batch(
    long long service_key,
    const long long* region_pairs,
    const int* offsets,
    int exchange_count,
    int* out_applied_count
) {
    if (!out_applied_count) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_exchange_region_halo_batch: out_applied_count is null");
        return 0;
    }
    *out_applied_count = 0;
    if (exchange_count < 0) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_exchange_region_halo_batch: negative exchange_count");
        return 0;
    }
    if (exchange_count == 0) {
        return 1;
    }
    if (!region_pairs || !offsets) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_exchange_region_halo_batch: null exchange buffers");
        return 0;
    }
    int grid_size = 0;
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_exchange_region_halo_batch: missing service");
            return 0;
        }
        grid_size = service->l2_nx;
    }
    int applied_count = 0;
    for (int index = 0; index < exchange_count; ++index) {
        const long long first_region_key = region_pairs[index * 2];
        const long long second_region_key = region_pairs[index * 2 + 1];
        const int offset_x = offsets[index * 3];
        const int offset_y = offsets[index * 3 + 1];
        const int offset_z = offsets[index * 3 + 2];
        if (first_region_key == 0L || second_region_key == 0L) {
            continue;
        }
        if (!aero_lbm_has_context(first_region_key) || !aero_lbm_has_context(second_region_key)) {
            continue;
        }
        if (!aero_lbm_exchange_halo(grid_size, first_region_key, second_region_key, offset_x, offset_y, offset_z)) {
            set_simulation_last_error(
                std::string("simulation_exchange_region_halo_batch failed at exchange ")
                    + std::to_string(index) + ": " + aero_lbm_last_error()
            );
            return 0;
        }
        applied_count++;
    }
    *out_applied_count = applied_count;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_sync_region_state(
    long long service_key,
    long long region_key,
    float* out_max_speed
) {
    if (!out_max_speed) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_sync_region_state: out_max_speed is null");
        return 0;
    }

    DynamicRegionData dynamic_region;
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_sync_region_state: missing service");
            return 0;
        }
        auto dynamic_it = service->dynamic_regions.find(region_key);
        if (dynamic_it == service->dynamic_regions.end()) {
            set_simulation_last_error("simulation_sync_region_state: missing dynamic region");
            return 0;
        }
        dynamic_region = dynamic_it->second;
    }
    AtlasData atlas;
    float max_speed = sync_dynamic_region_from_native(region_key, dynamic_region, &atlas);
    if (max_speed < 0.0f) {
        set_simulation_last_error(std::string("simulation_sync_region_state failed: ") + aero_lbm_last_error());
        return 0;
    }
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_sync_region_state: missing service after sync");
            return 0;
        }
        service->dynamic_regions[region_key] = std::move(dynamic_region);
        service->atlases[region_key] = std::move(atlas);
    }
    *out_max_speed = max_speed;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_sync_region_flow_state(
    long long service_key,
    long long region_key,
    float* out_max_speed
) {
    if (!out_max_speed) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_sync_region_flow_state: out_max_speed is null");
        return 0;
    }

    DynamicRegionData dynamic_region;
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_sync_region_flow_state: missing service");
            return 0;
        }
        auto dynamic_it = service->dynamic_regions.find(region_key);
        if (dynamic_it == service->dynamic_regions.end()) {
            set_simulation_last_error("simulation_sync_region_flow_state: missing dynamic region");
            return 0;
        }
        dynamic_region = dynamic_it->second;
    }
    AtlasData atlas;
    float max_speed = 0.0f;
    if (!sync_dynamic_region_flow_from_native(region_key, dynamic_region, &atlas, &max_speed)) {
        set_simulation_last_error(std::string("simulation_sync_region_flow_state failed: ") + aero_lbm_last_error());
        return 0;
    }
    {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        ServiceState* service = lookup_service(service_key);
        if (!service) {
            set_simulation_last_error("simulation_sync_region_flow_state: missing service after sync");
            return 0;
        }
        service->dynamic_regions[region_key].flow_state = std::move(dynamic_region.flow_state);
        service->atlases[region_key] = std::move(atlas);
    }
    *out_max_speed = max_speed;
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
    if (!ensure_l2_runtime(*service, nx, ny, nz, 11, 4)) {
        return 0;
    }
    if (!aero_lbm_get_temperature_state_rect(nx, ny, nz, region_key, out_temperature)) {
        set_simulation_last_error(std::string("simulation_get_region_temperature_state failed: ") + aero_lbm_last_error());
        return 0;
    }
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_region_flow_state(
    long long service_key,
    long long region_key,
    int nx,
    int ny,
    int nz,
    float* out_flow_state
) {
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells) || !out_flow_state) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_get_region_flow_state: invalid arguments");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_get_region_flow_state: missing service");
        return 0;
    }
    if (!ensure_l2_runtime(*service, nx, ny, nz, 11, 4)) {
        return 0;
    }
    if (!aero_lbm_get_flow_state_rect(nx, ny, nz, region_key, out_flow_state)) {
        set_simulation_last_error(std::string("simulation_get_region_flow_state failed: ") + aero_lbm_last_error());
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
    if (!ensure_l2_runtime(*service, nx, ny, nz, 11, 4)) {
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
    reset_nested_feedback_progress(region.nested_feedback);
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
    DynamicRegionData& region = region_it->second;
    if (region.nx != nx || region.ny != ny || region.nz != nz) {
        set_simulation_last_error("simulation_export_dynamic_region: dimension mismatch");
        return 0;
    }

    // Hot runtime stepping no longer requires Java-side flow-state mirrors to be
    // synchronized after every seam exchange. Export remains a cold path, so it
    // can afford an on-demand sync from the authoritative native context.
    if (aero_lbm_has_context(region_key)) {
        if (!sync_dynamic_region_flow_from_native(region_key, region, nullptr, nullptr)) {
            set_simulation_last_error(std::string("simulation_export_dynamic_region sync failed: ") + aero_lbm_last_error());
            return 0;
        }
        if (!sync_dynamic_region_temperature_from_native(region_key, region)) {
            set_simulation_last_error(std::string("simulation_export_dynamic_region temperature sync failed: ") + aero_lbm_last_error());
            return 0;
        }
    }

    std::copy(region.flow_state.begin(), region.flow_state.end(), out_flow_state);
    std::copy(region.air_temperature.begin(), region.air_temperature.end(), out_air_temperature);
    std::copy(region.surface_temperature.begin(), region.surface_temperature.end(), out_surface_temperature);
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_poll_region_nested_feedback(
    long long service_key,
    long long region_key,
    float* out_values,
    int value_count
) {
    if (!out_values || value_count <= 0) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_poll_region_nested_feedback: invalid output buffer");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_poll_region_nested_feedback: missing service");
        return 0;
    }
    auto region_it = service->dynamic_regions.find(region_key);
    if (region_it == service->dynamic_regions.end()) {
        set_simulation_last_error("simulation_poll_region_nested_feedback: missing region");
        return 0;
    }
    DynamicRegionData::NestedFeedbackData& nested_feedback = region_it->second.nested_feedback;
    if (nested_feedback.ready_values.empty()) {
        return 0;
    }
    if (static_cast<int>(nested_feedback.ready_values.size()) != value_count) {
        set_simulation_last_error("simulation_poll_region_nested_feedback: size mismatch");
        return 0;
    }
    std::copy(nested_feedback.ready_values.begin(), nested_feedback.ready_values.end(), out_values);
    nested_feedback.ready_values.clear();
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_simulation_get_region_nested_feedback_status(
    long long service_key,
    long long region_key,
    int* out_status,
    int status_count
) {
    if (!out_status || status_count != k_nested_feedback_status_fields) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_get_region_nested_feedback_status: invalid output buffer");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_get_region_nested_feedback_status: missing service");
        return 0;
    }
    auto region_it = service->dynamic_regions.find(region_key);
    if (region_it == service->dynamic_regions.end()) {
        set_simulation_last_error("simulation_get_region_nested_feedback_status: missing region");
        return 0;
    }
    const DynamicRegionData::NestedFeedbackData& nested_feedback = region_it->second.nested_feedback;
    out_status[0] = static_cast<int>(nested_feedback.layout.size());
    out_status[1] = nested_feedback.steps_per_feedback;
    out_status[2] = nested_feedback.steps_accumulated;
    out_status[3] = nested_feedback.ready_values.empty() ? 0 : static_cast<int>(nested_feedback.layout.size());
    out_status[4] = nested_feedback.packets_emitted;
    out_status[5] = nested_feedback.reset_count;
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
) {
    if (value_count != AERO_LBM_SIMULATION_PLAYER_PROBE_CHANNELS || !out_probe_values) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_sample_region_point: invalid output buffer");
        return 0;
    }
    int cells = 0;
    if (!checked_cell_count(nx, ny, nz, &cells)) {
        std::lock_guard<SpinMutex> lock(g_simulation_mutex);
        set_simulation_last_error("simulation_sample_region_point: invalid dimensions");
        return 0;
    }

    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    ServiceState* service = lookup_service(service_key);
    if (!service) {
        set_simulation_last_error("simulation_sample_region_point: missing service");
        return 0;
    }
    auto region_it = service->dynamic_regions.find(region_key);
    if (region_it == service->dynamic_regions.end()) {
        set_simulation_last_error("simulation_sample_region_point: missing region");
        return 0;
    }
    const DynamicRegionData& region = region_it->second;
    if (region.nx != nx || region.ny != ny || region.nz != nz) {
        set_simulation_last_error("simulation_sample_region_point: dimension mismatch");
        return 0;
    }
    if (!in_bounds(nx, ny, nz, sample_x, sample_y, sample_z)) {
        set_simulation_last_error("simulation_sample_region_point: sample out of bounds");
        return 0;
    }
    const int cell = grid_cell_index(ny, nz, sample_x, sample_y, sample_z);
    if (region.surface_temperature.size() <= static_cast<size_t>(cell)) {
        set_simulation_last_error("simulation_sample_region_point: region buffers incomplete");
        return 0;
    }
    const size_t flow_base = static_cast<size_t>(cell) * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS;
    float sampled_flow[AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool flow_valid = false;
    float sampled_air_temperature = 0.0f;
    bool air_temperature_valid = false;
    if (aero_lbm_has_context(region_key) != 0) {
        flow_valid = aero_lbm_sample_flow_point_rect(
            nx,
            ny,
            nz,
            region_key,
            sample_x,
            sample_y,
            sample_z,
            sampled_flow
        ) != 0;
        air_temperature_valid = aero_lbm_sample_temperature_point_rect(
            nx,
            ny,
            nz,
            region_key,
            sample_x,
            sample_y,
            sample_z,
            &sampled_air_temperature
        ) != 0;
    }
    if (!flow_valid) {
        if (region.flow_state.size() < flow_base + AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS) {
            set_simulation_last_error("simulation_sample_region_point: region buffers incomplete");
            return 0;
        }
        sampled_flow[0] = region.flow_state[flow_base];
        sampled_flow[1] = region.flow_state[flow_base + 1];
        sampled_flow[2] = region.flow_state[flow_base + 2];
        sampled_flow[3] = region.flow_state[flow_base + 3];
    }
    if (!air_temperature_valid) {
        if (region.air_temperature.size() <= static_cast<size_t>(cell)) {
            set_simulation_last_error("simulation_sample_region_point: region buffers incomplete");
            return 0;
        }
        sampled_air_temperature = region.air_temperature[static_cast<size_t>(cell)];
    }
    out_probe_values[0] = sampled_flow[0];
    out_probe_values[1] = sampled_flow[1];
    out_probe_values[2] = sampled_flow[2];
    out_probe_values[3] = sampled_flow[3];
    out_probe_values[4] = sampled_air_temperature;
    out_probe_values[5] = region.surface_temperature[static_cast<size_t>(cell)];
    return 1;
}

AERO_LBM_CAPI_EXPORT const char* aero_lbm_simulation_runtime_info(void) {
    static std::string text;
    std::lock_guard<SpinMutex> lock(g_simulation_mutex);
    size_t active_region_count = 0;
    size_t static_region_count = 0;
    size_t dynamic_region_count = 0;
    size_t atlas_count = 0;
    size_t brick_world_count = 0;
    size_t brick_count = 0;
    size_t active_brick_count = 0;
    std::uint64_t max_brick_epoch = 0;
    if (g_service) {
        for (const auto& region : g_service->regions) {
            if (region.second.active) {
                ++active_region_count;
            }
        }
        static_region_count = g_service->static_regions.size();
        dynamic_region_count = g_service->dynamic_regions.size();
        atlas_count = g_service->atlases.size();
        brick_world_count = g_service->brick_world_runtimes.size();
        for (const auto& entry : g_service->brick_world_runtimes) {
            brick_count += entry.second.bricks.size();
            active_brick_count += static_cast<size_t>(count_active_bricks(entry.second));
            max_brick_epoch = std::max(max_brick_epoch, entry.second.epoch);
        }
    }
    text = "simulation_bridge|services=" + std::to_string(g_service ? 1 : 0)
        + "|active_regions=" + std::to_string(active_region_count)
        + "|static_regions=" + std::to_string(static_region_count)
        + "|dynamic_regions=" + std::to_string(dynamic_region_count)
        + "|atlases=" + std::to_string(atlas_count)
        + "|brick_worlds=" + std::to_string(brick_world_count)
        + "|bricks=" + std::to_string(brick_count)
        + "|active_bricks=" + std::to_string(active_brick_count)
        + "|brick_epoch_max=" + std::to_string(max_brick_epoch);
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeEnsureBrickWorldRuntime(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong world_key,
    jint brick_size,
    jfloat dx_meters,
    jfloat dt_seconds
) {
    return aero_lbm_simulation_ensure_brick_world_runtime(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        brick_size,
        dx_meters,
        dt_seconds
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSetBrickWorldActiveHints(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key,
    jint brick_size,
    jintArray brick_coords,
    jint brick_count
) {
    if (!brick_coords) {
        return JNI_FALSE;
    }
    if (brick_count < 0 || env->GetArrayLength(brick_coords) < brick_count * 3) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jint* coords_ptr = env->GetIntArrayElements(brick_coords, &copy);
    if (!coords_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_set_brick_world_active_hints(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        brick_size,
        reinterpret_cast<const int*>(coords_ptr),
        brick_count
    );
    env->ReleaseIntArrayElements(brick_coords, coords_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeGetBrickWorldRuntimeStatus(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key,
    jintArray out_status
) {
    if (!out_status || env->GetArrayLength(out_status) != AERO_LBM_SIMULATION_BRICK_RUNTIME_STATUS_FIELDS) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jint* status_ptr = env->GetIntArrayElements(out_status, &copy);
    if (!status_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_get_brick_world_runtime_status(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        reinterpret_cast<int*>(status_ptr),
        AERO_LBM_SIMULATION_BRICK_RUNTIME_STATUS_FIELDS
    );
    env->ReleaseIntArrayElements(out_status, status_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeStepBrickWorldRuntime(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong world_key,
    jint step_count
) {
    return aero_lbm_simulation_step_brick_world_runtime(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        step_count
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jintArray JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeGetBrickWorldResidentBrickCoords(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key
) {
    const int brick_count = aero_lbm_simulation_get_brick_world_resident_brick_count(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key)
    );
    if (brick_count < 0) {
        return nullptr;
    }
    jintArray result = env->NewIntArray(brick_count * 3);
    if (!result) {
        return nullptr;
    }
    if (brick_count == 0) {
        return result;
    }
    std::vector<int> coords(static_cast<std::size_t>(brick_count) * 3u, 0);
    const int copied = aero_lbm_simulation_copy_brick_world_resident_bricks(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        coords.data(),
        brick_count
    );
    if (copied != brick_count) {
        return nullptr;
    }
    env->SetIntArrayRegion(
        result,
        0,
        brick_count * 3,
        reinterpret_cast<const jint*>(coords.data())
    );
    return result;
}

JNIEXPORT jintArray JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeGetBrickWorldActiveBrickCoords(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key
) {
    const int brick_count = aero_lbm_simulation_get_brick_world_active_brick_count(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key)
    );
    if (brick_count < 0) {
        return nullptr;
    }
    jintArray result = env->NewIntArray(brick_count * 3);
    if (!result) {
        return nullptr;
    }
    if (brick_count == 0) {
        return result;
    }
    std::vector<int> coords(static_cast<std::size_t>(brick_count) * 3u, 0);
    const int copied = aero_lbm_simulation_copy_brick_world_active_bricks(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        coords.data(),
        brick_count
    );
    if (copied != brick_count) {
        return nullptr;
    }
    env->SetIntArrayRegion(
        result,
        0,
        brick_count * 3,
        reinterpret_cast<const jint*>(coords.data())
    );
    return result;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeUploadBrickWorldStaticBrick(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key,
    jint brick_size,
    jint brick_x,
    jint brick_y,
    jint brick_z,
    jbyteArray obstacle,
    jbyteArray surface_kind,
    jshortArray open_face_mask,
    jfloatArray emitter_power_watts,
    jbyteArray face_sky_exposure,
    jbyteArray face_direct_exposure
) {
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        return JNI_FALSE;
    }
    const int cells = brick_size * brick_size * brick_size;
    if (cells <= 0
        || env->GetArrayLength(obstacle) != static_cast<jsize>(cells)
        || env->GetArrayLength(surface_kind) != static_cast<jsize>(cells)
        || env->GetArrayLength(open_face_mask) != static_cast<jsize>(cells)
        || env->GetArrayLength(emitter_power_watts) != static_cast<jsize>(cells)
        || env->GetArrayLength(face_sky_exposure) != static_cast<jsize>(cells * k_face_count)
        || env->GetArrayLength(face_direct_exposure) != static_cast<jsize>(cells * k_face_count)) {
        return JNI_FALSE;
    }
    jboolean obstacle_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jboolean open_copy = JNI_FALSE;
    jboolean emitter_copy = JNI_FALSE;
    jboolean sky_copy = JNI_FALSE;
    jboolean direct_copy = JNI_FALSE;
    jbyte* obstacle_ptr = env->GetByteArrayElements(obstacle, &obstacle_copy);
    jbyte* surface_ptr = env->GetByteArrayElements(surface_kind, &surface_copy);
    jshort* open_ptr = env->GetShortArrayElements(open_face_mask, &open_copy);
    jfloat* emitter_ptr = env->GetFloatArrayElements(emitter_power_watts, &emitter_copy);
    jbyte* sky_ptr = env->GetByteArrayElements(face_sky_exposure, &sky_copy);
    jbyte* direct_ptr = env->GetByteArrayElements(face_direct_exposure, &direct_copy);
    if (!obstacle_ptr || !surface_ptr || !open_ptr || !emitter_ptr || !sky_ptr || !direct_ptr) {
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
        if (sky_ptr) {
            env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
        }
        if (direct_ptr) {
            env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_upload_brick_world_static_brick(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        brick_size,
        brick_x,
        brick_y,
        brick_z,
        reinterpret_cast<const uint8_t*>(obstacle_ptr),
        reinterpret_cast<const uint8_t*>(surface_ptr),
        reinterpret_cast<const uint16_t*>(open_ptr),
        emitter_ptr,
        reinterpret_cast<const uint8_t*>(sky_ptr),
        reinterpret_cast<const uint8_t*>(direct_ptr)
    );
    env->ReleaseByteArrayElements(obstacle, obstacle_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(surface_kind, surface_ptr, JNI_ABORT);
    env->ReleaseShortArrayElements(open_face_mask, open_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(emitter_power_watts, emitter_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeQueueBrickWorldStaticBrickUpload(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key,
    jint brick_size,
    jint brick_x,
    jint brick_y,
    jint brick_z,
    jbyteArray obstacle,
    jbyteArray surface_kind,
    jshortArray open_face_mask,
    jfloatArray emitter_power_watts,
    jbyteArray face_sky_exposure,
    jbyteArray face_direct_exposure
) {
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        return JNI_FALSE;
    }
    const int cells = brick_size * brick_size * brick_size;
    if (cells <= 0
        || env->GetArrayLength(obstacle) != static_cast<jsize>(cells)
        || env->GetArrayLength(surface_kind) != static_cast<jsize>(cells)
        || env->GetArrayLength(open_face_mask) != static_cast<jsize>(cells)
        || env->GetArrayLength(emitter_power_watts) != static_cast<jsize>(cells)
        || env->GetArrayLength(face_sky_exposure) != static_cast<jsize>(cells * k_face_count)
        || env->GetArrayLength(face_direct_exposure) != static_cast<jsize>(cells * k_face_count)) {
        return JNI_FALSE;
    }
    jboolean obstacle_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jboolean open_copy = JNI_FALSE;
    jboolean emitter_copy = JNI_FALSE;
    jboolean sky_copy = JNI_FALSE;
    jboolean direct_copy = JNI_FALSE;
    jbyte* obstacle_ptr = env->GetByteArrayElements(obstacle, &obstacle_copy);
    jbyte* surface_ptr = env->GetByteArrayElements(surface_kind, &surface_copy);
    jshort* open_ptr = env->GetShortArrayElements(open_face_mask, &open_copy);
    jfloat* emitter_ptr = env->GetFloatArrayElements(emitter_power_watts, &emitter_copy);
    jbyte* sky_ptr = env->GetByteArrayElements(face_sky_exposure, &sky_copy);
    jbyte* direct_ptr = env->GetByteArrayElements(face_direct_exposure, &direct_copy);
    if (!obstacle_ptr || !surface_ptr || !open_ptr || !emitter_ptr || !sky_ptr || !direct_ptr) {
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
        if (sky_ptr) {
            env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
        }
        if (direct_ptr) {
            env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_queue_brick_world_static_brick_upload(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        brick_size,
        brick_x,
        brick_y,
        brick_z,
        reinterpret_cast<const uint8_t*>(obstacle_ptr),
        reinterpret_cast<const uint8_t*>(surface_ptr),
        reinterpret_cast<const uint16_t*>(open_ptr),
        emitter_ptr,
        reinterpret_cast<const uint8_t*>(sky_ptr),
        reinterpret_cast<const uint8_t*>(direct_ptr)
    );
    env->ReleaseByteArrayElements(obstacle, obstacle_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(surface_kind, surface_ptr, JNI_ABORT);
    env->ReleaseShortArrayElements(open_face_mask, open_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(emitter_power_watts, emitter_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSyncRegionCoreToBrickWorld(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key,
    jlong world_key,
    jint region_nx,
    jint region_ny,
    jint region_nz,
    jint core_offset_x,
    jint core_offset_y,
    jint core_offset_z,
    jint core_nx,
    jint core_ny,
    jint core_nz,
    jint brick_x,
    jint brick_y,
    jint brick_z
) {
    return aero_lbm_simulation_sync_region_core_to_brick_world(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        static_cast<long long>(world_key),
        region_nx,
        region_ny,
        region_nz,
        core_offset_x,
        core_offset_y,
        core_offset_z,
        core_nx,
        core_ny,
        core_nz,
        brick_x,
        brick_y,
        brick_z
    ) ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeCopyBrickWorldDynamicBrick(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong world_key,
    jint brick_size,
    jint brick_x,
    jint brick_y,
    jint brick_z,
    jfloatArray out_flow_state,
    jfloatArray out_air_temperature,
    jfloatArray out_surface_temperature
) {
    if (!out_flow_state || !out_air_temperature || !out_surface_temperature) {
        return JNI_FALSE;
    }
    const int cells = brick_size * brick_size * brick_size;
    if (cells <= 0
        || env->GetArrayLength(out_flow_state) != static_cast<jsize>(cells * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS)
        || env->GetArrayLength(out_air_temperature) != static_cast<jsize>(cells)
        || env->GetArrayLength(out_surface_temperature) != static_cast<jsize>(cells)) {
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
            env->ReleaseFloatArrayElements(out_flow_state, flow_ptr, JNI_ABORT);
        }
        if (air_ptr) {
            env->ReleaseFloatArrayElements(out_air_temperature, air_ptr, JNI_ABORT);
        }
        if (surface_ptr) {
            env->ReleaseFloatArrayElements(out_surface_temperature, surface_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_copy_brick_world_dynamic_brick(
        static_cast<long long>(service_key),
        static_cast<long long>(world_key),
        brick_size,
        brick_x,
        brick_y,
        brick_z,
        flow_ptr,
        air_ptr,
        surface_ptr
    );
    env->ReleaseFloatArrayElements(out_flow_state, flow_ptr, ok ? 0 : JNI_ABORT);
    env->ReleaseFloatArrayElements(out_air_temperature, air_ptr, ok ? 0 : JNI_ABORT);
    env->ReleaseFloatArrayElements(out_surface_temperature, surface_ptr, ok ? 0 : JNI_ABORT);
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
    jfloatArray emitter_power_watts,
    jbyteArray face_sky_exposure,
    jbyteArray face_direct_exposure
) {
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0
        || env->GetArrayLength(obstacle) != static_cast<jsize>(cells)
        || env->GetArrayLength(surface_kind) != static_cast<jsize>(cells)
        || env->GetArrayLength(open_face_mask) != static_cast<jsize>(cells)
        || env->GetArrayLength(emitter_power_watts) != static_cast<jsize>(cells)
        || env->GetArrayLength(face_sky_exposure) != static_cast<jsize>(cells * k_face_count)
        || env->GetArrayLength(face_direct_exposure) != static_cast<jsize>(cells * k_face_count)) {
        return JNI_FALSE;
    }
    jboolean obstacle_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jboolean open_copy = JNI_FALSE;
    jboolean emitter_copy = JNI_FALSE;
    jboolean sky_copy = JNI_FALSE;
    jboolean direct_copy = JNI_FALSE;
    jbyte* obstacle_ptr = env->GetByteArrayElements(obstacle, &obstacle_copy);
    jbyte* surface_ptr = env->GetByteArrayElements(surface_kind, &surface_copy);
    jshort* open_ptr = env->GetShortArrayElements(open_face_mask, &open_copy);
    jfloat* emitter_ptr = env->GetFloatArrayElements(emitter_power_watts, &emitter_copy);
    jbyte* sky_ptr = env->GetByteArrayElements(face_sky_exposure, &sky_copy);
    jbyte* direct_ptr = env->GetByteArrayElements(face_direct_exposure, &direct_copy);
    if (!obstacle_ptr || !surface_ptr || !open_ptr || !emitter_ptr || !sky_ptr || !direct_ptr) {
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
        if (sky_ptr) {
            env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
        }
        if (direct_ptr) {
            env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
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
        emitter_ptr,
        reinterpret_cast<const uint8_t*>(sky_ptr),
        reinterpret_cast<const uint8_t*>(direct_ptr)
    );
    env->ReleaseByteArrayElements(obstacle, obstacle_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(surface_kind, surface_ptr, JNI_ABORT);
    env->ReleaseShortArrayElements(open_face_mask, open_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(emitter_power_watts, emitter_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeUploadStaticRegionPatch(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jint offset_x,
    jint offset_y,
    jint offset_z,
    jint patch_nx,
    jint patch_ny,
    jint patch_nz,
    jbyteArray obstacle,
    jbyteArray surface_kind,
    jshortArray open_face_mask,
    jfloatArray emitter_power_watts,
    jbyteArray face_sky_exposure,
    jbyteArray face_direct_exposure
) {
    if (!obstacle || !surface_kind || !open_face_mask || !emitter_power_watts
        || !face_sky_exposure || !face_direct_exposure) {
        return JNI_FALSE;
    }
    const int cells = patch_nx * patch_ny * patch_nz;
    if (cells <= 0
        || env->GetArrayLength(obstacle) != static_cast<jsize>(cells)
        || env->GetArrayLength(surface_kind) != static_cast<jsize>(cells)
        || env->GetArrayLength(open_face_mask) != static_cast<jsize>(cells)
        || env->GetArrayLength(emitter_power_watts) != static_cast<jsize>(cells)
        || env->GetArrayLength(face_sky_exposure) != static_cast<jsize>(cells * k_face_count)
        || env->GetArrayLength(face_direct_exposure) != static_cast<jsize>(cells * k_face_count)) {
        return JNI_FALSE;
    }
    jboolean obstacle_copy = JNI_FALSE;
    jboolean surface_copy = JNI_FALSE;
    jboolean open_copy = JNI_FALSE;
    jboolean emitter_copy = JNI_FALSE;
    jboolean sky_copy = JNI_FALSE;
    jboolean direct_copy = JNI_FALSE;
    jbyte* obstacle_ptr = env->GetByteArrayElements(obstacle, &obstacle_copy);
    jbyte* surface_ptr = env->GetByteArrayElements(surface_kind, &surface_copy);
    jshort* open_ptr = env->GetShortArrayElements(open_face_mask, &open_copy);
    jfloat* emitter_ptr = env->GetFloatArrayElements(emitter_power_watts, &emitter_copy);
    jbyte* sky_ptr = env->GetByteArrayElements(face_sky_exposure, &sky_copy);
    jbyte* direct_ptr = env->GetByteArrayElements(face_direct_exposure, &direct_copy);
    if (!obstacle_ptr || !surface_ptr || !open_ptr || !emitter_ptr || !sky_ptr || !direct_ptr) {
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
        if (sky_ptr) {
            env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
        }
        if (direct_ptr) {
            env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
        }
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_upload_static_region_patch(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        offset_x,
        offset_y,
        offset_z,
        patch_nx,
        patch_ny,
        patch_nz,
        reinterpret_cast<const uint8_t*>(obstacle_ptr),
        reinterpret_cast<const uint8_t*>(surface_ptr),
        reinterpret_cast<const uint16_t*>(open_ptr),
        emitter_ptr,
        reinterpret_cast<const uint8_t*>(sky_ptr),
        reinterpret_cast<const uint8_t*>(direct_ptr)
    );
    env->ReleaseByteArrayElements(obstacle, obstacle_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(surface_kind, surface_ptr, JNI_ABORT);
    env->ReleaseShortArrayElements(open_face_mask, open_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(emitter_power_watts, emitter_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_sky_exposure, sky_ptr, JNI_ABORT);
    env->ReleaseByteArrayElements(face_direct_exposure, direct_ptr, JNI_ABORT);
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSetRegionNestedFeedbackLayout(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint steps_per_feedback,
    jintArray layout
) {
    if (!layout) {
        return JNI_FALSE;
    }
    const jsize value_count = env->GetArrayLength(layout);
    jboolean copy = JNI_FALSE;
    jint* layout_ptr = env->GetIntArrayElements(layout, &copy);
    if (!layout_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_set_region_nested_feedback_layout(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        steps_per_feedback,
        reinterpret_cast<const int*>(layout_ptr),
        static_cast<int>(value_count)
    );
    env->ReleaseIntArrayElements(layout, layout_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeUploadRegionForcing(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray thermal_source,
    jbyteArray fan_mask,
    jfloatArray fan_vx,
    jfloatArray fan_vy,
    jfloatArray fan_vz
) {
    if (!fan_mask || !fan_vx || !fan_vy || !fan_vz) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0
        || (thermal_source && env->GetArrayLength(thermal_source) != static_cast<jsize>(cells))
        || env->GetArrayLength(fan_mask) != static_cast<jsize>(cells)
        || env->GetArrayLength(fan_vx) != static_cast<jsize>(cells)
        || env->GetArrayLength(fan_vy) != static_cast<jsize>(cells)
        || env->GetArrayLength(fan_vz) != static_cast<jsize>(cells)) {
        return JNI_FALSE;
    }
    jboolean thermal_copy = JNI_FALSE;
    jboolean mask_copy = JNI_FALSE;
    jboolean vx_copy = JNI_FALSE;
    jboolean vy_copy = JNI_FALSE;
    jboolean vz_copy = JNI_FALSE;
    jfloat* thermal_ptr = thermal_source ? env->GetFloatArrayElements(thermal_source, &thermal_copy) : nullptr;
    jbyte* mask_ptr = env->GetByteArrayElements(fan_mask, &mask_copy);
    jfloat* vx_ptr = env->GetFloatArrayElements(fan_vx, &vx_copy);
    jfloat* vy_ptr = env->GetFloatArrayElements(fan_vy, &vy_copy);
    jfloat* vz_ptr = env->GetFloatArrayElements(fan_vz, &vz_copy);
    if ((thermal_source && !thermal_ptr) || !mask_ptr || !vx_ptr || !vy_ptr || !vz_ptr) {
        if (thermal_ptr) env->ReleaseFloatArrayElements(thermal_source, thermal_ptr, JNI_ABORT);
        if (mask_ptr) env->ReleaseByteArrayElements(fan_mask, mask_ptr, JNI_ABORT);
        if (vx_ptr) env->ReleaseFloatArrayElements(fan_vx, vx_ptr, JNI_ABORT);
        if (vy_ptr) env->ReleaseFloatArrayElements(fan_vy, vy_ptr, JNI_ABORT);
        if (vz_ptr) env->ReleaseFloatArrayElements(fan_vz, vz_ptr, JNI_ABORT);
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_upload_region_forcing(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        thermal_ptr,
        reinterpret_cast<const uint8_t*>(mask_ptr),
        vx_ptr,
        vy_ptr,
        vz_ptr
    );
    if (thermal_ptr) {
        env->ReleaseFloatArrayElements(thermal_source, thermal_ptr, JNI_ABORT);
    }
    env->ReleaseByteArrayElements(fan_mask, mask_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(fan_vx, vx_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(fan_vy, vy_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(fan_vz, vz_ptr, JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeRefreshRegionThermal(
    JNIEnv*,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloat direct_solar_flux_w_m2,
    jfloat diffuse_solar_flux_w_m2,
    jfloat ambient_air_temperature_k,
    jfloat deep_ground_temperature_k,
    jfloat sky_temperature_k,
    jfloat precipitation_temperature_k,
    jfloat precipitation_strength,
    jfloat sun_x,
    jfloat sun_y,
    jfloat sun_z,
    jfloat surface_delta_seconds
) {
    return aero_lbm_simulation_refresh_region_thermal(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        direct_solar_flux_w_m2,
        diffuse_solar_flux_w_m2,
        ambient_air_temperature_k,
        deep_ground_temperature_k,
        sky_temperature_k,
        precipitation_temperature_k,
        precipitation_strength,
        sun_x,
        sun_y,
        sun_z,
        surface_delta_seconds
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeStepRegionStored(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloat boundary_wind_x,
    jfloat boundary_wind_y,
    jfloat boundary_wind_z,
    jfloat fallback_boundary_air_temperature_k,
    jint external_face_mask,
    jint boundary_face_resolution,
    jfloatArray boundary_wind_face_x,
    jfloatArray boundary_wind_face_y,
    jfloatArray boundary_wind_face_z,
    jfloatArray boundary_air_temperature_k,
    jint sponge_thickness_cells,
    jfloat sponge_velocity_relaxation,
    jfloat sponge_temperature_relaxation,
    jint tornado_descriptor_count,
    jfloatArray tornado_descriptors,
    jfloatArray out_max_speed
) {
    if (!out_max_speed || env->GetArrayLength(out_max_speed) != 1) {
        return JNI_FALSE;
    }
    const int face_cells = boundary_face_resolution <= 0
        ? 0
        : k_face_count * boundary_face_resolution * boundary_face_resolution;
    if (face_cells > 0
        && (!boundary_wind_face_x
            || !boundary_wind_face_y
            || !boundary_wind_face_z
            || !boundary_air_temperature_k
            || env->GetArrayLength(boundary_wind_face_x) != static_cast<jsize>(face_cells)
            || env->GetArrayLength(boundary_wind_face_y) != static_cast<jsize>(face_cells)
            || env->GetArrayLength(boundary_wind_face_z) != static_cast<jsize>(face_cells)
            || env->GetArrayLength(boundary_air_temperature_k) != static_cast<jsize>(face_cells))) {
        return JNI_FALSE;
    }
    if (tornado_descriptor_count < 0
        || (tornado_descriptor_count > 0
            && (!tornado_descriptors
                || env->GetArrayLength(tornado_descriptors) != static_cast<jsize>(tornado_descriptor_count * k_tornado_descriptor_floats)))) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jfloat* max_speed_ptr = env->GetFloatArrayElements(out_max_speed, &copy);
    if (!max_speed_ptr) {
        return JNI_FALSE;
    }
    jboolean face_x_copy = JNI_FALSE;
    jboolean face_y_copy = JNI_FALSE;
    jboolean face_z_copy = JNI_FALSE;
    jboolean face_temp_copy = JNI_FALSE;
    jboolean tornado_copy = JNI_FALSE;
    jfloat* face_x_ptr = face_cells > 0 ? env->GetFloatArrayElements(boundary_wind_face_x, &face_x_copy) : nullptr;
    jfloat* face_y_ptr = face_cells > 0 ? env->GetFloatArrayElements(boundary_wind_face_y, &face_y_copy) : nullptr;
    jfloat* face_z_ptr = face_cells > 0 ? env->GetFloatArrayElements(boundary_wind_face_z, &face_z_copy) : nullptr;
    jfloat* face_temp_ptr = face_cells > 0 ? env->GetFloatArrayElements(boundary_air_temperature_k, &face_temp_copy) : nullptr;
    jfloat* tornado_ptr = tornado_descriptor_count > 0 ? env->GetFloatArrayElements(tornado_descriptors, &tornado_copy) : nullptr;
    if ((face_cells > 0 && (!face_x_ptr || !face_y_ptr || !face_z_ptr || !face_temp_ptr))
        || (tornado_descriptor_count > 0 && !tornado_ptr)) {
        if (face_x_ptr) {
            env->ReleaseFloatArrayElements(boundary_wind_face_x, face_x_ptr, JNI_ABORT);
        }
        if (face_y_ptr) {
            env->ReleaseFloatArrayElements(boundary_wind_face_y, face_y_ptr, JNI_ABORT);
        }
        if (face_z_ptr) {
            env->ReleaseFloatArrayElements(boundary_wind_face_z, face_z_ptr, JNI_ABORT);
        }
        if (face_temp_ptr) {
            env->ReleaseFloatArrayElements(boundary_air_temperature_k, face_temp_ptr, JNI_ABORT);
        }
        if (tornado_ptr) {
            env->ReleaseFloatArrayElements(tornado_descriptors, tornado_ptr, JNI_ABORT);
        }
        env->ReleaseFloatArrayElements(out_max_speed, max_speed_ptr, JNI_ABORT);
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_step_region_stored(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        boundary_wind_x,
        boundary_wind_y,
        boundary_wind_z,
        fallback_boundary_air_temperature_k,
        external_face_mask,
        boundary_face_resolution,
        face_x_ptr,
        face_y_ptr,
        face_z_ptr,
        face_temp_ptr,
        sponge_thickness_cells,
        sponge_velocity_relaxation,
        sponge_temperature_relaxation,
        tornado_descriptor_count,
        tornado_ptr,
        max_speed_ptr
    );
    if (face_x_ptr) {
        env->ReleaseFloatArrayElements(boundary_wind_face_x, face_x_ptr, JNI_ABORT);
    }
    if (face_y_ptr) {
        env->ReleaseFloatArrayElements(boundary_wind_face_y, face_y_ptr, JNI_ABORT);
    }
    if (face_z_ptr) {
        env->ReleaseFloatArrayElements(boundary_wind_face_z, face_z_ptr, JNI_ABORT);
    }
    if (face_temp_ptr) {
        env->ReleaseFloatArrayElements(boundary_air_temperature_k, face_temp_ptr, JNI_ABORT);
    }
    if (tornado_ptr) {
        env->ReleaseFloatArrayElements(tornado_descriptors, tornado_ptr, JNI_ABORT);
    }
    env->ReleaseFloatArrayElements(out_max_speed, max_speed_ptr, ok ? 0 : JNI_ABORT);
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

JNIEXPORT jint JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeExchangeRegionHaloBatch(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlongArray region_pairs,
    jintArray offsets,
    jint exchange_count
) {
    if (!region_pairs || !offsets || exchange_count < 0) {
        return -1;
    }
    if (exchange_count == 0) {
        return 0;
    }
    if (env->GetArrayLength(region_pairs) < static_cast<jsize>(exchange_count * 2)
        || env->GetArrayLength(offsets) < static_cast<jsize>(exchange_count * 3)) {
        return -1;
    }
    jboolean region_copy = JNI_FALSE;
    jboolean offset_copy = JNI_FALSE;
    jlong* region_pairs_ptr = env->GetLongArrayElements(region_pairs, &region_copy);
    jint* offsets_ptr = env->GetIntArrayElements(offsets, &offset_copy);
    if (!region_pairs_ptr || !offsets_ptr) {
        if (region_pairs_ptr) {
            env->ReleaseLongArrayElements(region_pairs, region_pairs_ptr, JNI_ABORT);
        }
        if (offsets_ptr) {
            env->ReleaseIntArrayElements(offsets, offsets_ptr, JNI_ABORT);
        }
        return -1;
    }
    int applied_count = 0;
    const int ok = aero_lbm_simulation_exchange_region_halo_batch(
        static_cast<long long>(service_key),
        reinterpret_cast<const long long*>(region_pairs_ptr),
        reinterpret_cast<const int*>(offsets_ptr),
        exchange_count,
        &applied_count
    );
    env->ReleaseLongArrayElements(region_pairs, region_pairs_ptr, JNI_ABORT);
    env->ReleaseIntArrayElements(offsets, offsets_ptr, JNI_ABORT);
    return ok ? applied_count : -1;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSyncRegionState(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jfloatArray out_max_speed
) {
    if (!out_max_speed || env->GetArrayLength(out_max_speed) != 1) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jfloat* max_speed_ptr = env->GetFloatArrayElements(out_max_speed, &copy);
    if (!max_speed_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_sync_region_state(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        max_speed_ptr
    );
    env->ReleaseFloatArrayElements(out_max_speed, max_speed_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSyncRegionFlowState(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jfloatArray out_max_speed
) {
    if (!out_max_speed || env->GetArrayLength(out_max_speed) != 1) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jfloat* max_speed_ptr = env->GetFloatArrayElements(out_max_speed, &copy);
    if (!max_speed_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_sync_region_flow_state(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        max_speed_ptr
    );
    env->ReleaseFloatArrayElements(out_max_speed, max_speed_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeGetRegionFlowState(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray out_flow
) {
    if (!out_flow) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0 || env->GetArrayLength(out_flow) != static_cast<jsize>(cells * AERO_LBM_SIMULATION_FLOW_STATE_CHANNELS)) {
        return JNI_FALSE;
    }
    jboolean copy = JNI_FALSE;
    jfloat* flow_ptr = env->GetFloatArrayElements(out_flow, &copy);
    if (!flow_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_get_region_flow_state(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        flow_ptr
    );
    env->ReleaseFloatArrayElements(out_flow, flow_ptr, ok ? 0 : JNI_ABORT);
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativePollRegionNestedFeedback(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jfloatArray out_values
) {
    if (!out_values) {
        return JNI_FALSE;
    }
    const jsize length = env->GetArrayLength(out_values);
    jboolean copy = JNI_FALSE;
    jfloat* values_ptr = env->GetFloatArrayElements(out_values, &copy);
    if (!values_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_poll_region_nested_feedback(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        values_ptr,
        static_cast<int>(length)
    );
    env->ReleaseFloatArrayElements(out_values, values_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeGetRegionNestedFeedbackStatus(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jintArray out_status
) {
    if (!out_status) {
        return JNI_FALSE;
    }
    const jsize length = env->GetArrayLength(out_status);
    jboolean copy = JNI_FALSE;
    jint* status_ptr = env->GetIntArrayElements(out_status, &copy);
    if (!status_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_get_region_nested_feedback_status(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        reinterpret_cast<int*>(status_ptr),
        static_cast<int>(length)
    );
    env->ReleaseIntArrayElements(out_status, status_ptr, ok ? 0 : JNI_ABORT);
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeSampleRegionPoint(
    JNIEnv* env,
    jclass,
    jlong service_key,
    jlong region_key,
    jint nx,
    jint ny,
    jint nz,
    jint sample_x,
    jint sample_y,
    jint sample_z,
    jfloatArray out_probe_values
) {
    if (!out_probe_values) {
        return JNI_FALSE;
    }
    const jsize length = env->GetArrayLength(out_probe_values);
    jboolean copy = JNI_FALSE;
    jfloat* values_ptr = env->GetFloatArrayElements(out_probe_values, &copy);
    if (!values_ptr) {
        return JNI_FALSE;
    }
    const int ok = aero_lbm_simulation_sample_region_point(
        static_cast<long long>(service_key),
        static_cast<long long>(region_key),
        nx,
        ny,
        nz,
        sample_x,
        sample_y,
        sample_z,
        values_ptr,
        static_cast<int>(length)
    );
    env->ReleaseFloatArrayElements(out_probe_values, values_ptr, 0);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jbyteArray JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeCompressFloatGrid3d(
    JNIEnv* env,
    jclass,
    jfloatArray values,
    jint nx,
    jint ny,
    jint nz,
    jdouble tolerance
) {
    if (!values) {
        return nullptr;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0 || env->GetArrayLength(values) != static_cast<jsize>(cells)) {
        return nullptr;
    }
    jboolean copy = JNI_FALSE;
    jfloat* values_ptr = env->GetFloatArrayElements(values, &copy);
    if (!values_ptr) {
        return nullptr;
    }
    std::vector<std::uint8_t> compressed;
    std::string error;
    const bool ok = aero_lbm_analysis_codec::compress_float_grid_3d(
        values_ptr,
        nx,
        ny,
        nz,
        static_cast<double>(tolerance),
        compressed,
        error
    );
    env->ReleaseFloatArrayElements(values, values_ptr, JNI_ABORT);
    if (!ok || compressed.size() > static_cast<std::size_t>(std::numeric_limits<jsize>::max())) {
        return nullptr;
    }
    jbyteArray out = env->NewByteArray(static_cast<jsize>(compressed.size()));
    if (!out) {
        return nullptr;
    }
    env->SetByteArrayRegion(
        out,
        0,
        static_cast<jsize>(compressed.size()),
        reinterpret_cast<const jbyte*>(compressed.data())
    );
    return out;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeDecompressFloatGrid3d(
    JNIEnv* env,
    jclass,
    jbyteArray compressed,
    jint nx,
    jint ny,
    jint nz,
    jfloatArray out_values
) {
    if (!compressed || !out_values) {
        return JNI_FALSE;
    }
    const int cells = nx * ny * nz;
    if (cells <= 0 || env->GetArrayLength(out_values) != static_cast<jsize>(cells)) {
        return JNI_FALSE;
    }
    const jsize compressed_size = env->GetArrayLength(compressed);
    std::vector<std::uint8_t> compressed_bytes(static_cast<std::size_t>(compressed_size));
    if (compressed_size > 0) {
        env->GetByteArrayRegion(
            compressed,
            0,
            compressed_size,
            reinterpret_cast<jbyte*>(compressed_bytes.data())
        );
        if (env->ExceptionCheck()) {
            return JNI_FALSE;
        }
    }
    jboolean copy = JNI_FALSE;
    jfloat* out_ptr = env->GetFloatArrayElements(out_values, &copy);
    if (!out_ptr) {
        return JNI_FALSE;
    }
    std::string error;
    const bool ok = aero_lbm_analysis_codec::decompress_float_grid_3d(
        compressed_bytes.data(),
        compressed_bytes.size(),
        nx,
        ny,
        nz,
        out_ptr,
        error
    );
    env->ReleaseFloatArrayElements(out_values, out_ptr, ok ? 0 : JNI_ABORT);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeRuntimeInfo(JNIEnv* env, jclass) {
    return new_java_string(env, aero_lbm_simulation_runtime_info());
}

JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_runtime_NativeSimulationBridge_nativeLastError(JNIEnv* env, jclass) {
    return new_java_string(env, aero_lbm_simulation_last_error());
}

}  // extern "C"
