#include "aero_solver_capi.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr int kInputChannels = 11;
constexpr int kOutputChannels = AERO_SOLVER_FLOW_CHANNELS;
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
constexpr float kDefaultDxMeters = 1.0f;
constexpr float kDefaultDtSeconds = 0.05f;
constexpr float kDefaultDensityKgM3 = 1.225f;
constexpr float kDefaultViscosityM2S = 1.5e-5f;
constexpr float kMinLatticeNu = 1.0e-7f;
constexpr float kMaxLatticeNu = 0.12f;

struct SolverContext {
    long long handle = 0;
    long long context_key = 0;
    AeroGridDesc grid{};
    int cells = 0;
    bool custom_flow_state = false;
    bool packet_dirty = true;
    AeroBoundaryDesc packet_boundary{};
    std::vector<uint8_t> solid_mask;
    std::vector<float> packet;
    std::vector<float> scratch_flow;
};

std::mutex g_solver_mutex;
std::unordered_map<long long, std::unique_ptr<SolverContext>> g_contexts;
long long g_next_handle = 1;
std::string g_last_error;

void set_error(const char* message) {
    g_last_error = message ? message : "unknown solver error";
}

void set_error(const std::string& message) {
    g_last_error = message;
}

void clear_error() {
    g_last_error.clear();
}

bool valid_grid(const AeroGridDesc& grid, int* out_cells) {
    if (grid.nx <= 0 || grid.ny <= 0 || grid.nz <= 0) {
        set_error("invalid grid dimensions");
        return false;
    }
    if (!std::isfinite(grid.dx) || grid.dx <= 0.0f || !std::isfinite(grid.dt) || grid.dt <= 0.0f) {
        set_error("invalid grid dx/dt");
        return false;
    }
    const int64_t cells = static_cast<int64_t>(grid.nx) * grid.ny * grid.nz;
    if (cells <= 0 || cells > static_cast<int64_t>(std::numeric_limits<int>::max())) {
        set_error("grid cell count overflows int");
        return false;
    }
    if (out_cells) {
        *out_cells = static_cast<int>(cells);
    }
    return true;
}

SolverContext* lookup_context(long long handle) {
    auto it = g_contexts.find(handle);
    if (it == g_contexts.end() || !it->second) {
        set_error("invalid solver handle");
        return nullptr;
    }
    return it->second.get();
}

float finite_or(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

float lattice_velocity_scale(const AeroGridDesc& grid) {
    return grid.dx / grid.dt;
}

float velocity_mps_to_lattice(const AeroGridDesc& grid, float velocity_mps) {
    return finite_or(velocity_mps, 0.0f) / lattice_velocity_scale(grid);
}

float velocity_lattice_to_mps(const AeroGridDesc& grid, float velocity_lattice) {
    return finite_or(velocity_lattice, 0.0f) * lattice_velocity_scale(grid);
}

float pressure_proxy(float pressure) {
    return finite_or(pressure, 0.0f);
}

bool same_boundary_for_packet(const AeroBoundaryDesc& a, const AeroBoundaryDesc& b) {
    return a.mode == b.mode
        && a.inlet_vx == b.inlet_vx
        && a.inlet_vy == b.inlet_vy
        && a.inlet_vz == b.inlet_vz
        && a.outlet_pressure == b.outlet_pressure;
}

std::size_t packet_base(std::size_t cell) {
    return cell * static_cast<std::size_t>(kInputChannels);
}

void write_packet_flow_cell(
    SolverContext& ctx,
    std::size_t cell,
    float vx_mps,
    float vy_mps,
    float vz_mps,
    float pressure
) {
    const std::size_t base = packet_base(cell);
    ctx.packet[base + kChannelStateVx] = velocity_mps_to_lattice(ctx.grid, vx_mps);
    ctx.packet[base + kChannelStateVy] = velocity_mps_to_lattice(ctx.grid, vy_mps);
    ctx.packet[base + kChannelStateVz] = velocity_mps_to_lattice(ctx.grid, vz_mps);
    ctx.packet[base + kChannelStateP] = pressure_proxy(pressure);
}

void initialize_packet(SolverContext& ctx, const AeroBoundaryDesc& boundary, bool overwrite_flow_state) {
    const float vx = finite_or(boundary.inlet_vx, 0.0f);
    const float vy = finite_or(boundary.inlet_vy, 0.0f);
    const float vz = finite_or(boundary.inlet_vz, 0.0f);
    const float pressure = pressure_proxy(boundary.outlet_pressure);
    for (int cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t index = static_cast<std::size_t>(cell);
        const std::size_t base = packet_base(index);
        ctx.packet[base + kChannelObstacle] = ctx.solid_mask[index] != 0 ? 1.0f : 0.0f;
        ctx.packet[base + kChannelFanMask] = 0.0f;
        ctx.packet[base + kChannelFanVx] = 0.0f;
        ctx.packet[base + kChannelFanVy] = 0.0f;
        ctx.packet[base + kChannelFanVz] = 0.0f;
        ctx.packet[base + kChannelThermalSource] = 0.0f;
        ctx.packet[base + kChannelStateTemp] = 0.0f;
        if (ctx.solid_mask[index] != 0) {
            write_packet_flow_cell(ctx, index, 0.0f, 0.0f, 0.0f, 0.0f);
        } else if (overwrite_flow_state) {
            write_packet_flow_cell(ctx, index, vx, vy, vz, pressure);
        }
    }
}

float max_velocity_from_output(int cells, const float* flow, int value_count) {
    if (!flow || value_count < cells * kOutputChannels) {
        return 0.0f;
    }
    float max_velocity = 0.0f;
    for (int cell = 0; cell < cells; ++cell) {
        const std::size_t base = static_cast<std::size_t>(cell) * kOutputChannels;
        const float vx = flow[base + 0];
        const float vy = flow[base + 1];
        const float vz = flow[base + 2];
        const float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (std::isfinite(speed)) {
            max_velocity = std::max(max_velocity, speed);
        }
    }
    return max_velocity;
}

AeroLbmBoundaryFaceConfig make_face(int hydro_kind, int thermal_kind) {
    AeroLbmBoundaryFaceConfig face{};
    face.hydrodynamic_kind = hydro_kind;
    face.thermal_kind = thermal_kind;
    return face;
}

void set_all_faces(AeroLbmBenchmarkConfig& cfg, int hydro_kind, int thermal_kind) {
    cfg.x_min = make_face(hydro_kind, thermal_kind);
    cfg.x_max = make_face(hydro_kind, thermal_kind);
    cfg.y_min = make_face(hydro_kind, thermal_kind);
    cfg.y_max = make_face(hydro_kind, thermal_kind);
    cfg.z_min = make_face(hydro_kind, thermal_kind);
    cfg.z_max = make_face(hydro_kind, thermal_kind);
}

float boundary_speed_lattice(const SolverContext& ctx, const AeroBoundaryDesc& boundary) {
    const float vx = velocity_mps_to_lattice(ctx.grid, boundary.inlet_vx);
    const float vy = velocity_mps_to_lattice(ctx.grid, boundary.inlet_vy);
    const float vz = velocity_mps_to_lattice(ctx.grid, boundary.inlet_vz);
    return std::sqrt(vx * vx + vy * vy + vz * vz);
}

float lattice_viscosity(const SolverContext& ctx, const AeroBoundaryDesc& boundary) {
    const float viscosity = finite_or(boundary.viscosity, kDefaultViscosityM2S);
    const float nu = viscosity > 0.0f ? viscosity * ctx.grid.dt / (ctx.grid.dx * ctx.grid.dx) : kDefaultViscosityM2S;
    return std::clamp(nu, kMinLatticeNu, kMaxLatticeNu);
}

bool configure_benchmark_boundary(const SolverContext& ctx, const AeroBoundaryDesc& boundary) {
    AeroLbmBenchmarkConfig cfg{};
    aero_lbm_benchmark_default_config(&cfg);
    cfg.enabled = 1;
    cfg.preset = AERO_LBM_BENCHMARK_PRESET_NONE;
    cfg.flags |=
        AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_FORCING
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_FAN_NOISE
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_SPONGE
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_OBSTACLE_BOUNCE_BLEND
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_SGS
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_INTERNAL_THERMAL_SOURCE
        | AERO_LBM_BENCHMARK_FLAG_DISABLE_BUOYANCY;
    cfg.flags &= ~static_cast<uint32_t>(AERO_LBM_BENCHMARK_FLAG_DISABLE_CONVECTIVE_OUTFLOW);
    cfg.reference_density = std::max(1.0e-6f, finite_or(boundary.density, kDefaultDensityKgM3));
    cfg.reference_temperature = 0.0f;
    cfg.reference_length = 1.0f;
    cfg.initial_velocity[0] = velocity_mps_to_lattice(ctx.grid, boundary.inlet_vx);
    cfg.initial_velocity[1] = velocity_mps_to_lattice(ctx.grid, boundary.inlet_vy);
    cfg.initial_velocity[2] = velocity_mps_to_lattice(ctx.grid, boundary.inlet_vz);
    cfg.initial_pressure = pressure_proxy(boundary.outlet_pressure);
    const float speed = std::max(boundary_speed_lattice(ctx, boundary), 1.0e-6f);
    cfg.reynolds_number = std::max(1.0e-6f, speed / lattice_viscosity(ctx, boundary));
    cfg.mach_number = std::clamp(speed / std::sqrt(1.0f / 3.0f), 1.0e-4f, 0.30f);

    const int mode = boundary.mode == 0 ? AERO_SOLVER_BOUNDARY_WIND_TUNNEL : boundary.mode;
    switch (mode) {
        case AERO_SOLVER_BOUNDARY_CLOSED:
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_BOUNCE_BACK, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            break;
        case AERO_SOLVER_BOUNDARY_PERIODIC:
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_PERIODIC, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            break;
        case AERO_SOLVER_BOUNDARY_WIND_TUNNEL_PRESSURE_OUTLET:
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_SYMMETRY, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.x_min = make_face(AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.x_max = make_face(AERO_LBM_HYDRO_BOUNDARY_PRESSURE_DIRICHLET, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.x_max.pressure = pressure_proxy(boundary.outlet_pressure);
            break;
        case AERO_SOLVER_BOUNDARY_WIND_TUNNEL:
            set_all_faces(cfg, AERO_LBM_HYDRO_BOUNDARY_SYMMETRY, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.x_min = make_face(AERO_LBM_HYDRO_BOUNDARY_VELOCITY_DIRICHLET, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            cfg.x_max = make_face(AERO_LBM_HYDRO_BOUNDARY_CONVECTIVE_OUTFLOW, AERO_LBM_THERMAL_BOUNDARY_DISABLED);
            break;
        default:
            set_error("invalid wind-tunnel boundary mode");
            return false;
    }
    cfg.x_min.velocity[0] = cfg.initial_velocity[0];
    cfg.x_min.velocity[1] = cfg.initial_velocity[1];
    cfg.x_min.velocity[2] = cfg.initial_velocity[2];
    return aero_lbm_benchmark_set_config(&cfg) != 0;
}

bool convert_output_to_public_units(SolverContext& ctx, float* out_flow, int out_value_count) {
    if (!out_flow || out_value_count != ctx.cells * kOutputChannels) {
        set_error("invalid output flow buffer");
        return false;
    }
    for (int cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t base = static_cast<std::size_t>(cell) * kOutputChannels;
        out_flow[base + 0] = velocity_lattice_to_mps(ctx.grid, out_flow[base + 0]);
        out_flow[base + 1] = velocity_lattice_to_mps(ctx.grid, out_flow[base + 1]);
        out_flow[base + 2] = velocity_lattice_to_mps(ctx.grid, out_flow[base + 2]);
        out_flow[base + 3] = pressure_proxy(out_flow[base + 3]);
    }
    return true;
}

bool step_solver_locked(
    SolverContext& ctx,
    const AeroBoundaryDesc& boundary,
    int steps,
    float* out_flow,
    int out_value_count
) {
    if (steps <= 0) {
        set_error("steps must be positive");
        return false;
    }
    if (!configure_benchmark_boundary(ctx, boundary)) {
        if (g_last_error.empty()) {
            set_error(std::string("failed to configure boundary: ") + aero_lbm_last_error());
        }
        return false;
    }
    const bool overwrite_flow_state = !ctx.custom_flow_state;
    const bool boundary_changes_packet = overwrite_flow_state && !same_boundary_for_packet(ctx.packet_boundary, boundary);
    if (ctx.packet_dirty || boundary_changes_packet) {
        initialize_packet(ctx, boundary, overwrite_flow_state);
        ctx.packet_boundary = boundary;
        ctx.packet_dirty = false;
    }
    if (ctx.scratch_flow.size() != static_cast<std::size_t>(ctx.cells) * kOutputChannels) {
        ctx.scratch_flow.assign(static_cast<std::size_t>(ctx.cells) * kOutputChannels, 0.0f);
    }
    for (int step = 0; step < steps; ++step) {
        float* step_output = step + 1 == steps ? ctx.scratch_flow.data() : nullptr;
        if (!aero_lbm_step_rect(ctx.packet.data(), ctx.grid.nx, ctx.grid.ny, ctx.grid.nz, ctx.context_key, step_output)) {
            set_error(std::string("aero_lbm_step_rect failed: ") + aero_lbm_last_error());
            return false;
        }
    }
    std::copy(ctx.scratch_flow.begin(), ctx.scratch_flow.end(), out_flow);
    return convert_output_to_public_units(ctx, out_flow, out_value_count);
}

}  // namespace

extern "C" {

AERO_LBM_CAPI_EXPORT void aero_solver_default_grid(AeroGridDesc* out_grid) {
    if (!out_grid) return;
    out_grid->nx = 64;
    out_grid->ny = 64;
    out_grid->nz = 64;
    out_grid->dx = kDefaultDxMeters;
    out_grid->dt = kDefaultDtSeconds;
}

AERO_LBM_CAPI_EXPORT void aero_solver_default_boundary(AeroBoundaryDesc* out_boundary) {
    if (!out_boundary) return;
    out_boundary->mode = AERO_SOLVER_BOUNDARY_WIND_TUNNEL;
    out_boundary->inlet_vx = 5.0f;
    out_boundary->inlet_vy = 0.0f;
    out_boundary->inlet_vz = 0.0f;
    out_boundary->outlet_pressure = 0.0f;
    out_boundary->density = kDefaultDensityKgM3;
    out_boundary->viscosity = kDefaultViscosityM2S;
}

AERO_LBM_CAPI_EXPORT int aero_solver_create(
    int nx,
    int ny,
    int nz,
    float dx,
    float dt,
    long long* out_handle
) {
    AeroGridDesc grid{};
    grid.nx = nx;
    grid.ny = ny;
    grid.nz = nz;
    grid.dx = dx;
    grid.dt = dt;
    return aero_solver_create_with_grid(&grid, out_handle);
}

AERO_LBM_CAPI_EXPORT int aero_solver_create_with_grid(
    const AeroGridDesc* grid,
    long long* out_handle
) {
    std::lock_guard<std::mutex> lock(g_solver_mutex);
    clear_error();
    if (!grid || !out_handle) {
        set_error("missing grid or output handle");
        return AERO_SOLVER_STATUS_ERROR;
    }
    int cells = 0;
    if (!valid_grid(*grid, &cells)) {
        return AERO_SOLVER_STATUS_ERROR;
    }
    if (!aero_lbm_init_rect(grid->nx, grid->ny, grid->nz, kInputChannels, kOutputChannels)) {
        set_error(std::string("aero_lbm_init_rect failed: ") + aero_lbm_last_error());
        return AERO_SOLVER_STATUS_ERROR;
    }

    g_contexts.clear();
    auto ctx = std::make_unique<SolverContext>();
    ctx->handle = g_next_handle++;
    if (g_next_handle <= 0) {
        g_next_handle = 1;
    }
    ctx->context_key = ctx->handle;
    ctx->grid = *grid;
    ctx->cells = cells;
    ctx->solid_mask.assign(static_cast<std::size_t>(cells), 0u);
    ctx->packet.assign(static_cast<std::size_t>(cells) * kInputChannels, 0.0f);
    ctx->scratch_flow.assign(static_cast<std::size_t>(cells) * kOutputChannels, 0.0f);
    AeroBoundaryDesc boundary{};
    aero_solver_default_boundary(&boundary);
    initialize_packet(*ctx, boundary, true);
    ctx->packet_boundary = boundary;
    ctx->packet_dirty = false;

    const long long handle = ctx->handle;
    g_contexts.emplace(handle, std::move(ctx));
    *out_handle = handle;
    return AERO_SOLVER_STATUS_OK;
}

AERO_LBM_CAPI_EXPORT int aero_solver_set_solid_mask(
    long long handle,
    const uint8_t* solid_mask,
    int cell_count
) {
    std::lock_guard<std::mutex> lock(g_solver_mutex);
    clear_error();
    SolverContext* ctx = lookup_context(handle);
    if (!ctx) {
        return AERO_SOLVER_STATUS_ERROR;
    }
    if (!solid_mask || cell_count != ctx->cells) {
        set_error("invalid solid mask buffer");
        return AERO_SOLVER_STATUS_ERROR;
    }
    ctx->solid_mask.assign(solid_mask, solid_mask + cell_count);
    ctx->custom_flow_state = false;
    ctx->packet_dirty = true;
    aero_lbm_release_context(ctx->context_key);
    return AERO_SOLVER_STATUS_OK;
}

AERO_LBM_CAPI_EXPORT int aero_solver_set_flow_state(
    long long handle,
    const float* flow,
    int value_count
) {
    std::lock_guard<std::mutex> lock(g_solver_mutex);
    clear_error();
    SolverContext* ctx = lookup_context(handle);
    if (!ctx) {
        return AERO_SOLVER_STATUS_ERROR;
    }
    if (!flow || value_count != ctx->cells * kOutputChannels) {
        set_error("invalid flow state buffer");
        return AERO_SOLVER_STATUS_ERROR;
    }
    for (int cell = 0; cell < ctx->cells; ++cell) {
        const std::size_t index = static_cast<std::size_t>(cell);
        const std::size_t flow_base = index * kOutputChannels;
        if (ctx->solid_mask[index] != 0) {
            write_packet_flow_cell(*ctx, index, 0.0f, 0.0f, 0.0f, 0.0f);
            continue;
        }
        write_packet_flow_cell(
            *ctx,
            index,
            flow[flow_base + 0],
            flow[flow_base + 1],
            flow[flow_base + 2],
            flow[flow_base + 3]
        );
    }
    ctx->custom_flow_state = true;
    ctx->packet_dirty = false;
    aero_lbm_release_context(ctx->context_key);
    return AERO_SOLVER_STATUS_OK;
}

AERO_LBM_CAPI_EXPORT int aero_solver_step_wind_tunnel(
    long long handle,
    const AeroBoundaryDesc* boundary,
    int steps,
    float* out_flow,
    int out_value_count
) {
    std::lock_guard<std::mutex> lock(g_solver_mutex);
    clear_error();
    SolverContext* ctx = lookup_context(handle);
    if (!ctx) {
        return AERO_SOLVER_STATUS_ERROR;
    }
    AeroBoundaryDesc effective_boundary{};
    aero_solver_default_boundary(&effective_boundary);
    if (boundary) {
        effective_boundary = *boundary;
    }
    return step_solver_locked(*ctx, effective_boundary, steps, out_flow, out_value_count)
        ? AERO_SOLVER_STATUS_OK
        : AERO_SOLVER_STATUS_ERROR;
}

AERO_LBM_CAPI_EXPORT int aero_solver_run_wind_tunnel(
    const AeroStepInput* input,
    AeroStepOutput* output
) {
    if (!input || !output || !output->flow_out) {
        std::lock_guard<std::mutex> lock(g_solver_mutex);
        set_error("missing step input or output");
        return AERO_SOLVER_STATUS_ERROR;
    }
    long long handle = 0;
    if (!aero_solver_create_with_grid(&input->grid, &handle)) {
        output->status = AERO_SOLVER_STATUS_ERROR;
        return AERO_SOLVER_STATUS_ERROR;
    }
    const int cells = input->grid.nx * input->grid.ny * input->grid.nz;
    if (input->solid_mask && !aero_solver_set_solid_mask(handle, input->solid_mask, cells)) {
        aero_solver_destroy(handle);
        output->status = AERO_SOLVER_STATUS_ERROR;
        return AERO_SOLVER_STATUS_ERROR;
    }
    if (input->prev_flow && !aero_solver_set_flow_state(handle, input->prev_flow, cells * kOutputChannels)) {
        aero_solver_destroy(handle);
        output->status = AERO_SOLVER_STATUS_ERROR;
        return AERO_SOLVER_STATUS_ERROR;
    }
    const int ok = aero_solver_step_wind_tunnel(
        handle,
        &input->boundary,
        input->steps,
        output->flow_out,
        cells * kOutputChannels
    );
    output->max_velocity = ok ? max_velocity_from_output(cells, output->flow_out, cells * kOutputChannels) : 0.0f;
    output->status = ok;
    aero_solver_destroy(handle);
    return ok;
}

AERO_LBM_CAPI_EXPORT void aero_solver_destroy(long long handle) {
    std::lock_guard<std::mutex> lock(g_solver_mutex);
    auto it = g_contexts.find(handle);
    if (it == g_contexts.end()) {
        return;
    }
    aero_lbm_release_context(it->second->context_key);
    g_contexts.erase(it);
}

AERO_LBM_CAPI_EXPORT const char* aero_solver_last_error(void) {
    return g_last_error.empty() ? aero_lbm_last_error() : g_last_error.c_str();
}

AERO_LBM_CAPI_EXPORT const char* aero_solver_runtime_info(void) {
    return aero_lbm_runtime_info();
}

}  // extern "C"
