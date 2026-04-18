#include <jni.h>

#include "aero_lbm_hydro_core.h"
#include "aero_lbm_thermal_core.h"
#include "aero_lbm_mesoscale.h"
#include "aero_lbm_capi.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(AERO_LBM_OPENCL) && !defined(CL_TARGET_OPENCL_VERSION)
#define CL_TARGET_OPENCL_VERSION 120
#endif

#if defined(AERO_LBM_OPENCL)
#include <CL/cl.h>
#endif

namespace aero_lbm::mesoscale {

bool valid(const DomainSpec& spec) {
    return hydro_core::valid_transport_spec(hydro_core::TransportSpec{
        spec.nx,
        spec.ny,
        spec.nz,
        spec.dx_m,
        spec.dt_s,
        spec.molecular_nu_m2_s,
        spec.prandtl_air,
        spec.turbulent_prandtl
    });
}

TransportCoefficients derive_transport(const DomainSpec& spec) {
    TransportCoefficients out{};
    if (!valid(spec)) {
        return out;
    }
    const hydro_core::TransportSpec transport_spec{
        spec.nx,
        spec.ny,
        spec.nz,
        spec.dx_m,
        spec.dt_s,
        spec.molecular_nu_m2_s,
        spec.prandtl_air,
        spec.turbulent_prandtl
    };
    const auto hydro = hydro_core::derive_hydro_transport(transport_spec);
    const auto thermal = thermal_core::derive_thermal_transport(transport_spec);
    out.velocity_scale_m_s_per_lattice = hydro.velocity_scale_m_s_per_lattice;
    out.nu_molecular_lattice = hydro.nu_molecular_lattice;
    out.alpha_molecular_lattice = thermal.alpha_molecular_lattice;
    out.tau_shear_molecular = hydro.tau_shear_molecular;
    out.tau_thermal_molecular = thermal.tau_thermal_molecular;
    return out;
}

float meters_per_second_to_lattice(float velocity_m_s, const DomainSpec& spec) {
    if (!valid(spec)) {
        return 0.0f;
    }
    return hydro_core::meters_per_second_to_lattice(
        velocity_m_s,
        hydro_core::TransportSpec{
            spec.nx,
            spec.ny,
            spec.nz,
            spec.dx_m,
            spec.dt_s,
            spec.molecular_nu_m2_s,
            spec.prandtl_air,
            spec.turbulent_prandtl
        }
    );
}

float lattice_to_meters_per_second(float velocity_lattice, const DomainSpec& spec) {
    if (!valid(spec)) {
        return 0.0f;
    }
    return hydro_core::lattice_to_meters_per_second(
        velocity_lattice,
        hydro_core::TransportSpec{
            spec.nx,
            spec.ny,
            spec.nz,
            spec.dx_m,
            spec.dt_s,
            spec.molecular_nu_m2_s,
            spec.prandtl_air,
            spec.turbulent_prandtl
        }
    );
}

}  // namespace aero_lbm::mesoscale

namespace {

constexpr std::uint32_t kMesoscaleAbiVersion = AERO_LBM_MESOSCALE_ABI_VERSION;
constexpr int kForcingChannels = AERO_LBM_MESOSCALE_FORCING_CHANNELS;
constexpr int kStateChannels = AERO_LBM_MESOSCALE_STATE_CHANNELS;
constexpr int kChTerrainHeight = 0;
constexpr int kChBiomeTemperature = 1;
constexpr int kChAmbientTarget = 2;
constexpr int kChDeepGroundTarget = 3;
constexpr int kChSurfaceTarget = 4;
constexpr int kChRoughness = 5;
constexpr int kChBackgroundWindX = 6;
constexpr int kChBackgroundWindZ = 7;
constexpr int kChSurfaceClass = 8;
constexpr int kChHumidity = 9;
constexpr int kChConvectiveHeating = 10;
constexpr int kChConvectiveMoistening = 11;
constexpr int kChConvectiveInflowX = 12;
constexpr int kChConvectiveInflowZ = 13;
constexpr int kChTornadoWindX = 14;
constexpr int kChTornadoWindZ = 15;
constexpr int kChTornadoHeating = 16;
constexpr int kChTornadoMoistening = 17;
constexpr int kChTornadoUpdraft = 18;
constexpr int kChNestedUpdraft = 19;
constexpr int kChNestedWindXDelta = 20;
constexpr int kChNestedWindZDelta = 21;
constexpr int kChNestedAmbientDelta = 22;
constexpr int kChNestedSurfaceDelta = 23;
constexpr int kOutAmbient = 0;
constexpr int kOutDeepGround = 1;
constexpr int kOutSurface = 2;
constexpr int kOutWindX = 3;
constexpr int kOutWindZ = 4;
constexpr int kMesoQ = 9;
constexpr std::array<int, kMesoQ> kMesoCx = {0, 1, -1, 0, 0, 1, -1, 1, -1};
constexpr std::array<int, kMesoQ> kMesoCz = {0, 0, 0, 1, -1, 1, -1, -1, 1};
constexpr std::array<int, kMesoQ> kMesoOpp = {0, 2, 1, 4, 3, 6, 5, 8, 7};
constexpr std::array<float, kMesoQ> kMesoW = {
    4.0f / 9.0f,
    1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f,
    1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f
};
constexpr float kEdgeBlendThicknessCells = 2.0f;
constexpr float kMaxLatticeWind = 0.20f;

struct CellTargets {
    float rho = 1.0f;
    float relax_ux = 0.0f;
    float relax_uz = 0.0f;
    float thermal_eq = 288.15f;
    float next_surface = 288.15f;
    float next_deep = 289.65f;
};

struct MesoscaleContextState {
    AeroLbmMesoscaleConfig config{};
    bool initialized = false;
    std::vector<float> rho;
    std::vector<float> ambient;
    std::vector<float> deep_ground;
    std::vector<float> surface;
    std::vector<float> wind_x;
    std::vector<float> wind_z;
    std::vector<float> f;
    std::vector<float> f_next;
    std::vector<float> g;
    std::vector<float> g_next;

#if defined(AERO_LBM_OPENCL)
    bool gpu_buffers_ready = false;
    bool gpu_seeded = false;
    cl_mem d_forcing = nullptr;
    cl_mem d_f = nullptr;
    cl_mem d_f_next = nullptr;
    cl_mem d_g = nullptr;
    cl_mem d_g_next = nullptr;
    cl_mem d_surface = nullptr;
    cl_mem d_surface_next = nullptr;
    cl_mem d_deep = nullptr;
    cl_mem d_deep_next = nullptr;
    cl_mem d_out_state = nullptr;
#endif
};

std::unordered_map<long long, MesoscaleContextState> g_mesoscale_contexts;
long long g_next_mesoscale_context_key = 1;

aero_lbm::mesoscale::DomainSpec to_domain_spec(const AeroLbmMesoscaleConfig& cfg) {
    return aero_lbm::mesoscale::DomainSpec{
        cfg.nx,
        cfg.ny,
        cfg.nz,
        cfg.dx_m,
        cfg.dt_s,
        cfg.molecular_nu_m2_s,
        cfg.prandtl_air,
        cfg.turbulent_prandtl
    };
}

bool mesoscale_config_valid(const AeroLbmMesoscaleConfig* cfg) {
    return cfg != nullptr
        && cfg->abi_version == kMesoscaleAbiVersion
        && cfg->struct_size >= sizeof(AeroLbmMesoscaleConfig)
        && aero_lbm::mesoscale::valid(to_domain_spec(*cfg));
}

int mesoscale_cells(const AeroLbmMesoscaleConfig& cfg) {
    return cfg.nx * cfg.ny * cfg.nz;
}

int mesoscale_index(int x, int y, int z, int ny, int nz) {
    return (x * ny + y) * nz + z;
}

int dist_index(int cell, int q) {
    return cell * kMesoQ + q;
}

float hydro_feq_2d(int q, float rho, float ux, float uz) {
    const float cu = 3.0f * (kMesoCx[q] * ux + kMesoCz[q] * uz);
    const float uu = ux * ux + uz * uz;
    return kMesoW[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

float thermal_feq_2d(int q, float temperature, float ux, float uz) {
    const float cu = 3.0f * (kMesoCx[q] * ux + kMesoCz[q] * uz);
    return kMesoW[q] * temperature * (1.0f + cu);
}

float clamp_lattice_speed(float u) {
    return std::clamp(u, -kMaxLatticeWind, kMaxLatticeWind);
}

void ensure_mesoscale_storage(MesoscaleContextState& ctx) {
    const int cells = mesoscale_cells(ctx.config);
    ctx.rho.resize(cells);
    ctx.ambient.resize(cells);
    ctx.deep_ground.resize(cells);
    ctx.surface.resize(cells);
    ctx.wind_x.resize(cells);
    ctx.wind_z.resize(cells);
    ctx.f.resize(cells * kMesoQ);
    ctx.f_next.resize(cells * kMesoQ);
    ctx.g.resize(cells * kMesoQ);
    ctx.g_next.resize(cells * kMesoQ);
}

void seed_mesoscale_state(
    MesoscaleContextState& ctx,
    const float* forcing,
    float velocity_scale_m_s_per_lattice
) {
    const int cells = mesoscale_cells(ctx.config);
    for (int i = 0; i < cells; ++i) {
        const int base = i * kForcingChannels;
        const float ambient_target = forcing[base + kChAmbientTarget] + forcing[base + kChNestedAmbientDelta];
        const float surface_target = forcing[base + kChSurfaceTarget] + forcing[base + kChNestedSurfaceDelta];
        const float bg_wind_x = forcing[base + kChBackgroundWindX] + forcing[base + kChNestedWindXDelta];
        const float bg_wind_z = forcing[base + kChBackgroundWindZ] + forcing[base + kChNestedWindZDelta];
        const float bg_wind_x_lattice = clamp_lattice_speed(
            bg_wind_x / std::max(1.0e-6f, velocity_scale_m_s_per_lattice)
        );
        const float bg_wind_z_lattice = clamp_lattice_speed(
            bg_wind_z / std::max(1.0e-6f, velocity_scale_m_s_per_lattice)
        );
        ctx.rho[i] = 1.0f;
        ctx.ambient[i] = ambient_target;
        ctx.deep_ground[i] = forcing[base + kChDeepGroundTarget];
        ctx.surface[i] = surface_target;
        ctx.wind_x[i] = bg_wind_x;
        ctx.wind_z[i] = bg_wind_z;
        for (int q = 0; q < kMesoQ; ++q) {
            ctx.f[dist_index(i, q)] = hydro_feq_2d(q, 1.0f, bg_wind_x_lattice, bg_wind_z_lattice);
            ctx.g[dist_index(i, q)] = thermal_feq_2d(q, ctx.ambient[i], bg_wind_x_lattice, bg_wind_z_lattice);
        }
    }
}

float horizontal_laplacian(
    const std::vector<float>& field,
    int x,
    int y,
    int z,
    int nx,
    int ny,
    int nz
) {
    const float center = field[mesoscale_index(x, y, z, ny, nz)];
    const float xp = field[mesoscale_index(std::min(nx - 1, x + 1), y, z, ny, nz)];
    const float xm = field[mesoscale_index(std::max(0, x - 1), y, z, ny, nz)];
    const float zp = field[mesoscale_index(x, y, std::min(nz - 1, z + 1), ny, nz)];
    const float zm = field[mesoscale_index(x, y, std::max(0, z - 1), ny, nz)];
    return xp + xm + zp + zm - 4.0f * center;
}

float vertical_laplacian(
    const std::vector<float>& field,
    int x,
    int y,
    int z,
    int ny,
    int nz
) {
    if (ny <= 1) {
        return 0.0f;
    }
    const float center = field[mesoscale_index(x, y, z, ny, nz)];
    const float yp = field[mesoscale_index(x, std::min(ny - 1, y + 1), z, ny, nz)];
    const float ym = field[mesoscale_index(x, std::max(0, y - 1), z, ny, nz)];
    return yp + ym - 2.0f * center;
}

float terrain_gradient_component(
    const float* forcing,
    int x,
    int y,
    int z,
    int nx,
    int ny,
    int nz,
    int axis
) {
    const int xp = std::min(nx - 1, x + 1);
    const int xm = std::max(0, x - 1);
    const int zp = std::min(nz - 1, z + 1);
    const int zm = std::max(0, z - 1);
    if (axis == 0) {
        const float hp = forcing[(mesoscale_index(xp, y, z, ny, nz) * kForcingChannels) + kChTerrainHeight];
        const float hm = forcing[(mesoscale_index(xm, y, z, ny, nz) * kForcingChannels) + kChTerrainHeight];
        return 0.5f * (hp - hm);
    }
    const float hp = forcing[(mesoscale_index(x, y, zp, ny, nz) * kForcingChannels) + kChTerrainHeight];
    const float hm = forcing[(mesoscale_index(x, y, zm, ny, nz) * kForcingChannels) + kChTerrainHeight];
    return 0.5f * (hp - hm);
}

CellTargets compute_cell_targets(
    const MesoscaleContextState& ctx,
    const float* forcing,
    const aero_lbm::mesoscale::TransportCoefficients& transport,
    int x,
    int y,
    int z
) {
    const int nx = ctx.config.nx;
    const int ny = ctx.config.ny;
    const int nz = ctx.config.nz;
    const int i = mesoscale_index(x, y, z, ny, nz);
    const int base = i * kForcingChannels;
    const float dt = std::max(1.0e-3f, ctx.config.dt_s);
    const float ambient_relax = std::clamp(dt / 600.0f, 0.0f, 1.0f);
    const float deep_relax = std::clamp(dt / 2400.0f, 0.0f, 1.0f);
    const float surface_relax = std::clamp(dt / 300.0f, 0.0f, 1.0f);
    const float wind_relax = std::clamp(dt / 120.0f, 0.0f, 1.0f);

    float rho = 0.0f;
    float ux = 0.0f;
    float uz = 0.0f;
    float ambient = 0.0f;
    for (int q = 0; q < kMesoQ; ++q) {
        const float fq = ctx.f[dist_index(i, q)];
        rho += fq;
        ux += fq * static_cast<float>(kMesoCx[q]);
        uz += fq * static_cast<float>(kMesoCz[q]);
        ambient += ctx.g[dist_index(i, q)];
    }
    rho = std::max(1.0e-6f, rho);
    ux /= rho;
    uz /= rho;

    const float ambient_target = forcing[base + kChAmbientTarget] + forcing[base + kChNestedAmbientDelta];
    const float deep_target = forcing[base + kChDeepGroundTarget];
    const float surface_target = forcing[base + kChSurfaceTarget] + forcing[base + kChNestedSurfaceDelta];
    const float roughness = std::max(0.0f, forcing[base + kChRoughness]);
    const float convective_heating = forcing[base + kChConvectiveHeating];
    const float convective_moistening = forcing[base + kChConvectiveMoistening];
    const float convective_inflow_x = forcing[base + kChConvectiveInflowX];
    const float convective_inflow_z = forcing[base + kChConvectiveInflowZ];
    const float tornado_wind_x = forcing[base + kChTornadoWindX];
    const float tornado_wind_z = forcing[base + kChTornadoWindZ];
    const float tornado_heating = forcing[base + kChTornadoHeating];
    const float tornado_moistening = forcing[base + kChTornadoMoistening];
    const float tornado_updraft = forcing[base + kChTornadoUpdraft];
    const float nested_updraft = forcing[base + kChNestedUpdraft];
    const float nested_wind_x_delta = forcing[base + kChNestedWindXDelta];
    const float nested_wind_z_delta = forcing[base + kChNestedWindZDelta];
    const float bg_wind_x = forcing[base + kChBackgroundWindX] + nested_wind_x_delta + convective_inflow_x + tornado_wind_x;
    const float bg_wind_z = forcing[base + kChBackgroundWindZ] + nested_wind_z_delta + convective_inflow_z + tornado_wind_z;
    const float humidity = std::clamp(forcing[base + kChHumidity] + convective_moistening + tornado_moistening, 0.0f, 1.0f);
    const float slope_x = terrain_gradient_component(forcing, x, y, z, nx, ny, nz, 0);
    const float slope_z = terrain_gradient_component(forcing, x, y, z, nx, ny, nz, 1);

    const float surface_h_lap = horizontal_laplacian(ctx.surface, x, y, z, nx, ny, nz);
    const float surface_v_lap = vertical_laplacian(ctx.surface, x, y, z, ny, nz);
    const float deep_v_lap = vertical_laplacian(ctx.deep_ground, x, y, z, ny, nz);
    const float ambient_h_lap = horizontal_laplacian(ctx.ambient, x, y, z, nx, ny, nz);
    const float ambient_v_lap = vertical_laplacian(ctx.ambient, x, y, z, ny, nz);

    const float next_deep = ctx.deep_ground[i]
        + deep_relax * (deep_target - ctx.deep_ground[i])
        + 0.01f * deep_v_lap;
    const float humidity_surface_damping = 1.0f - 0.12f * humidity;
    const float next_surface = ctx.surface[i]
        + surface_relax * humidity_surface_damping * (surface_target - ctx.surface[i])
        + 0.10f * (next_deep - ctx.surface[i])
        + 0.04f * convective_heating
        + 0.08f * tornado_heating
        + 0.05f * surface_h_lap
        + 0.025f * surface_v_lap;

    const float bg_wind_x_lattice = clamp_lattice_speed(
        bg_wind_x / std::max(1.0e-6f, transport.velocity_scale_m_s_per_lattice)
    );
    const float bg_wind_z_lattice = clamp_lattice_speed(
        bg_wind_z / std::max(1.0e-6f, transport.velocity_scale_m_s_per_lattice)
    );
    const float thermal_push = 0.0008f * (next_surface - ambient);
    const float terrain_steer_x = -0.005f * slope_x * bg_wind_z_lattice;
    const float terrain_steer_z = 0.005f * slope_z * bg_wind_x_lattice;
    const float drag = std::clamp(roughness * 0.02f, 0.0f, 0.25f);
    const float target_ux = clamp_lattice_speed(
        bg_wind_x_lattice + terrain_steer_x + thermal_push * slope_x - drag * ux
    );
    const float target_uz = clamp_lattice_speed(
        bg_wind_z_lattice + terrain_steer_z + thermal_push * slope_z - drag * uz
    );
    const float relax_ux = clamp_lattice_speed((1.0f - wind_relax) * ux + wind_relax * target_ux);
    const float relax_uz = clamp_lattice_speed((1.0f - wind_relax) * uz + wind_relax * target_uz);

    const float thermal_source = ambient_relax * (ambient_target - ambient)
        + (0.018f + 0.004f * humidity) * (next_surface - ambient)
        + 0.01f * (next_deep - ambient)
        + 0.08f * convective_heating * (0.5f + 0.5f * humidity)
        + 0.10f * tornado_heating * (0.45f + 0.55f * humidity)
        + 0.06f * tornado_updraft
        + 0.06f * nested_updraft
        + 0.01f * ambient_h_lap
        + 0.02f * ambient_v_lap;

    CellTargets out{};
    out.rho = rho;
    out.relax_ux = relax_ux;
    out.relax_uz = relax_uz;
    out.thermal_eq = ambient + thermal_source;
    out.next_surface = next_surface;
    out.next_deep = next_deep;
    return out;
}

void export_mesoscale_state(const MesoscaleContextState& ctx, float* out_state) {
    if (!out_state) {
        return;
    }
    const int cells = mesoscale_cells(ctx.config);
    for (int i = 0; i < cells; ++i) {
        const int base = i * kStateChannels;
        out_state[base + kOutAmbient] = ctx.ambient[i];
        out_state[base + kOutDeepGround] = ctx.deep_ground[i];
        out_state[base + kOutSurface] = ctx.surface[i];
        out_state[base + kOutWindX] = ctx.wind_x[i];
        out_state[base + kOutWindZ] = ctx.wind_z[i];
    }
}

int mesoscale_cpu_step(
    MesoscaleContextState& ctx,
    const AeroLbmMesoscaleConfig* config,
    const aero_lbm::mesoscale::TransportCoefficients& transport,
    const float* forcing,
    float* out_state
) {
    const int nx = config->nx;
    const int ny = config->ny;
    const int nz = config->nz;
    const int cells = mesoscale_cells(*config);
    const float tau_h = std::clamp(transport.tau_shear_molecular, 0.50001f, 3.0f);
    const float omega_h = 1.0f / tau_h;
    const float tau_t = std::clamp(transport.tau_thermal_molecular, 0.50001f, 3.0f);
    const float omega_t = 1.0f / tau_t;
    const float max_speed = 0.25f * transport.velocity_scale_m_s_per_lattice;
    std::vector<float> post_f(ctx.f.size());
    std::vector<float> post_g(ctx.g.size());
    std::vector<float> next_deep(cells);
    std::vector<float> next_surface(cells);

    for (int x = 0; x < nx; ++x) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const int i = mesoscale_index(x, y, z, ny, nz);
                const CellTargets targets = compute_cell_targets(ctx, forcing, transport, x, y, z);
                next_deep[i] = targets.next_deep;
                next_surface[i] = targets.next_surface;
                for (int q = 0; q < kMesoQ; ++q) {
                    const int d = dist_index(i, q);
                    const float feq = hydro_feq_2d(q, targets.rho, targets.relax_ux, targets.relax_uz);
                    const float geq = thermal_feq_2d(q, targets.thermal_eq, targets.relax_ux, targets.relax_uz);
                    post_f[d] = ctx.f[d] - omega_h * (ctx.f[d] - feq);
                    post_g[d] = ctx.g[d] - omega_t * (ctx.g[d] - geq);
                }
            }
        }
    }

    for (int x = 0; x < nx; ++x) {
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                const int i = mesoscale_index(x, y, z, ny, nz);
                const int base = i * kForcingChannels;
                const float boundary_bg_x = clamp_lattice_speed(
                    (forcing[base + kChBackgroundWindX]
                        + forcing[base + kChNestedWindXDelta]
                        + forcing[base + kChConvectiveInflowX]
                        + forcing[base + kChTornadoWindX])
                        / std::max(1.0e-6f, transport.velocity_scale_m_s_per_lattice)
                );
                const float boundary_bg_z = clamp_lattice_speed(
                    (forcing[base + kChBackgroundWindZ]
                        + forcing[base + kChNestedWindZDelta]
                        + forcing[base + kChConvectiveInflowZ]
                        + forcing[base + kChTornadoWindZ])
                        / std::max(1.0e-6f, transport.velocity_scale_m_s_per_lattice)
                );
                const float boundary_temp = forcing[base + kChAmbientTarget] + forcing[base + kChNestedAmbientDelta];
                const int edge_distance = std::min(std::min(x, z), std::min(nx - 1 - x, nz - 1 - z));
                const float edge_alpha = edge_distance <= 0
                    ? 1.0f
                    : edge_distance < static_cast<int>(kEdgeBlendThicknessCells) ? 0.5f : 0.0f;
                for (int q = 0; q < kMesoQ; ++q) {
                    const int sx = x - kMesoCx[q];
                    const int sz = z - kMesoCz[q];
                    const int d = dist_index(i, q);
                    float f_value;
                    float g_value;
                    if (sx >= 0 && sx < nx && sz >= 0 && sz < nz) {
                        const int src = mesoscale_index(sx, y, sz, ny, nz);
                        f_value = post_f[dist_index(src, q)];
                        g_value = post_g[dist_index(src, q)];
                    } else {
                        f_value = hydro_feq_2d(q, 1.0f, boundary_bg_x, boundary_bg_z);
                        g_value = thermal_feq_2d(q, boundary_temp, boundary_bg_x, boundary_bg_z);
                    }
                    if (edge_alpha > 0.0f) {
                        const float f_bg = hydro_feq_2d(q, 1.0f, boundary_bg_x, boundary_bg_z);
                        const float g_bg = thermal_feq_2d(q, boundary_temp, boundary_bg_x, boundary_bg_z);
                        f_value = (1.0f - edge_alpha) * f_value + edge_alpha * f_bg;
                        g_value = (1.0f - edge_alpha) * g_value + edge_alpha * g_bg;
                    }
                    ctx.f_next[d] = f_value;
                    ctx.g_next[d] = g_value;
                }
            }
        }
    }

    ctx.f.swap(ctx.f_next);
    ctx.g.swap(ctx.g_next);
    ctx.deep_ground.swap(next_deep);
    ctx.surface.swap(next_surface);

    for (int i = 0; i < cells; ++i) {
        float rho = 0.0f;
        float ux = 0.0f;
        float uz = 0.0f;
        float ambient = 0.0f;
        for (int q = 0; q < kMesoQ; ++q) {
            const float fq = ctx.f[dist_index(i, q)];
            rho += fq;
            ux += fq * static_cast<float>(kMesoCx[q]);
            uz += fq * static_cast<float>(kMesoCz[q]);
            ambient += ctx.g[dist_index(i, q)];
        }
        rho = std::max(1.0e-6f, rho);
        ux /= rho;
        uz /= rho;
        ctx.rho[i] = rho;
        ctx.ambient[i] = ambient;
        ctx.wind_x[i] = std::clamp(ux * transport.velocity_scale_m_s_per_lattice, -max_speed, max_speed);
        ctx.wind_z[i] = std::clamp(uz * transport.velocity_scale_m_s_per_lattice, -max_speed, max_speed);
    }

    export_mesoscale_state(ctx, out_state);
    return 1;
}

#if defined(AERO_LBM_OPENCL)

struct MesoscaleOpenClRuntime {
    bool attempted = false;
    bool available = false;
    std::string error;
    std::string device_name;
    cl_platform_id platform = nullptr;
    cl_device_id device = nullptr;
    cl_context context = nullptr;
    cl_command_queue queue = nullptr;
    cl_program program = nullptr;
    cl_kernel k_step = nullptr;
};

MesoscaleOpenClRuntime g_meso_opencl;

const char* kMesoscaleOpenClSource =
R"CLC(
constant int MESO_CX[9] = {0, 1, -1, 0, 0, 1, -1, 1, -1};
constant int MESO_CZ[9] = {0, 0, 0, 1, -1, 1, -1, -1, 1};
constant float MESO_W[9] = {
    4.0f / 9.0f,
    1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f,
    1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f
};

inline int mesoscale_index(int x, int y, int z, int ny, int nz) {
    return (x * ny + y) * nz + z;
}

inline int dist_index(int cell, int q) {
    return cell * 9 + q;
}

inline float clamp_lattice_speed(float u) {
    return clamp(u, -0.20f, 0.20f);
}

inline float hydro_feq_2d(int q, float rho, float ux, float uz) {
    float cu = 3.0f * ((float)MESO_CX[q] * ux + (float)MESO_CZ[q] * uz);
    float uu = ux * ux + uz * uz;
    return MESO_W[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

inline float thermal_feq_2d(int q, float temperature, float ux, float uz) {
    float cu = 3.0f * ((float)MESO_CX[q] * ux + (float)MESO_CZ[q] * uz);
    return MESO_W[q] * temperature * (1.0f + cu);
}

inline void recover_cell_state(__global const float* f_in, __global const float* g_in, int cell, float* rho, float* ux, float* uz, float* ambient) {
    float local_rho = 0.0f;
    float local_ux = 0.0f;
    float local_uz = 0.0f;
    float local_ambient = 0.0f;
    for (int q = 0; q < 9; ++q) {
        float fq = f_in[dist_index(cell, q)];
        local_rho += fq;
        local_ux += fq * (float)MESO_CX[q];
        local_uz += fq * (float)MESO_CZ[q];
        local_ambient += g_in[dist_index(cell, q)];
    }
    local_rho = fmax(local_rho, 1.0e-6f);
    *rho = local_rho;
    *ux = local_ux / local_rho;
    *uz = local_uz / local_rho;
    *ambient = local_ambient;
}

inline float horizontal_laplacian(__global const float* field, int x, int y, int z, int nx, int ny, int nz) {
    float center = field[mesoscale_index(x, y, z, ny, nz)];
    float xp = field[mesoscale_index(min(nx - 1, x + 1), y, z, ny, nz)];
    float xm = field[mesoscale_index(max(0, x - 1), y, z, ny, nz)];
    float zp = field[mesoscale_index(x, y, min(nz - 1, z + 1), ny, nz)];
    float zm = field[mesoscale_index(x, y, max(0, z - 1), ny, nz)];
    return xp + xm + zp + zm - 4.0f * center;
}

inline float vertical_laplacian(__global const float* field, int x, int y, int z, int ny, int nz) {
    if (ny <= 1) {
        return 0.0f;
    }
    float center = field[mesoscale_index(x, y, z, ny, nz)];
    float yp = field[mesoscale_index(x, min(ny - 1, y + 1), z, ny, nz)];
    float ym = field[mesoscale_index(x, max(0, y - 1), z, ny, nz)];
    return yp + ym - 2.0f * center;
}

inline float ambient_at(__global const float* g_in, int x, int y, int z, int ny, int nz) {
    int cell = mesoscale_index(x, y, z, ny, nz);
    float ambient = 0.0f;
    for (int q = 0; q < 9; ++q) {
        ambient += g_in[dist_index(cell, q)];
    }
    return ambient;
}

inline float ambient_horizontal_laplacian(__global const float* g_in, int x, int y, int z, int nx, int ny, int nz) {
    float center = ambient_at(g_in, x, y, z, ny, nz);
    float xp = ambient_at(g_in, min(nx - 1, x + 1), y, z, ny, nz);
    float xm = ambient_at(g_in, max(0, x - 1), y, z, ny, nz);
    float zp = ambient_at(g_in, x, y, min(nz - 1, z + 1), ny, nz);
    float zm = ambient_at(g_in, x, y, max(0, z - 1), ny, nz);
    return xp + xm + zp + zm - 4.0f * center;
}

inline float ambient_vertical_laplacian(__global const float* g_in, int x, int y, int z, int ny, int nz) {
    if (ny <= 1) {
        return 0.0f;
    }
    float center = ambient_at(g_in, x, y, z, ny, nz);
    float yp = ambient_at(g_in, x, min(ny - 1, y + 1), z, ny, nz);
    float ym = ambient_at(g_in, x, max(0, y - 1), z, ny, nz);
    return yp + ym - 2.0f * center;
}

inline float terrain_gradient_component(__global const float* forcing, int x, int y, int z, int nx, int ny, int nz, int axis) {
    int xp = min(nx - 1, x + 1);
    int xm = max(0, x - 1);
    int zp = min(nz - 1, z + 1);
    int zm = max(0, z - 1);
    if (axis == 0) {
        float hp = forcing[mesoscale_index(xp, y, z, ny, nz) * kForcingChannels + 0];
        float hm = forcing[mesoscale_index(xm, y, z, ny, nz) * kForcingChannels + 0];
        return 0.5f * (hp - hm);
    }
    float hp = forcing[mesoscale_index(x, y, zp, ny, nz) * kForcingChannels + 0];
    float hm = forcing[mesoscale_index(x, y, zm, ny, nz) * kForcingChannels + 0];
    return 0.5f * (hp - hm);
}

inline void compute_cell_targets(
    int x,
    int y,
    int z,
    int nx,
    int ny,
    int nz,
    float dt,
    float velocity_scale,
    __global const float* forcing,
    __global const float* f_in,
    __global const float* g_in,
    __global const float* surface_in,
    __global const float* deep_in,
    float* rho_out,
    float* ux_out,
    float* uz_out,
    float* thermal_eq_out,
    float* next_surface_out,
    float* next_deep_out
) {
    int cell = mesoscale_index(x, y, z, ny, nz);
    int base = cell * kForcingChannels;
    float rho;
    float ux;
    float uz;
    float ambient;
    recover_cell_state(f_in, g_in, cell, &rho, &ux, &uz, &ambient);

    float ambient_target = forcing[base + 2] + forcing[base + 22];
    float deep_target = forcing[base + 3];
    float surface_target = forcing[base + 4] + forcing[base + 23];
    float roughness = fmax(0.0f, forcing[base + 5]);
    float convective_heating = forcing[base + 10];
    float convective_moistening = forcing[base + 11];
    float convective_inflow_x = forcing[base + 12];
    float convective_inflow_z = forcing[base + 13];
    float tornado_wind_x = forcing[base + 14];
    float tornado_wind_z = forcing[base + 15];
    float tornado_heating = forcing[base + 16];
    float tornado_moistening = forcing[base + 17];
    float tornado_updraft = forcing[base + 18];
    float nested_updraft = forcing[base + 19];
    float bg_wind_x = forcing[base + 6] + forcing[base + 20] + convective_inflow_x + tornado_wind_x;
    float bg_wind_z = forcing[base + 7] + forcing[base + 21] + convective_inflow_z + tornado_wind_z;
    float humidity = clamp(forcing[base + 9] + convective_moistening + tornado_moistening, 0.0f, 1.0f);
    float slope_x = terrain_gradient_component(forcing, x, y, z, nx, ny, nz, 0);
    float slope_z = terrain_gradient_component(forcing, x, y, z, nx, ny, nz, 1);

    float ambient_relax = clamp(dt / 600.0f, 0.0f, 1.0f);
    float deep_relax = clamp(dt / 2400.0f, 0.0f, 1.0f);
    float surface_relax = clamp(dt / 300.0f, 0.0f, 1.0f);
    float wind_relax = clamp(dt / 120.0f, 0.0f, 1.0f);

    float surface_h_lap = horizontal_laplacian(surface_in, x, y, z, nx, ny, nz);
    float surface_v_lap = vertical_laplacian(surface_in, x, y, z, ny, nz);
    float deep_v_lap = vertical_laplacian(deep_in, x, y, z, ny, nz);
    float ambient_h_lap = ambient_horizontal_laplacian(g_in, x, y, z, nx, ny, nz);
    float ambient_v_lap = ambient_vertical_laplacian(g_in, x, y, z, ny, nz);

    float humidity_surface_damping = 1.0f - 0.12f * humidity;
    float next_deep = deep_in[cell]
        + deep_relax * (deep_target - deep_in[cell])
        + 0.01f * deep_v_lap;
    float next_surface = surface_in[cell]
        + surface_relax * humidity_surface_damping * (surface_target - surface_in[cell])
        + 0.10f * (next_deep - surface_in[cell])
        + 0.04f * convective_heating
        + 0.08f * tornado_heating
        + 0.05f * surface_h_lap
        + 0.025f * surface_v_lap;

    float bg_wind_x_lattice = clamp_lattice_speed(bg_wind_x / fmax(1.0e-6f, velocity_scale));
    float bg_wind_z_lattice = clamp_lattice_speed(bg_wind_z / fmax(1.0e-6f, velocity_scale));
    float thermal_push = 0.0008f * (next_surface - ambient);
    float terrain_steer_x = -0.005f * slope_x * bg_wind_z_lattice;
    float terrain_steer_z = 0.005f * slope_z * bg_wind_x_lattice;
    float drag = clamp(roughness * 0.02f, 0.0f, 0.25f);
    float target_ux = clamp_lattice_speed(bg_wind_x_lattice + terrain_steer_x + thermal_push * slope_x - drag * ux);
    float target_uz = clamp_lattice_speed(bg_wind_z_lattice + terrain_steer_z + thermal_push * slope_z - drag * uz);
    float relax_ux = clamp_lattice_speed((1.0f - wind_relax) * ux + wind_relax * target_ux);
    float relax_uz = clamp_lattice_speed((1.0f - wind_relax) * uz + wind_relax * target_uz);

    float thermal_source = ambient_relax * (ambient_target - ambient)
        + (0.018f + 0.004f * humidity) * (next_surface - ambient)
        + 0.01f * (next_deep - ambient)
        + 0.08f * convective_heating * (0.5f + 0.5f * humidity)
        + 0.10f * tornado_heating * (0.45f + 0.55f * humidity)
        + 0.06f * tornado_updraft
        + 0.06f * nested_updraft
        + 0.01f * ambient_h_lap
        + 0.02f * ambient_v_lap;

    *rho_out = rho;
    *ux_out = relax_ux;
    *uz_out = relax_uz;
    *thermal_eq_out = ambient + thermal_source;
    *next_surface_out = next_surface;
    *next_deep_out = next_deep;
}

__kernel void mesoscale_step(
    int nx,
    int ny,
    int nz,
    float dt,
    float velocity_scale,
    float tau_h,
    float tau_t,
    __global const float* forcing,
    __global const float* f_in,
    __global const float* g_in,
    __global const float* surface_in,
    __global const float* deep_in,
    __global float* f_out,
    __global float* g_out,
    __global float* surface_out,
    __global float* deep_out,
    __global float* out_state
) {
    int cells = nx * ny * nz;
    int gid = (int)get_global_id(0);
    if (gid >= cells) {
        return;
    }

    int z = gid % nz;
    int yz = gid / nz;
    int y = yz % ny;
    int x = yz / ny;

    float rho_self;
    float ux_self;
    float uz_self;
    float thermal_eq_self;
    float next_surface_self;
    float next_deep_self;
    compute_cell_targets(
        x, y, z, nx, ny, nz, dt, velocity_scale,
        forcing, f_in, g_in, surface_in, deep_in,
        &rho_self, &ux_self, &uz_self, &thermal_eq_self,
        &next_surface_self, &next_deep_self
    );

    deep_out[gid] = next_deep_self;
    surface_out[gid] = next_surface_self;

    int base = gid * kForcingChannels;
    float boundary_bg_x = clamp_lattice_speed((forcing[base + 6] + forcing[base + 20] + forcing[base + 12] + forcing[base + 14]) / fmax(1.0e-6f, velocity_scale));
    float boundary_bg_z = clamp_lattice_speed((forcing[base + 7] + forcing[base + 21] + forcing[base + 13] + forcing[base + 15]) / fmax(1.0e-6f, velocity_scale));
    float boundary_temp = forcing[base + 2] + forcing[base + 22];
    int edge_distance = min(min(x, z), min(nx - 1 - x, nz - 1 - z));
    float edge_alpha = edge_distance <= 0 ? 1.0f : (edge_distance < 2 ? 0.5f : 0.0f);

    float rho_next = 0.0f;
    float ux_next = 0.0f;
    float uz_next = 0.0f;
    float ambient_next = 0.0f;
    for (int q = 0; q < 9; ++q) {
        float fq_value;
        float gq_value;
        int sx = x - MESO_CX[q];
        int sz = z - MESO_CZ[q];
        if (sx >= 0 && sx < nx && sz >= 0 && sz < nz) {
            int src = mesoscale_index(sx, y, sz, ny, nz);
            float rho_src;
            float ux_src;
            float uz_src;
            float thermal_eq_src;
            float next_surface_src;
            float next_deep_src;
            compute_cell_targets(
                sx, y, sz, nx, ny, nz, dt, velocity_scale,
                forcing, f_in, g_in, surface_in, deep_in,
                &rho_src, &ux_src, &uz_src, &thermal_eq_src,
                &next_surface_src, &next_deep_src
            );
            int src_dist = dist_index(src, q);
            fq_value = f_in[src_dist] - (f_in[src_dist] - hydro_feq_2d(q, rho_src, ux_src, uz_src)) / tau_h;
            gq_value = g_in[src_dist] - (g_in[src_dist] - thermal_feq_2d(q, thermal_eq_src, ux_src, uz_src)) / tau_t;
        } else {
            fq_value = hydro_feq_2d(q, 1.0f, boundary_bg_x, boundary_bg_z);
            gq_value = thermal_feq_2d(q, boundary_temp, boundary_bg_x, boundary_bg_z);
        }

        if (edge_alpha > 0.0f) {
            float f_bg = hydro_feq_2d(q, 1.0f, boundary_bg_x, boundary_bg_z);
            float g_bg = thermal_feq_2d(q, boundary_temp, boundary_bg_x, boundary_bg_z);
            fq_value = (1.0f - edge_alpha) * fq_value + edge_alpha * f_bg;
            gq_value = (1.0f - edge_alpha) * gq_value + edge_alpha * g_bg;
        }

        int d = dist_index(gid, q);
        f_out[d] = fq_value;
        g_out[d] = gq_value;
        rho_next += fq_value;
        ux_next += fq_value * (float)MESO_CX[q];
        uz_next += fq_value * (float)MESO_CZ[q];
        ambient_next += gq_value;
    }

    rho_next = fmax(rho_next, 1.0e-6f);
    ux_next /= rho_next;
    uz_next /= rho_next;
    int out_base = gid * 5;
    out_state[out_base + 0] = ambient_next;
    out_state[out_base + 1] = next_deep_self;
    out_state[out_base + 2] = next_surface_self;
    out_state[out_base + 3] = clamp(ux_next * velocity_scale, -0.25f * velocity_scale, 0.25f * velocity_scale);
    out_state[out_base + 4] = clamp(uz_next * velocity_scale, -0.25f * velocity_scale, 0.25f * velocity_scale);
}
)CLC";

std::string read_build_log(cl_program program, cl_device_id device) {
    size_t log_size = 0;
    clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
    if (log_size == 0) {
        return {};
    }
    std::string log(log_size, '\0');
    clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, log_size, log.data(), nullptr);
    while (!log.empty() && (log.back() == '\0' || log.back() == '\n' || log.back() == '\r')) {
        log.pop_back();
    }
    return log;
}

std::string read_device_name(cl_device_id device) {
    size_t size = 0;
    clGetDeviceInfo(device, CL_DEVICE_NAME, 0, nullptr, &size);
    if (size == 0) {
        return {};
    }
    std::string name(size, '\0');
    clGetDeviceInfo(device, CL_DEVICE_NAME, size, name.data(), nullptr);
    while (!name.empty() && name.back() == '\0') {
        name.pop_back();
    }
    return name;
}

std::string format_opencl_error(const char* api, cl_int err) {
    return std::string(api) + " failed (" + std::to_string(static_cast<int>(err)) + ")";
}

void release_context_gpu_buffers(MesoscaleContextState& ctx) {
    auto release_buffer = [](cl_mem& buffer) {
        if (buffer) {
            clReleaseMemObject(buffer);
            buffer = nullptr;
        }
    };
    release_buffer(ctx.d_forcing);
    release_buffer(ctx.d_f);
    release_buffer(ctx.d_f_next);
    release_buffer(ctx.d_g);
    release_buffer(ctx.d_g_next);
    release_buffer(ctx.d_surface);
    release_buffer(ctx.d_surface_next);
    release_buffer(ctx.d_deep);
    release_buffer(ctx.d_deep_next);
    release_buffer(ctx.d_out_state);
    ctx.gpu_buffers_ready = false;
    ctx.gpu_seeded = false;
}

bool initialize_opencl_runtime() {
    if (g_meso_opencl.available) {
        return true;
    }
    if (g_meso_opencl.attempted) {
        return false;
    }
    g_meso_opencl.attempted = true;

    cl_uint platform_count = 0;
    cl_int err = clGetPlatformIDs(0, nullptr, &platform_count);
    if (err != CL_SUCCESS || platform_count == 0) {
        g_meso_opencl.error = "No OpenCL platform";
        return false;
    }

    std::vector<cl_platform_id> platforms(platform_count);
    err = clGetPlatformIDs(platform_count, platforms.data(), nullptr);
    if (err != CL_SUCCESS) {
        g_meso_opencl.error = format_opencl_error("clGetPlatformIDs", err);
        return false;
    }

    cl_platform_id selected_platform = nullptr;
    cl_device_id selected_device = nullptr;
    for (cl_platform_id platform : platforms) {
        if (clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &selected_device, nullptr) == CL_SUCCESS) {
            selected_platform = platform;
            break;
        }
    }
    if (!selected_device) {
        for (cl_platform_id platform : platforms) {
            if (clGetDeviceIDs(platform, CL_DEVICE_TYPE_DEFAULT, 1, &selected_device, nullptr) == CL_SUCCESS) {
                selected_platform = platform;
                break;
            }
        }
    }
    if (!selected_device) {
        g_meso_opencl.error = "No usable OpenCL device";
        return false;
    }

    cl_context context = clCreateContext(nullptr, 1, &selected_device, nullptr, nullptr, &err);
    if (err != CL_SUCCESS || !context) {
        g_meso_opencl.error = format_opencl_error("clCreateContext", err);
        return false;
    }
    cl_command_queue queue = clCreateCommandQueue(context, selected_device, 0, &err);
    if (err != CL_SUCCESS || !queue) {
        g_meso_opencl.error = format_opencl_error("clCreateCommandQueue", err);
        clReleaseContext(context);
        return false;
    }

    const char* src = kMesoscaleOpenClSource;
    const size_t src_len = std::strlen(kMesoscaleOpenClSource);
    cl_program program = clCreateProgramWithSource(context, 1, &src, &src_len, &err);
    if (err != CL_SUCCESS || !program) {
        g_meso_opencl.error = format_opencl_error("clCreateProgramWithSource", err);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        return false;
    }
    err = clBuildProgram(program, 1, &selected_device, "-cl-fast-relaxed-math", nullptr, nullptr);
    if (err != CL_SUCCESS) {
        const std::string log = read_build_log(program, selected_device);
        g_meso_opencl.error = log.empty()
            ? format_opencl_error("clBuildProgram", err)
            : format_opencl_error("clBuildProgram", err) + ": " + log;
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        return false;
    }
    cl_kernel k_step = clCreateKernel(program, "mesoscale_step", &err);
    if (err != CL_SUCCESS || !k_step) {
        g_meso_opencl.error = format_opencl_error("clCreateKernel(mesoscale_step)", err);
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        return false;
    }

    g_meso_opencl.platform = selected_platform;
    g_meso_opencl.device = selected_device;
    g_meso_opencl.context = context;
    g_meso_opencl.queue = queue;
    g_meso_opencl.program = program;
    g_meso_opencl.k_step = k_step;
    g_meso_opencl.device_name = read_device_name(selected_device);
    g_meso_opencl.available = true;
    return true;
}

bool ensure_context_gpu_buffers(MesoscaleContextState& ctx) {
    if (!g_meso_opencl.available) {
        return false;
    }
    if (ctx.gpu_buffers_ready) {
        return true;
    }

    const std::size_t cells = static_cast<std::size_t>(mesoscale_cells(ctx.config));
    const std::size_t forcing_bytes = cells * kForcingChannels * sizeof(float);
    const std::size_t state_bytes = cells * kStateChannels * sizeof(float);
    const std::size_t dist_bytes = cells * kMesoQ * sizeof(float);
    const std::size_t scalar_bytes = cells * sizeof(float);
    auto create_buffer = [&](cl_mem& target, cl_mem_flags flags, std::size_t bytes, const char* label) -> bool {
        cl_int err = CL_SUCCESS;
        target = clCreateBuffer(g_meso_opencl.context, flags, bytes, nullptr, &err);
        if (err != CL_SUCCESS || !target) {
            g_meso_opencl.error = format_opencl_error(label, err);
            return false;
        }
        return true;
    };

    if (!create_buffer(ctx.d_forcing, CL_MEM_READ_ONLY, forcing_bytes, "clCreateBuffer(d_forcing)")
        || !create_buffer(ctx.d_f, CL_MEM_READ_WRITE, dist_bytes, "clCreateBuffer(d_f)")
        || !create_buffer(ctx.d_f_next, CL_MEM_READ_WRITE, dist_bytes, "clCreateBuffer(d_f_next)")
        || !create_buffer(ctx.d_g, CL_MEM_READ_WRITE, dist_bytes, "clCreateBuffer(d_g)")
        || !create_buffer(ctx.d_g_next, CL_MEM_READ_WRITE, dist_bytes, "clCreateBuffer(d_g_next)")
        || !create_buffer(ctx.d_surface, CL_MEM_READ_WRITE, scalar_bytes, "clCreateBuffer(d_surface)")
        || !create_buffer(ctx.d_surface_next, CL_MEM_READ_WRITE, scalar_bytes, "clCreateBuffer(d_surface_next)")
        || !create_buffer(ctx.d_deep, CL_MEM_READ_WRITE, scalar_bytes, "clCreateBuffer(d_deep)")
        || !create_buffer(ctx.d_deep_next, CL_MEM_READ_WRITE, scalar_bytes, "clCreateBuffer(d_deep_next)")
        || !create_buffer(ctx.d_out_state, CL_MEM_WRITE_ONLY, state_bytes, "clCreateBuffer(d_out_state)")) {
        release_context_gpu_buffers(ctx);
        return false;
    }
    ctx.gpu_buffers_ready = true;
    ctx.gpu_seeded = false;
    return true;
}

bool upload_seeded_state_to_gpu(MesoscaleContextState& ctx) {
    if (!ctx.gpu_buffers_ready) {
        return false;
    }
    const std::size_t cells = static_cast<std::size_t>(mesoscale_cells(ctx.config));
    const std::size_t dist_bytes = cells * kMesoQ * sizeof(float);
    const std::size_t scalar_bytes = cells * sizeof(float);
    cl_int err = CL_SUCCESS;
    err = clEnqueueWriteBuffer(g_meso_opencl.queue, ctx.d_f, CL_TRUE, 0, dist_bytes, ctx.f.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) { g_meso_opencl.error = format_opencl_error("clEnqueueWriteBuffer(d_f)", err); return false; }
    err = clEnqueueWriteBuffer(g_meso_opencl.queue, ctx.d_g, CL_TRUE, 0, dist_bytes, ctx.g.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) { g_meso_opencl.error = format_opencl_error("clEnqueueWriteBuffer(d_g)", err); return false; }
    err = clEnqueueWriteBuffer(g_meso_opencl.queue, ctx.d_surface, CL_TRUE, 0, scalar_bytes, ctx.surface.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) { g_meso_opencl.error = format_opencl_error("clEnqueueWriteBuffer(d_surface)", err); return false; }
    err = clEnqueueWriteBuffer(g_meso_opencl.queue, ctx.d_deep, CL_TRUE, 0, scalar_bytes, ctx.deep_ground.data(), 0, nullptr, nullptr);
    if (err != CL_SUCCESS) { g_meso_opencl.error = format_opencl_error("clEnqueueWriteBuffer(d_deep)", err); return false; }
    ctx.gpu_seeded = true;
    return true;
}

bool mesoscale_opencl_step(
    MesoscaleContextState& ctx,
    const AeroLbmMesoscaleConfig* config,
    const aero_lbm::mesoscale::TransportCoefficients& transport,
    const float* forcing,
    float* out_state
) {
    if (!initialize_opencl_runtime()) {
        return false;
    }
    if (!ensure_context_gpu_buffers(ctx)) {
        return false;
    }
    if (!ctx.gpu_seeded) {
        seed_mesoscale_state(ctx, forcing, transport.velocity_scale_m_s_per_lattice);
        if (!upload_seeded_state_to_gpu(ctx)) {
            return false;
        }
    }

    const std::size_t cells = static_cast<std::size_t>(mesoscale_cells(*config));
    const std::size_t forcing_bytes = cells * kForcingChannels * sizeof(float);
    const std::size_t state_bytes = cells * kStateChannels * sizeof(float);
    cl_int err = clEnqueueWriteBuffer(g_meso_opencl.queue, ctx.d_forcing, CL_TRUE, 0, forcing_bytes, forcing, 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_meso_opencl.error = format_opencl_error("clEnqueueWriteBuffer(d_forcing)", err);
        return false;
    }

    int arg = 0;
    auto set_arg = [&](size_t size, const void* value, const char* label) -> bool {
        cl_int local_err = clSetKernelArg(g_meso_opencl.k_step, arg++, size, value);
        if (local_err != CL_SUCCESS) {
            g_meso_opencl.error = format_opencl_error(label, local_err);
            return false;
        }
        return true;
    };

    const float dt = std::max(1.0e-3f, config->dt_s);
    const float tau_h = std::clamp(transport.tau_shear_molecular, 0.50001f, 3.0f);
    const float tau_t = std::clamp(transport.tau_thermal_molecular, 0.50001f, 3.0f);
    if (!set_arg(sizeof(cl_int), &config->nx, "clSetKernelArg(nx)")
        || !set_arg(sizeof(cl_int), &config->ny, "clSetKernelArg(ny)")
        || !set_arg(sizeof(cl_int), &config->nz, "clSetKernelArg(nz)")
        || !set_arg(sizeof(cl_float), &dt, "clSetKernelArg(dt)")
        || !set_arg(sizeof(cl_float), &transport.velocity_scale_m_s_per_lattice, "clSetKernelArg(velocity_scale)")
        || !set_arg(sizeof(cl_float), &tau_h, "clSetKernelArg(tau_h)")
        || !set_arg(sizeof(cl_float), &tau_t, "clSetKernelArg(tau_t)")
        || !set_arg(sizeof(cl_mem), &ctx.d_forcing, "clSetKernelArg(d_forcing)")
        || !set_arg(sizeof(cl_mem), &ctx.d_f, "clSetKernelArg(d_f)")
        || !set_arg(sizeof(cl_mem), &ctx.d_g, "clSetKernelArg(d_g)")
        || !set_arg(sizeof(cl_mem), &ctx.d_surface, "clSetKernelArg(d_surface)")
        || !set_arg(sizeof(cl_mem), &ctx.d_deep, "clSetKernelArg(d_deep)")
        || !set_arg(sizeof(cl_mem), &ctx.d_f_next, "clSetKernelArg(d_f_next)")
        || !set_arg(sizeof(cl_mem), &ctx.d_g_next, "clSetKernelArg(d_g_next)")
        || !set_arg(sizeof(cl_mem), &ctx.d_surface_next, "clSetKernelArg(d_surface_next)")
        || !set_arg(sizeof(cl_mem), &ctx.d_deep_next, "clSetKernelArg(d_deep_next)")
        || !set_arg(sizeof(cl_mem), &ctx.d_out_state, "clSetKernelArg(d_out_state)")) {
        return false;
    }

    const size_t global = cells;
    err = clEnqueueNDRangeKernel(g_meso_opencl.queue, g_meso_opencl.k_step, 1, nullptr, &global, nullptr, 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_meso_opencl.error = format_opencl_error("clEnqueueNDRangeKernel(mesoscale_step)", err);
        return false;
    }
    err = clFinish(g_meso_opencl.queue);
    if (err != CL_SUCCESS) {
        g_meso_opencl.error = format_opencl_error("clFinish(mesoscale_step)", err);
        return false;
    }
    err = clEnqueueReadBuffer(g_meso_opencl.queue, ctx.d_out_state, CL_TRUE, 0, state_bytes, out_state, 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        g_meso_opencl.error = format_opencl_error("clEnqueueReadBuffer(d_out_state)", err);
        return false;
    }

    std::swap(ctx.d_f, ctx.d_f_next);
    std::swap(ctx.d_g, ctx.d_g_next);
    std::swap(ctx.d_surface, ctx.d_surface_next);
    std::swap(ctx.d_deep, ctx.d_deep_next);

    const float max_speed = 0.25f * transport.velocity_scale_m_s_per_lattice;
    for (std::size_t i = 0; i < cells; ++i) {
        const int base = static_cast<int>(i) * kStateChannels;
        ctx.ambient[i] = out_state[base + kOutAmbient];
        ctx.deep_ground[i] = out_state[base + kOutDeepGround];
        ctx.surface[i] = out_state[base + kOutSurface];
        ctx.wind_x[i] = std::clamp(out_state[base + kOutWindX], -max_speed, max_speed);
        ctx.wind_z[i] = std::clamp(out_state[base + kOutWindZ], -max_speed, max_speed);
        ctx.rho[i] = 1.0f;
    }
    return true;
}

#endif

}  // namespace

extern "C" {

AERO_LBM_CAPI_EXPORT void aero_lbm_mesoscale_default_config(AeroLbmMesoscaleConfig* out_config) {
    if (!out_config) {
        return;
    }
    *out_config = AeroLbmMesoscaleConfig{};
    out_config->abi_version = kMesoscaleAbiVersion;
    out_config->struct_size = sizeof(AeroLbmMesoscaleConfig);
    out_config->nx = 32;
    out_config->ny = 4;
    out_config->nz = 32;
    out_config->dx_m = 64.0f;
    out_config->dt_s = 5.0f;
    out_config->molecular_nu_m2_s = 1.5e-5f;
    out_config->prandtl_air = 0.71f;
    out_config->turbulent_prandtl = 0.85f;
    out_config->reference_density_kg_m3 = 1.225f;
    out_config->ambient_air_temperature_k = 288.15f;
    out_config->deep_ground_temperature_k = 289.65f;
    out_config->background_wind_m_s[0] = 0.0f;
    out_config->background_wind_m_s[1] = 0.0f;
    out_config->background_wind_m_s[2] = 0.0f;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_mesoscale_derive_transport(
    const AeroLbmMesoscaleConfig* config,
    AeroLbmMesoscaleTransport* out_transport
) {
    if (!mesoscale_config_valid(config) || !out_transport) {
        return 0;
    }
    const auto derived = aero_lbm::mesoscale::derive_transport(to_domain_spec(*config));
    out_transport->velocity_scale_m_s_per_lattice = derived.velocity_scale_m_s_per_lattice;
    out_transport->nu_molecular_lattice = derived.nu_molecular_lattice;
    out_transport->alpha_molecular_lattice = derived.alpha_molecular_lattice;
    out_transport->tau_shear_molecular = derived.tau_shear_molecular;
    out_transport->tau_thermal_molecular = derived.tau_thermal_molecular;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_mesoscale_create_context(
    const AeroLbmMesoscaleConfig* config,
    long long* out_context_key
) {
    if (!mesoscale_config_valid(config) || !out_context_key) {
        return 0;
    }
    const long long key = g_next_mesoscale_context_key++;
    MesoscaleContextState ctx{};
    ctx.config = *config;
    ensure_mesoscale_storage(ctx);
    g_mesoscale_contexts.emplace(key, std::move(ctx));
    *out_context_key = key;
    return 1;
}

AERO_LBM_CAPI_EXPORT int aero_lbm_mesoscale_step_context(
    long long context_key,
    const AeroLbmMesoscaleConfig* config,
    const float* forcing,
    float* out_state
) {
    if (!mesoscale_config_valid(config) || !forcing || !out_state) {
        return 0;
    }
    auto it = g_mesoscale_contexts.find(context_key);
    if (it == g_mesoscale_contexts.end()) {
        return 0;
    }

    MesoscaleContextState& ctx = it->second;
    const bool shape_changed = ctx.config.nx != config->nx || ctx.config.ny != config->ny || ctx.config.nz != config->nz;
    if (shape_changed) {
        ctx.config = *config;
        ensure_mesoscale_storage(ctx);
#if defined(AERO_LBM_OPENCL)
        release_context_gpu_buffers(ctx);
#endif
        ctx.initialized = false;
    } else {
        ctx.config = *config;
    }

    const auto transport = aero_lbm::mesoscale::derive_transport(to_domain_spec(*config));
    if (!ctx.initialized) {
        seed_mesoscale_state(ctx, forcing, transport.velocity_scale_m_s_per_lattice);
        ctx.initialized = true;
    }

#if defined(AERO_LBM_OPENCL)
    if (mesoscale_opencl_step(ctx, config, transport, forcing, out_state)) {
        return 1;
    }
    release_context_gpu_buffers(ctx);
    ctx.initialized = false;
    seed_mesoscale_state(ctx, forcing, transport.velocity_scale_m_s_per_lattice);
    ctx.initialized = true;
#endif

    return mesoscale_cpu_step(ctx, config, transport, forcing, out_state);
}

AERO_LBM_CAPI_EXPORT void aero_lbm_mesoscale_release_context(long long context_key) {
    auto it = g_mesoscale_contexts.find(context_key);
    if (it == g_mesoscale_contexts.end()) {
        return;
    }
#if defined(AERO_LBM_OPENCL)
    release_context_gpu_buffers(it->second);
#endif
    g_mesoscale_contexts.erase(it);
}

JNIEXPORT jlong JNICALL Java_com_aerodynamics4mc_runtime_MesoscaleNativeBridge_nativeCreateContext(
    JNIEnv*,
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
    long long key = 0;
    return aero_lbm_mesoscale_create_context(&cfg, &key) ? static_cast<jlong>(key) : 0;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_runtime_MesoscaleNativeBridge_nativeStepContext(
    JNIEnv* env,
    jclass,
    jlong context_key,
    jint nx,
    jint ny,
    jint nz,
    jfloat dx_m,
    jfloat dt_s,
    jfloat molecular_nu_m2_s,
    jfloat prandtl_air,
    jfloat turbulent_prandtl,
    jfloatArray forcing,
    jfloatArray out_state
) {
    if (!forcing || !out_state) {
        return JNI_FALSE;
    }
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

    const int cells = cfg.nx * cfg.ny * cfg.nz;
    if (env->GetArrayLength(forcing) != cells * kForcingChannels
        || env->GetArrayLength(out_state) != cells * kStateChannels) {
        return JNI_FALSE;
    }

    jboolean force_copy = JNI_FALSE;
    jboolean out_copy = JNI_FALSE;
    jfloat* forcing_ptr = env->GetFloatArrayElements(forcing, &force_copy);
    jfloat* out_ptr = env->GetFloatArrayElements(out_state, &out_copy);
    if (!forcing_ptr || !out_ptr) {
        if (forcing_ptr) {
            env->ReleaseFloatArrayElements(forcing, forcing_ptr, JNI_ABORT);
        }
        if (out_ptr) {
            env->ReleaseFloatArrayElements(out_state, out_ptr, 0);
        }
        return JNI_FALSE;
    }

    const int ok = aero_lbm_mesoscale_step_context(
        static_cast<long long>(context_key),
        &cfg,
        forcing_ptr,
        out_ptr
    );
    env->ReleaseFloatArrayElements(forcing, forcing_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(out_state, out_ptr, 0);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_runtime_MesoscaleNativeBridge_nativeReleaseContext(
    JNIEnv*,
    jclass,
    jlong context_key
) {
    aero_lbm_mesoscale_release_context(static_cast<long long>(context_key));
}

}  // extern "C"
