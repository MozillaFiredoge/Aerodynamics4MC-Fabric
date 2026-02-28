#include <jni.h>

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

constexpr float kLatticeSoundSpeed = 0.57735026919f;
constexpr float kMaxMach = 0.60f;
constexpr float kHardMaxLatticeSpeed = kLatticeSoundSpeed * kMaxMach;
constexpr float kCs2 = 1.0f / 3.0f;

constexpr float kRhoMin = 0.97f;
constexpr float kRhoMax = 1.03f;
constexpr float kPressureMin = -0.03f;
constexpr float kPressureMax = 0.03f;

// D3Q27 cumulant closure with low-viscosity baseline tau.
constexpr float kTauShear = 0.51503f;
constexpr float kTauShearMin = 0.51501f;
constexpr float kTauShearMax = 0.95f;
constexpr float kTauNormal = 0.51504f;
constexpr float kTauNormalMin = 0.51501f;
constexpr float kTauNormalMax = 0.95f;
constexpr bool kEnableSgs = true;
constexpr float kSgsC = 0.10f;
constexpr float kSgsC2 = kSgsC * kSgsC;
constexpr float kSgsNutToNu0Max = 1.0f;
constexpr float kSgsBulkCoupling = 0.30f;

constexpr int kSpongeLayers = 6;
constexpr float kSpongeStrength = 0.10f;
constexpr float kBoundaryConvectiveBeta = 0.30f;

constexpr float kObstacleBounceBlend = 0.30f;
constexpr float kFanBeta = 0.07f;
constexpr float kFanTargetScale = 1.0f / 30.0f;
constexpr float kFanTargetMax = 0.34f;
constexpr float kFanNoiseAmp = 0.02f;
constexpr float kFanSpeedSoftCap = 0.30f;
constexpr float kFanSpeedDampWidth = 0.06f;
constexpr float kStateNudge = 0.0f;
constexpr float kMaxSpeed = kHardMaxLatticeSpeed;

struct Config {
    int grid_size = 0;
    int input_channels = 0;
    int output_channels = 0;
    bool initialized = false;
    bool opencl_enabled = false;
    std::string runtime_info;
};

struct ContextState {
    int n = 0;
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
    std::vector<uint8_t> obstacle;
    std::uint64_t step_counter = 0;

#if defined(AERO_LBM_OPENCL)
    bool gpu_buffers_ready = false;
    bool gpu_initialized = false;
    cl_mem d_payload = nullptr;
    cl_mem d_f = nullptr;
    cl_mem d_f_post = nullptr;
    cl_mem d_output = nullptr;
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
using Clock = std::chrono::steady_clock;

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

inline std::size_t cell_index(int x, int y, int z, int n) {
    return (static_cast<std::size_t>(x) * n + y) * n + z;
}

// 极为关键的一步：CPU 侧同样采用 SoA (Structure of Arrays) 布局！
inline std::size_t dist_index(std::size_t cell, int q, std::size_t cells) {
    return static_cast<std::size_t>(q) * cells + cell;
}

inline float clampf(float v, float lo, float hi) {
    return std::min(hi, std::max(lo, v));
}

inline float feq(int q, float rho, float ux, float uy, float uz) {
    const float cu = 3.0f * (kCx[q] * ux + kCy[q] * uy + kCz[q] * uz);
    const float uu = ux * ux + uy * uy + uz * uz;
    return kW[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

inline void decode_cell(std::size_t cell, int n, int& x, int& y, int& z) {
    const int yz = n * n;
    x = static_cast<int>(cell / static_cast<std::size_t>(yz));
    const int rem = static_cast<int>(cell - static_cast<std::size_t>(x) * yz);
    y = rem / n;
    z = rem - y * n;
}

inline bool solid_or_oob(const ContextState& ctx, int x, int y, int z) {
    if (x < 0 || y < 0 || z < 0 || x >= ctx.n || y >= ctx.n || z >= ctx.n) return true;
    return ctx.obstacle[cell_index(x, y, z, ctx.n)] != 0;
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

inline float sponge_alpha(int n, int x, int y, int z) {
    if (kSpongeLayers <= 0) return 0.0f;
    const int d = std::min({x, y, z, n - 1 - x, n - 1 - y, n - 1 - z});
    if (d >= kSpongeLayers) return 0.0f;
    const float eta = static_cast<float>(kSpongeLayers - d) / static_cast<float>(kSpongeLayers);
    return clampf(kSpongeStrength * eta * eta, 0.0f, 0.95f);
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
    if (kObstacleBounceBlend <= 0.0f) return bounced;

    const int s2x = x - 2 * kCx[q];
    const int s2y = y - 2 * kCy[q];
    const int s2z = z - 2 * kCz[q];
    if (s2x < 0 || s2y < 0 || s2z < 0 || s2x >= ctx.n || s2y >= ctx.n || s2z >= ctx.n) return bounced;

    const std::size_t src2 = cell_index(s2x, s2y, s2z, ctx.n);
    if (ctx.obstacle[src2]) return bounced;

    const float upstream = ctx.f_post[dist_index(src2, q, ctx.cells)];
    return bounced + kObstacleBounceBlend * (upstream - bounced);
}

inline float boundary_convective_value(
    const ContextState& ctx, std::size_t cell, int x, int y, int z, int q, int sx, int sy, int sz
) {
    const int dx = (sx < 0 || sx >= ctx.n) ? kCx[q] : 0;
    const int dy = (sy < 0 || sy >= ctx.n) ? kCy[q] : 0;
    const int dz = (sz < 0 || sz >= ctx.n) ? kCz[q] : 0;

    const int i1x = std::clamp(x + dx, 0, ctx.n - 1);
    const int i1y = std::clamp(y + dy, 0, ctx.n - 1);
    const int i1z = std::clamp(z + dz, 0, ctx.n - 1);
    const std::size_t src1 = cell_index(i1x, i1y, i1z, ctx.n);
    if (ctx.obstacle[src1]) return obstacle_bounce_value(ctx, cell, x, y, z, q);

    const float f1 = ctx.f_post[dist_index(src1, q, ctx.cells)];
    const int i2x = std::clamp(x + 2 * dx, 0, ctx.n - 1);
    const int i2y = std::clamp(y + 2 * dy, 0, ctx.n - 1);
    const int i2z = std::clamp(z + 2 * dz, 0, ctx.n - 1);
    const std::size_t src2 = cell_index(i2x, i2y, i2z, ctx.n);
    if (ctx.obstacle[src2]) return f1;

    const float f2 = ctx.f_post[dist_index(src2, q, ctx.cells)];
    return f1 + kBoundaryConvectiveBeta * (f1 - f2);
}

void allocate_cpu_context(ContextState& ctx, int n) {
    ctx.n = n;
    ctx.cells = static_cast<std::size_t>(n) * n * n;
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
    ctx.obstacle.assign(ctx.cells, 0);
}

void ingest_payload(ContextState& ctx, const float* payload, int in_channels) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t base = cell * in_channels;
        ctx.obstacle[cell] = payload[base + kChannelObstacle] > 0.5f ? 1 : 0;
        ctx.fan_mask[cell] = clampf(payload[base + kChannelFanMask], 0.0f, 1.0f);
        ctx.fan_ux[cell] = payload[base + kChannelFanVx];
        ctx.fan_uy[cell] = payload[base + kChannelFanVy];
        ctx.fan_uz[cell] = payload[base + kChannelFanVz];
        ctx.ref_ux[cell] = payload[base + kChannelStateVx];
        ctx.ref_uy[cell] = payload[base + kChannelStateVy];
        ctx.ref_uz[cell] = payload[base + kChannelStateVz];
        ctx.ref_pressure[cell] = clampf(payload[base + kChannelStateP], kPressureMin, kPressureMax);
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
    }
    ctx.cpu_initialized = true;
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
        for (int q = 0; q < kQ; ++q) {
            const float fq = ctx.f[dist_index(cell, q, ctx.cells)];
            f_local[q] = fq;
            rho += fq;
            mom_x += fq * static_cast<float>(kCx[q]);
            mom_y += fq * static_cast<float>(kCy[q]);
            mom_z += fq * static_cast<float>(kCz[q]);
        }

        const float rho_safe = std::max(1e-6f, rho);
        const float inv_rho = 1.0f / rho_safe;
        float ux = mom_x * inv_rho;
        float uy = mom_y * inv_rho;
        float uz = mom_z * inv_rho;
        const float speed_pre = std::sqrt(ux * ux + uy * uy + uz * uz);

        float fx = 0.0f;
        float fy = 0.0f;
        float fz = 0.0f;
        if (ctx.fan_mask[cell] > 0.0f) {
            const float fan_norm = std::sqrt(ctx.fan_ux[cell] * ctx.fan_ux[cell] + ctx.fan_uy[cell] * ctx.fan_uy[cell] + ctx.fan_uz[cell] * ctx.fan_uz[cell]);
            if (fan_norm > 1e-8f) {
                const float inv_norm = 1.0f / fan_norm;
                const float noise = 1.0f + kFanNoiseAmp * hash_signed_noise(cell, ctx.step_counter);
                const float target_speed = clampf(
                    fan_norm * kFanTargetScale * std::max(0.0f, noise),
                    0.0f,
                    kFanTargetMax
                );
                const float target_ux = ctx.fan_ux[cell] * inv_norm * target_speed;
                const float target_uy = ctx.fan_uy[cell] * inv_norm * target_speed;
                const float target_uz = ctx.fan_uz[cell] * inv_norm * target_speed;
                float speed_damp = 1.0f;
                if (speed_pre > kFanSpeedSoftCap) {
                    const float r = (speed_pre - kFanSpeedSoftCap) / std::max(1e-4f, kFanSpeedDampWidth);
                    speed_damp = 1.0f / (1.0f + r * r);
                }
                const float beta = ctx.fan_mask[cell] * kFanBeta * speed_damp;
                fx = beta * rho_safe * (target_ux - ux);
                fy = beta * rho_safe * (target_uy - uy);
                fz = beta * rho_safe * (target_uz - uz);
            }
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

        float speed2 = ux * ux + uy * uy + uz * uz;
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

        float tau_shear_local = kTauShear;
        float tau_normal_local = clampf(kTauNormal, kTauNormalMin, kTauNormalMax);
        if (kEnableSgs) {
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
        decode_cell(cell, ctx.n, cx, cy, cz);
        const float alpha = sponge_alpha(ctx.n, cx, cy, cz);
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
    const int n = ctx.n;
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < n; ++y) {
            for (int z = 0; z < n; ++z) {
                const std::size_t cell = cell_index(x, y, z, n);
                std::array<float, kQ> f_local{};
                if (ctx.obstacle[cell]) {
                    for (int q = 0; q < kQ; ++q) {
                        f_local[q] = ctx.f_post[dist_index(cell, kOpp[q], ctx.cells)];
                    }
                } else {
                    for (int q = 0; q < kQ; ++q) {
                        const int sx = x - kCx[q];
                        const int sy = y - kCy[q];
                        const int sz = z - kCz[q];
                        if (sx < 0 || sy < 0 || sz < 0 || sx >= n || sy >= n || sz >= n) {
                            f_local[q] = boundary_convective_value(ctx, cell, x, y, z, q, sx, sy, sz);
                            continue;
                        }

                        const std::size_t src = cell_index(sx, sy, sz, n);
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
            rho += fq;
            ux += fq * kCx[q]; uy += fq * kCy[q]; uz += fq * kCz[q];
        }
        rho = clampf(rho, kRhoMin, kRhoMax);
        ux /= rho; uy /= rho; uz /= rho;
        if (std::fabs(rho - 1.0f) < 1e-6f) rho = 1.0f;
        if (std::fabs(ux) < 1e-7f) ux = 0.0f;
        if (std::fabs(uy) < 1e-7f) uy = 0.0f;
        if (std::fabs(uz) < 1e-7f) uz = 0.0f;

        out[out_base + 0] = ux; out[out_base + 1] = uy; out[out_base + 2] = uz; out[out_base + 3] = rho - 1.0f;
        for (int c = 4; c < out_channels; ++c) out[out_base + c] = 0.0f;
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

__constant float TAU_SHEAR = 0.51503f;
__constant float TAU_SHEAR_MIN = 0.51501f;
__constant float TAU_SHEAR_MAX = 0.95f;
__constant float TAU_NORMAL = 0.51504f;
__constant float TAU_NORMAL_MIN = 0.51501f;
__constant float TAU_NORMAL_MAX = 0.95f;
__constant int SGS_ENABLED = 1;
__constant float SGS_C2 = 0.01f;
__constant float SGS_NUT_TO_NU0_MAX = 1.0f;
__constant float SGS_BULK_COUPLING = 0.30f;
__constant int SPONGE_LAYERS = 6;
__constant float SPONGE_STRENGTH = 0.10f;
__constant float BOUNDARY_CONVECTIVE_BETA = 0.30f;
__constant float OBSTACLE_BOUNCE_BLEND = 0.30f;
__constant float FAN_BETA = 0.07f;
__constant float FAN_TARGET_SCALE = 0.033333335f;
__constant float FAN_TARGET_MAX = 0.34f;
__constant float FAN_NOISE_AMP = 0.02f;
__constant float FAN_SPEED_SOFT_CAP = 0.30f;
__constant float FAN_SPEED_DAMP_WIDTH = 0.06f;
__constant float STATE_NUDGE = 0.0f;
__constant float MAX_SPEED = 0.34641016f;
__constant float RHO_MIN = 0.97f;
__constant float RHO_MAX = 1.03f;
__constant float CS2 = 0.33333334f;
__constant float P_MIN = -0.03f;
__constant float P_MAX = 0.03f;

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
    int in_ch, int n, int cells, int cell, int x, int y, int z, int q, int opp
);

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

inline float sponge_alpha(int n, int x, int y, int z) {
    if (SPONGE_LAYERS <= 0) return 0.0f;
    int d = min(min(min(x, y), min(z, n - 1 - z)), min(n - 1 - x, n - 1 - y));
    if (d >= SPONGE_LAYERS) return 0.0f;
    float eta = (float)(SPONGE_LAYERS - d) / (float)SPONGE_LAYERS;
    return clampf(SPONGE_STRENGTH * eta * eta, 0.0f, 0.95f);
}

inline float boundary_convective_value(
    __global const float* f_read, __global const float* payload,
    int in_ch, int n, int cells, int cell, int x, int y, int z, int q, int sx, int sy, int sz
) {
    int dx = (sx < 0 || sx >= n) ? CX[q] : 0;
    int dy = (sy < 0 || sy >= n) ? CY[q] : 0;
    int dz = (sz < 0 || sz >= n) ? CZ[q] : 0;

    int i1x = clampi(x + dx, 0, n - 1);
    int i1y = clampi(y + dy, 0, n - 1);
    int i1z = clampi(z + dz, 0, n - 1);
    int src1 = (i1x * n + i1y) * n + i1z;
    if (payload[src1 * in_ch + 0] > 0.5f) {
        return obstacle_bounce_value(f_read, payload, in_ch, n, cells, cell, x, y, z, q, OPP[q]);
    }

    float f1 = f_read[q * cells + src1];
    int i2x = clampi(x + 2 * dx, 0, n - 1);
    int i2y = clampi(y + 2 * dy, 0, n - 1);
    int i2z = clampi(z + 2 * dz, 0, n - 1);
    int src2 = (i2x * n + i2y) * n + i2z;
    if (payload[src2 * in_ch + 0] > 0.5f) return f1;

    float f2 = f_read[q * cells + src2];
    return f1 + BOUNDARY_CONVECTIVE_BETA * (f1 - f2);
}

inline float obstacle_bounce_value(
    __global const float* f_read, __global const float* payload,
    int in_ch, int n, int cells, int cell, int x, int y, int z, int q, int opp
) {
    float bounced = f_read[opp * cells + cell];
    if (OBSTACLE_BOUNCE_BLEND <= 0.0f) return bounced;

    int s2x = x - 2 * CX[q];
    int s2y = y - 2 * CY[q];
    int s2z = z - 2 * CZ[q];
    if (s2x < 0 || s2y < 0 || s2z < 0 || s2x >= n || s2y >= n || s2z >= n) return bounced;

    int src2 = (s2x * n + s2y) * n + s2z;
    if (payload[src2 * in_ch + 0] > 0.5f) return bounced;

    float upstream = f_read[q * cells + src2];
    return bounced + OBSTACLE_BOUNCE_BLEND * (upstream - bounced);
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
    int in_ch, int n, int cells, int tick,
    __global float* f_write
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) return;

    int yz = n * n;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / n;
    int z = rem - y * n;
    
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
        if (sx < 0 || sy < 0 || sz < 0 || sx >= n || sy >= n || sz >= n) {
            f_local[q] = boundary_convective_value(f_read, payload, in_ch, n, cells, cell, x, y, z, q, sx, sy, sz);
        } else {
            int src = (sx * n + sy) * n + sz;
            if (payload[src * in_ch + 0] > 0.5f) {
                f_local[q] = obstacle_bounce_value(f_read, payload, in_ch, n, cells, cell, x, y, z, q, opp);
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
        mx += fq * (float)CX[q]; my += fq * (float)CY[q]; mz += fq * (float)CZ[q];
    }

    float rho_safe = fmax(1e-6f, rho);
    float inv_rho = 1.0f / rho_safe;
    float ux = mx * inv_rho;
    float uy = my * inv_rho;
    float uz = mz * inv_rho;
    float speed_pre = sqrt(ux * ux + uy * uy + uz * uz);

    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
    float fan = clampf(payload[base + 1], 0.0f, 1.0f);
    if (fan > 0.0f) {
        float fan_ux = payload[base + 2], fan_uy = payload[base + 3], fan_uz = payload[base + 4];
        float fan_norm = sqrt(fan_ux * fan_ux + fan_uy * fan_uy + fan_uz * fan_uz);
        if (fan_norm > 1e-8f) {
            float inv_norm = 1.0f / fan_norm;
            float noise = 1.0f + FAN_NOISE_AMP * signed_noise((uint)cell, (uint)tick);
            float target_speed = clampf(
                fan_norm * FAN_TARGET_SCALE * fmax(0.0f, noise),
                0.0f,
                FAN_TARGET_MAX
            );
            float target_ux = fan_ux * inv_norm * target_speed;
            float target_uy = fan_uy * inv_norm * target_speed;
            float target_uz = fan_uz * inv_norm * target_speed;
            float speed_damp = 1.0f;
            if (speed_pre > FAN_SPEED_SOFT_CAP) {
                float r = (speed_pre - FAN_SPEED_SOFT_CAP) / fmax(1e-4f, FAN_SPEED_DAMP_WIDTH);
                speed_damp = 1.0f / (1.0f + r * r);
            }
            float beta = fan * FAN_BETA * speed_damp;
            fx = beta * rho_safe * (target_ux - ux);
            fy = beta * rho_safe * (target_uy - uy);
            fz = beta * rho_safe * (target_uz - uz);
        }
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
    float tau_shear_local = TAU_SHEAR;
    float tau_normal_local = clampf(TAU_NORMAL, TAU_NORMAL_MIN, TAU_NORMAL_MAX);
    if (SGS_ENABLED) {
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
    float alpha_sponge = sponge_alpha(n, x, y, z);
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
    if (ctx.d_output) clReleaseMemObject(ctx.d_output);
    if (ctx.d_f_post) clReleaseMemObject(ctx.d_f_post);
    if (ctx.d_f) clReleaseMemObject(ctx.d_f);
    if (ctx.d_payload) clReleaseMemObject(ctx.d_payload);
    ctx.d_output = ctx.d_f_post = ctx.d_f = ctx.d_payload = nullptr;
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
        clReleaseProgram(program); clReleaseCommandQueue(queue); clReleaseContext(context);
        g_opencl.error = "clBuildProgram failed"; return false;
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

    cl_int err = CL_SUCCESS;
    ctx.d_payload = clCreateBuffer(g_opencl.context, CL_MEM_READ_ONLY, payload_bytes, nullptr, &err);
    ctx.d_f = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, dist_bytes, nullptr, &err);
    ctx.d_f_post = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, dist_bytes, nullptr, &err);
    ctx.d_output = clCreateBuffer(g_opencl.context, CL_MEM_WRITE_ONLY, output_bytes, nullptr, &err);

    if (!ctx.d_payload || !ctx.d_f || !ctx.d_f_post || !ctx.d_output) {
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
        ctx.gpu_initialized = true;
    }

    const int tick_i32 = static_cast<int>(ctx.step_counter & 0x7FFFFFFF);
    
    // Ping-Pong 双重缓冲交换
    cl_mem read_buf = (ctx.step_counter % 2 == 0) ? ctx.d_f : ctx.d_f_post;
    cl_mem write_buf = (ctx.step_counter % 2 == 0) ? ctx.d_f_post : ctx.d_f;

    err |= clSetKernelArg(g_opencl.k_stream_collide, 0, sizeof(cl_mem), &read_buf);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 1, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 3, sizeof(int), &ctx.n);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 4, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 5, sizeof(int), &tick_i32);
    err |= clSetKernelArg(g_opencl.k_stream_collide, 6, sizeof(cl_mem), &write_buf);
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
    timing.readback_ms += elapsed_ms(readback_begin, Clock::now());
    
    ctx.step_counter += 1;
    return true;
}

#else
struct OpenClRuntime { bool available = false; std::string error; std::string device_name; };
OpenClRuntime g_opencl;
void release_context_gpu_buffers(ContextState&) {}
void release_opencl_runtime() {}
bool initialize_opencl_runtime() { g_opencl.error = "Disabled"; return false; }
bool opencl_step(ContextState&, const float*, float*, StepTiming&) { return false; }
#endif

void clear_context(ContextState& ctx) { release_context_gpu_buffers(ctx); ctx = ContextState{}; }
void clear_all_contexts() { for (auto& e : g_contexts) clear_context(e.second); g_contexts.clear(); }
void reset_runtime_state() { clear_all_contexts(); release_opencl_runtime(); g_cfg = Config{}; reset_timing_stats(); }

void disable_opencl_runtime(const std::string& reason) {
    for (auto& e : g_contexts) release_context_gpu_buffers(e.second);
    release_opencl_runtime();
    g_cfg.opencl_enabled = false;
    g_cfg.runtime_info = "cpu|cumulant-d3q27+sgs (" + reason + ")";
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

void ensure_context_shape(ContextState& ctx, int n, std::size_t cells) {
    if (ctx.n == n && ctx.cells == cells) return;
    clear_context(ctx); ctx.n = n; ctx.cells = cells;
}

void run_cpu_step(ContextState& ctx, const float* packet, float* out) {
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.n);
    ingest_payload(ctx, packet, g_cfg.input_channels);
    if (!ctx.cpu_initialized) initialize_distributions(ctx);
    collide(ctx); stream_and_bounce(ctx); write_output(ctx, out, g_cfg.output_channels);
    ctx.step_counter += 1;
}

}  // namespace

extern "C" {

static jboolean native_init_impl(jint grid_size, jint input_channels, jint output_channels) {
    if (grid_size <= 0 || input_channels < 9 || output_channels < 4) { reset_runtime_state(); return JNI_FALSE; }
    clear_all_contexts(); reset_timing_stats();
    g_cfg.grid_size = grid_size; g_cfg.input_channels = input_channels; g_cfg.output_channels = output_channels; g_cfg.initialized = true;

    if (should_force_cpu_backend()) {
        g_cfg.opencl_enabled = false;
        g_cfg.runtime_info = "cpu|cumulant-d3q27+sgs (forced)";
        return JNI_TRUE;
    }
    g_cfg.opencl_enabled = initialize_opencl_runtime();
    if (g_cfg.opencl_enabled) {
        g_cfg.runtime_info = "opencl|cumulant-d3q27+sgs:" + g_opencl.device_name;
    } else {
        g_cfg.runtime_info = "cpu|cumulant-d3q27+sgs (" + g_opencl.error + ")";
    }
    return JNI_TRUE;
}

static jboolean native_step_impl(
    JNIEnv* env, jclass, jbyteArray payload, jint grid_size, jlong context_key, jfloatArray output_flow
) {
    auto tick_begin = Clock::now();
    StepTiming timing;

    if (!g_cfg.initialized || !payload || !output_flow || grid_size != g_cfg.grid_size) return JNI_FALSE;
    const std::size_t cells = static_cast<std::size_t>(grid_size) * grid_size * grid_size;
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
    ensure_context_shape(ctx, grid_size, cells);
    // Ensure CPU-side buffers exist and reuse packet buffer to avoid per-step allocations
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) allocate_cpu_context(ctx, ctx.n);
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

    bool ok = false;
    if (g_cfg.opencl_enabled) {
        if (!(ok = opencl_step(ctx, ctx.packet.data(), out, timing))) disable_opencl_runtime("OpenCL fail");
    }
    if (!ok) {
        auto solver_begin = Clock::now();
        run_cpu_step(ctx, ctx.packet.data(), out);
        timing.solver_ms += elapsed_ms(solver_begin, Clock::now());
        ok = true;
    }

    env->ReleaseFloatArrayElements(output_flow, out, 0);
    timing.total_ms = elapsed_ms(tick_begin, Clock::now());
    record_timing(timing);
    return ok ? JNI_TRUE : JNI_FALSE;
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

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeReleaseContext(JNIEnv*, jclass, jlong context_key) {
    native_release_context_impl(context_key);
}
JNIEXPORT void JNICALL Java_com_aerodynamics4mc_runtime_NativeLbmBridge_nativeReleaseContext(JNIEnv*, jclass, jlong context_key) {
    native_release_context_impl(context_key);
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
