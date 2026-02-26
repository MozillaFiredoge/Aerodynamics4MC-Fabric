#include <jni.h>

#include <algorithm>
#include <array>
#include <chrono>
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

constexpr int kQ = 19;
constexpr std::array<int, kQ> kCx = {0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0};
constexpr std::array<int, kQ> kCy = {0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1};
constexpr std::array<int, kQ> kCz = {0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1};
constexpr std::array<int, kQ> kOpp = {0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17};
constexpr std::array<float, kQ> kW = {
    1.0f / 3.0f,
    1.0f / 18.0f, 1.0f / 18.0f, 1.0f / 18.0f, 1.0f / 18.0f, 1.0f / 18.0f, 1.0f / 18.0f,
    1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f,
    1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f, 1.0f / 36.0f
};

constexpr int kChannelObstacle = 0;
constexpr int kChannelFanMask = 1;
constexpr int kChannelFanVx = 2;
constexpr int kChannelFanVy = 3;
constexpr int kChannelFanVz = 4;
constexpr int kChannelStateVx = 5;
constexpr int kChannelStateVy = 6;
constexpr int kChannelStateVz = 7;
constexpr int kChannelStateP = 8;

constexpr bool kEnableSmagorinskyLES = true;
constexpr float kLatticeSoundSpeed = 0.57735026919f;   // 1 / sqrt(3)
constexpr float kMaxMach = 0.35f;                      // hard velocity clamp inside solver
constexpr float kHardMaxLatticeSpeed = kLatticeSoundSpeed * kMaxMach;

constexpr float kRhoMin = 0.97f;
constexpr float kRhoMax = 1.03f;
constexpr float kPressureMin = -0.03f;
constexpr float kPressureMax = 0.03f;

constexpr float kTauPlus = 0.50003f;
constexpr float kTauPlusMin = 0.50003f;
constexpr float kTauPlusMax = 0.80f;
constexpr float kSmagorinskyC = 0.10f;
constexpr float kSmagorinskyC2 = kSmagorinskyC * kSmagorinskyC;
constexpr float kSmagorinskyFactor = 3.0f * kSmagorinskyC2;
constexpr float kLesSpeedThreshold = 0.0f;
constexpr float kTrtMagic = 0.25f;
constexpr float kTauMinus = 0.5f + kTrtMagic / (kTauPlus - 0.5f);
constexpr float kOmegaPlus = 1.0f / kTauPlus;
constexpr float kOmegaMinus = 1.0f / kTauMinus;
constexpr float kFanAccel = 0.0120f;
constexpr float kFanForceScalePerSpeed = 0.5f;
constexpr float kFanForceScaleMax = 4.0f;
constexpr float kFanPulseAmp = 0.05f;
constexpr float kFanPulseFreq = 0.12f;
constexpr float kStateNudge = 0.0f;
constexpr float kMaxSpeed = kHardMaxLatticeSpeed;
constexpr float kFneqRelax = 0.92f;

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

void reset_timing_stats() {
    g_timing = TimingStats{};
}

void record_timing(const StepTiming& timing) {
    g_timing.ticks += 1;
    g_timing.payload_copy_ms_sum += timing.payload_copy_ms;
    g_timing.solver_ms_sum += timing.solver_ms;
    g_timing.readback_ms_sum += timing.readback_ms;
    g_timing.total_ms_sum += timing.total_ms;
    g_timing.last = timing;
}

std::string timing_info_string() {
    if (g_timing.ticks == 0) {
        return "ticks=0";
    }

    const double inv = 1.0 / static_cast<double>(g_timing.ticks);
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(3)
        << "ticks=" << g_timing.ticks
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
    return (static_cast<std::size_t>(x) * static_cast<std::size_t>(n) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(n) + static_cast<std::size_t>(z);
}

inline std::size_t dist_index(std::size_t cell, int q) {
    return cell * static_cast<std::size_t>(kQ) + static_cast<std::size_t>(q);
}

inline float clampf(float v, float lo, float hi) {
    return std::min(hi, std::max(lo, v));
}

inline float feq(int q, float rho, float ux, float uy, float uz) {
    const float cu = 3.0f * (kCx[q] * ux + kCy[q] * uy + kCz[q] * uz);
    const float uu = ux * ux + uy * uy + uz * uz;
    return kW[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

void allocate_cpu_context(ContextState& ctx, int n) {
    ctx.n = n;
    ctx.cells = static_cast<std::size_t>(n) * static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    ctx.cpu_initialized = false;

    ctx.f.assign(ctx.cells * static_cast<std::size_t>(kQ), 0.0f);
    ctx.f_post.assign(ctx.cells * static_cast<std::size_t>(kQ), 0.0f);
    ctx.rho.assign(ctx.cells, 1.0f);
    ctx.ux.assign(ctx.cells, 0.0f);
    ctx.uy.assign(ctx.cells, 0.0f);
    ctx.uz.assign(ctx.cells, 0.0f);

    ctx.ref_ux.assign(ctx.cells, 0.0f);
    ctx.ref_uy.assign(ctx.cells, 0.0f);
    ctx.ref_uz.assign(ctx.cells, 0.0f);
    ctx.ref_pressure.assign(ctx.cells, 0.0f);

    ctx.fan_mask.assign(ctx.cells, 0.0f);
    ctx.fan_ux.assign(ctx.cells, 0.0f);
    ctx.fan_uy.assign(ctx.cells, 0.0f);
    ctx.fan_uz.assign(ctx.cells, 0.0f);
    ctx.obstacle.assign(ctx.cells, 0);
}

void ingest_payload(ContextState& ctx, const float* payload, int in_channels) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t base = cell * static_cast<std::size_t>(in_channels);
        ctx.obstacle[cell] = payload[base + kChannelObstacle] > 0.5f ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0);
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
            ctx.ux[cell] = 0.0f;
            ctx.uy[cell] = 0.0f;
            ctx.uz[cell] = 0.0f;
        } else {
            ctx.rho[cell] = clampf(1.0f + ctx.ref_pressure[cell], kRhoMin, kRhoMax);
            ctx.ux[cell] = ctx.ref_ux[cell];
            ctx.uy[cell] = ctx.ref_uy[cell];
            ctx.uz[cell] = ctx.ref_uz[cell];
        }

        const float rho = ctx.rho[cell];
        const float ux = ctx.ux[cell];
        const float uy = ctx.uy[cell];
        const float uz = ctx.uz[cell];
        for (int q = 0; q < kQ; ++q) {
            ctx.f[dist_index(cell, q)] = feq(q, rho, ux, uy, uz);
        }
    }
    ctx.cpu_initialized = true;
}

void collide(ContextState& ctx) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        if (ctx.obstacle[cell]) {
            ctx.rho[cell] = 1.0f;
            ctx.ux[cell] = 0.0f;
            ctx.uy[cell] = 0.0f;
            ctx.uz[cell] = 0.0f;
            for (int q = 0; q < kQ; ++q) {
                ctx.f_post[dist_index(cell, q)] = ctx.f[dist_index(cell, q)];
            }
            continue;
        }

        float rho = 0.0f;
        float ux = 0.0f;
        float uy = 0.0f;
        float uz = 0.0f;
        for (int q = 0; q < kQ; ++q) {
            const float fq = ctx.f[dist_index(cell, q)];
            rho += fq;
            ux += fq * static_cast<float>(kCx[q]);
            uy += fq * static_cast<float>(kCy[q]);
            uz += fq * static_cast<float>(kCz[q]);
        }
        rho = clampf(rho, kRhoMin, kRhoMax);
        ux /= rho;
        uy /= rho;
        uz /= rho;

        float fx = 0.0f;
        float fy = 0.0f;
        float fz = 0.0f;
        const float fan = ctx.fan_mask[cell];
        if (fan > 0.0f) {
            const float fan_ux = ctx.fan_ux[cell];
            const float fan_uy = ctx.fan_uy[cell];
            const float fan_uz = ctx.fan_uz[cell];
            const float fan_norm = std::sqrt(fan_ux * fan_ux + fan_uy * fan_uy + fan_uz * fan_uz);
            if (fan_norm > 1e-8f) {
                const float inv_norm = 1.0f / fan_norm;
                const float base_force_scale = std::min(kFanForceScaleMax, fan_norm * kFanForceScalePerSpeed);
                const float phase = kFanPulseFreq * static_cast<float>(ctx.step_counter)
                    + 0.037f * static_cast<float>(cell);
                const float pulse = 1.0f + kFanPulseAmp * std::sin(phase);
                const float force_scale = std::max(0.0f, base_force_scale * pulse);
                fx = fan * kFanAccel * force_scale * fan_ux * inv_norm;
                fy = fan * kFanAccel * force_scale * fan_uy * inv_norm;
                fz = fan * kFanAccel * force_scale * fan_uz * inv_norm;
            }
        }

        // Body-force injection (Guo-style momentum shift) is more stable than hard velocity pinning.
        ux += fx / rho * 0.5;
        uy += fy / rho * 0.5;
        uz += fz / rho * 0.5;

        ux = (1.0f - kStateNudge) * ux + kStateNudge * ctx.ref_ux[cell];
        uy = (1.0f - kStateNudge) * uy + kStateNudge * ctx.ref_uy[cell];
        uz = (1.0f - kStateNudge) * uz + kStateNudge * ctx.ref_uz[cell];

        const float speed2 = ux * ux + uy * uy + uz * uz;
        if (speed2 > kMaxSpeed * kMaxSpeed) {
            const float scale = kMaxSpeed / std::sqrt(speed2);
            ux *= scale;
            uy *= scale;
            uz *= scale;
        }

        ctx.rho[cell] = rho;
        ctx.ux[cell] = ux;
        ctx.uy[cell] = uy;
        ctx.uz[cell] = uz;

        std::array<float, kQ> feq_cache;
        float qxx = 0.0f;
        float qyy = 0.0f;
        float qzz = 0.0f;
        float qxy = 0.0f;
        float qxz = 0.0f;
        float qyz = 0.0f;
        const bool apply_les = kEnableSmagorinskyLES && speed2 > (kLesSpeedThreshold * kLesSpeedThreshold);

        for (int q = 0; q < kQ; ++q) {
            const float feq_q = feq(q, rho, ux, uy, uz);
            feq_cache[static_cast<std::size_t>(q)] = feq_q;

            if (apply_les) {
                const float fneq = ctx.f[dist_index(cell, q)] - feq_q;
                const float cx = static_cast<float>(kCx[q]);
                const float cy = static_cast<float>(kCy[q]);
                const float cz = static_cast<float>(kCz[q]);
                qxx += cx * cx * fneq;
                qyy += cy * cy * fneq;
                qzz += cz * cz * fneq;
                qxy += cx * cy * fneq;
                qxz += cx * cz * fneq;
                qyz += cy * cz * fneq;
            }
        }

        float tau_plus_local = kTauPlus;
        if (apply_les) {
            const float q_norm2 = qxx * qxx + qyy * qyy + qzz * qzz
                + 2.0f * (qxy * qxy + qxz * qxz + qyz * qyz);
            const float q_mag = std::sqrt(std::max(0.0f, q_norm2));
            const float s_mag = (3.0f / (2.0f * std::max(1e-6f, rho) * kTauPlus)) * q_mag;
            tau_plus_local = clampf(kTauPlus + kSmagorinskyFactor * s_mag, kTauPlusMin, kTauPlusMax);
        }
        float tau_minus_local = 0.5f + kTrtMagic / std::max(1e-6f, (tau_plus_local - 0.5f));
        tau_minus_local = fmin(tau_minus_local, 1.0f);
        const float omega_plus_local = 1.0f / tau_plus_local;
        const float omega_minus_local = 1.0f / tau_minus_local;

        for (int q = 0; q < kQ; ++q) {
            const std::size_t di = dist_index(cell, q);
            const int opp = kOpp[q];
            const std::size_t di_opp = dist_index(cell, opp);
            const float f_q = ctx.f[di];
            const float f_opp = ctx.f[di_opp];
            const float f_eq_q = feq_cache[static_cast<std::size_t>(q)];
            const float f_eq_opp = feq_cache[static_cast<std::size_t>(opp)];

            const float f_plus = 0.5f * (f_q + f_opp);
            const float f_minus = 0.5f * (f_q - f_opp);
            const float feq_plus = 0.5f * (f_eq_q + f_eq_opp);
            const float feq_minus = 0.5f * (f_eq_q - f_eq_opp);

            const float ci_dot_u = static_cast<float>(kCx[q]) * ux + static_cast<float>(kCy[q]) * uy + static_cast<float>(kCz[q]) * uz;
            const float ci_dot_f = static_cast<float>(kCx[q]) * fx + static_cast<float>(kCy[q]) * fy + static_cast<float>(kCz[q]) * fz;
            const float ciminu_dot_f = (static_cast<float>(kCx[q]) - ux) * fx
                + (static_cast<float>(kCy[q]) - uy) * fy
                + (static_cast<float>(kCz[q]) - uz) * fz;
            const float source = kW[q] * (1.0f - 0.5f * omega_plus_local) * 3.0f * (ciminu_dot_f + 3.0f * ci_dot_u * ci_dot_f);

            const float f_collided = f_q
                - omega_plus_local * (f_plus - feq_plus)
                - omega_minus_local * (f_minus - feq_minus)
                + source;
            // Damp high-frequency checkerboard modes (odd-even decoupling) while keeping macroscopic moments.
            ctx.f_post[di] = f_eq_q + kFneqRelax * (f_collided - f_eq_q);
        }
    }
}

void stream_and_bounce(ContextState& ctx) {
    const int n = ctx.n;
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < n; ++y) {
            for (int z = 0; z < n; ++z) {
                const std::size_t cell = cell_index(x, y, z, n);
                if (ctx.obstacle[cell]) {
                    for (int q = 0; q < kQ; ++q) {
                        ctx.f[dist_index(cell, q)] = ctx.f_post[dist_index(cell, kOpp[q])];
                    }
                    continue;
                }

                for (int q = 0; q < kQ; ++q) {
                    const int sx = x - kCx[q];
                    const int sy = y - kCy[q];
                    const int sz = z - kCz[q];

                    if (sx < 0 || sy < 0 || sz < 0 || sx >= n || sy >= n || sz >= n) {
                        // Zero-gradient domain boundary using mirrored interior ghost sampling.
                        const int cx = std::min(n - 1, std::max(0, (sx < 0 || sx >= n) ? (x + kCx[q]) : sx));
                        const int cy = std::min(n - 1, std::max(0, (sy < 0 || sy >= n) ? (y + kCy[q]) : sy));
                        const int cz = std::min(n - 1, std::max(0, (sz < 0 || sz >= n) ? (z + kCz[q]) : sz));
                        const std::size_t src = cell_index(cx, cy, cz, n);
                        if (ctx.obstacle[src]) {
                            ctx.f[dist_index(cell, q)] = ctx.f_post[dist_index(cell, q)];
                        } else {
                            ctx.f[dist_index(cell, q)] = ctx.f_post[dist_index(src, q)];
                        }
                        continue;
                    }

                    const std::size_t src = cell_index(sx, sy, sz, n);
                    if (ctx.obstacle[src]) {
                        ctx.f[dist_index(cell, q)] = ctx.f_post[dist_index(cell, kOpp[q])];
                    } else {
                        ctx.f[dist_index(cell, q)] = ctx.f_post[dist_index(src, q)];
                    }
                }
            }
        }
    }
}

void write_output(const ContextState& ctx, float* out, int out_channels) {
    for (std::size_t cell = 0; cell < ctx.cells; ++cell) {
        const std::size_t out_base = cell * static_cast<std::size_t>(out_channels);
        if (ctx.obstacle[cell]) {
            for (int c = 0; c < out_channels; ++c) {
                out[out_base + static_cast<std::size_t>(c)] = 0.0f;
            }
            continue;
        }

        float rho = 0.0f;
        float ux = 0.0f;
        float uy = 0.0f;
        float uz = 0.0f;
        for (int q = 0; q < kQ; ++q) {
            const float fq = ctx.f[dist_index(cell, q)];
            rho += fq;
            ux += fq * static_cast<float>(kCx[q]);
            uy += fq * static_cast<float>(kCy[q]);
            uz += fq * static_cast<float>(kCz[q]);
        }
        rho = clampf(rho, kRhoMin, kRhoMax);
        ux /= rho;
        uy /= rho;
        uz /= rho;

        out[out_base + 0] = ux;
        out[out_base + 1] = uy;
        out[out_base + 2] = uz;
        out[out_base + 3] = rho - 1.0f;
        for (int c = 4; c < out_channels; ++c) {
            out[out_base + static_cast<std::size_t>(c)] = 0.0f;
        }
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
    cl_kernel k_collide = nullptr;
    cl_kernel k_stream = nullptr;
    cl_kernel k_output = nullptr;
};

OpenClRuntime g_opencl;

const char* kOpenClSource = R"CLC(
#define KQ 19

__constant int CX[KQ] = {0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0};
__constant int CY[KQ] = {0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1};
__constant int CZ[KQ] = {0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1};
__constant int OPP[KQ] = {0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17};
__constant float W[KQ] = {
    0.33333334f,
    0.055555556f, 0.055555556f, 0.055555556f, 0.055555556f, 0.055555556f, 0.055555556f,
    0.027777778f, 0.027777778f, 0.027777778f, 0.027777778f, 0.027777778f, 0.027777778f,
    0.027777778f, 0.027777778f, 0.027777778f, 0.027777778f, 0.027777778f, 0.027777778f
};

__constant int LES_ENABLED = 1;
__constant float TAU_PLUS_BASE = 0.50003f;
__constant float TAU_PLUS_MIN = 0.50003f;
__constant float TAU_PLUS_MAX = 0.8f;
__constant float TRT_MAGIC = 0.25f;
__constant float SMAG_FACTOR = 0.03f;
__constant float LES_SPEED_THRESHOLD = 0.0f;
__constant float FAN_ACCEL = 0.0120f;
__constant float FAN_FORCE_SCALE_PER_SPEED = 0.5f;
__constant float FAN_FORCE_SCALE_MAX = 4.0f;
__constant float FAN_PULSE_AMP = 0.05f;
__constant float FAN_PULSE_FREQ = 0.12f;
__constant float STATE_NUDGE = 0.0f;
__constant float MAX_SPEED = 0.20207259f;
__constant float RHO_MIN = 0.97f;
__constant float RHO_MAX = 1.03f;
__constant float P_MIN = -0.03f;
__constant float P_MAX = 0.03f;
__constant float FNEQ_RELAX = 0.92f;

inline float clampf(float v, float lo, float hi) {
    return fmin(hi, fmax(lo, v));
}

inline int clampi(int v, int lo, int hi) {
    return min(hi, max(lo, v));
}

inline int is_obstacle(__global const float* payload, int cell, int in_ch) {
    return payload[cell * in_ch + 0] > 0.5f;
}

inline float feq(int q, float rho, float ux, float uy, float uz) {
    float cu = 3.0f * (CX[q] * ux + CY[q] * uy + CZ[q] * uz);
    float uu = ux * ux + uy * uy + uz * uz;
    return W[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

kernel void init_distributions(
    __global const float* payload,
    int in_ch,
    int cells,
    __global float* f
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) {
        return;
    }

    int base = cell * in_ch;
    int solid = payload[base + 0] > 0.5f;
    float rho = 1.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;

    if (!solid) {
        float p = clampf(payload[base + 8], P_MIN, P_MAX);
        rho = clampf(1.0f + p, RHO_MIN, RHO_MAX);
        ux = payload[base + 5];
        uy = payload[base + 6];
        uz = payload[base + 7];
    }

    int fq_base = cell * KQ;
    for (int q = 0; q < KQ; ++q) {
        f[fq_base + q] = feq(q, rho, ux, uy, uz);
    }
}

kernel void collide_step(
    __global const float* f,
    __global const float* payload,
    int in_ch,
    int cells,
    int tick,
    __global float* f_post
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) {
        return;
    }

    int fq_base = cell * KQ;
    int base = cell * in_ch;
    if (payload[base + 0] > 0.5f) {
        for (int q = 0; q < KQ; ++q) {
            f_post[fq_base + q] = f[fq_base + q];
        }
        return;
    }

    float rho = 0.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;

    for (int q = 0; q < KQ; ++q) {
        float fq = f[fq_base + q];
        rho += fq;
        ux += fq * (float)CX[q];
        uy += fq * (float)CY[q];
        uz += fq * (float)CZ[q];
    }

    rho = clampf(rho, RHO_MIN, RHO_MAX);
    ux /= rho;
    uy /= rho;
    uz /= rho;

    float fx = 0.0f;
    float fy = 0.0f;
    float fz = 0.0f;
    float fan = clampf(payload[base + 1], 0.0f, 1.0f);
    if (fan > 0.0f) {
        float fan_ux = payload[base + 2];
        float fan_uy = payload[base + 3];
        float fan_uz = payload[base + 4];
        float fan_norm = sqrt(fan_ux * fan_ux + fan_uy * fan_uy + fan_uz * fan_uz);
        if (fan_norm > 1e-8f) {
            float inv_norm = 1.0f / fan_norm;
            float base_force_scale = fmin(FAN_FORCE_SCALE_MAX, fan_norm * FAN_FORCE_SCALE_PER_SPEED);
            float phase = FAN_PULSE_FREQ * (float)tick + 0.037f * (float)cell;
            float pulse = 1.0f + FAN_PULSE_AMP * sin(phase);
            float force_scale = fmax(0.0f, base_force_scale * pulse);
            fx = fan * FAN_ACCEL * force_scale * fan_ux * inv_norm;
            fy = fan * FAN_ACCEL * force_scale * fan_uy * inv_norm;
            fz = fan * FAN_ACCEL * force_scale * fan_uz * inv_norm;
        }
    }

    ux += fx / rho * 0.5;
    uy += fy / rho * 0.5;
    uz += fz / rho * 0.5;

    float ref_ux = payload[base + 5];
    float ref_uy = payload[base + 6];
    float ref_uz = payload[base + 7];
    ux = (1.0f - STATE_NUDGE) * ux + STATE_NUDGE * ref_ux;
    uy = (1.0f - STATE_NUDGE) * uy + STATE_NUDGE * ref_uy;
    uz = (1.0f - STATE_NUDGE) * uz + STATE_NUDGE * ref_uz;

    float speed2 = ux * ux + uy * uy + uz * uz;
    float max_speed2 = MAX_SPEED * MAX_SPEED;
    if (speed2 > max_speed2) {
        float scale = MAX_SPEED / sqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
    }

    float feq_cache[KQ];
    float qxx = 0.0f;
    float qyy = 0.0f;
    float qzz = 0.0f;
    float qxy = 0.0f;
    float qxz = 0.0f;
    float qyz = 0.0f;
    int apply_les = LES_ENABLED && speed2 > (LES_SPEED_THRESHOLD * LES_SPEED_THRESHOLD);

    for (int q = 0; q < KQ; ++q) {
        float f_eq_q = feq(q, rho, ux, uy, uz);
        feq_cache[q] = f_eq_q;

        if (apply_les) {
            float fneq = f[fq_base + q] - f_eq_q;
            float cx = (float)CX[q];
            float cy = (float)CY[q];
            float cz = (float)CZ[q];
            qxx += cx * cx * fneq;
            qyy += cy * cy * fneq;
            qzz += cz * cz * fneq;
            qxy += cx * cy * fneq;
            qxz += cx * cz * fneq;
            qyz += cy * cz * fneq;
        }
    }

    float tau_plus_local = TAU_PLUS_BASE;
    if (apply_les) {
        float q_norm2 = qxx * qxx + qyy * qyy + qzz * qzz
            + 2.0f * (qxy * qxy + qxz * qxz + qyz * qyz);
        float q_mag = sqrt(fmax(0.0f, q_norm2));
        float s_mag = (3.0f / (2.0f * fmax(1e-6f, rho) * TAU_PLUS_BASE)) * q_mag;
        tau_plus_local = clampf(TAU_PLUS_BASE + SMAG_FACTOR * s_mag, TAU_PLUS_MIN, TAU_PLUS_MAX);
    }
    float tau_minus_local = 0.5f + TRT_MAGIC / fmax(1e-6f, (tau_plus_local - 0.5f));
    tau_minus_local = fmin(tau_minus_local, 1.0f);
    float omega_plus_local = 1.0f / tau_plus_local;
    float omega_minus_local = 1.0f / tau_minus_local;

    for (int q = 0; q < KQ; ++q) {
        int opp = OPP[q];
        float f_q = f[fq_base + q];
        float f_opp = f[fq_base + opp];
        float f_eq_q = feq_cache[q];
        float f_eq_opp = feq_cache[opp];

        float f_plus = 0.5f * (f_q + f_opp);
        float f_minus = 0.5f * (f_q - f_opp);
        float feq_plus = 0.5f * (f_eq_q + f_eq_opp);
        float feq_minus = 0.5f * (f_eq_q - f_eq_opp);

        float ci_dot_u = (float)CX[q] * ux + (float)CY[q] * uy + (float)CZ[q] * uz;
        float ci_dot_f = (float)CX[q] * fx + (float)CY[q] * fy + (float)CZ[q] * fz;
        float ciminu_dot_f = ((float)CX[q] - ux) * fx
            + ((float)CY[q] - uy) * fy
            + ((float)CZ[q] - uz) * fz;
        float source = W[q] * (1.0f - 0.5f * omega_plus_local) * 3.0f * (ciminu_dot_f + 3.0f * ci_dot_u * ci_dot_f);

        float f_collided = f_q
            - omega_plus_local * (f_plus - feq_plus)
            - omega_minus_local * (f_minus - feq_minus)
            + source;
        f_post[fq_base + q] = f_eq_q + FNEQ_RELAX * (f_collided - f_eq_q);
    }
}

kernel void stream_step(
    __global const float* f_post,
    __global const float* payload,
    int in_ch,
    int n,
    int cells,
    __global float* f
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) {
        return;
    }

    int yz = n * n;
    int x = cell / yz;
    int rem = cell - x * yz;
    int y = rem / n;
    int z = rem - y * n;
    int fq_base = cell * KQ;

    if (is_obstacle(payload, cell, in_ch)) {
        for (int q = 0; q < KQ; ++q) {
            f[fq_base + q] = f_post[fq_base + OPP[q]];
        }
        return;
    }

    for (int q = 0; q < KQ; ++q) {
        int sx = x - CX[q];
        int sy = y - CY[q];
        int sz = z - CZ[q];

        if (sx < 0 || sy < 0 || sz < 0 || sx >= n || sy >= n || sz >= n) {
            // Zero-gradient domain boundary using mirrored interior ghost sampling.
            int cx = (sx < 0 || sx >= n) ? (x + CX[q]) : sx;
            int cy = (sy < 0 || sy >= n) ? (y + CY[q]) : sy;
            int cz = (sz < 0 || sz >= n) ? (z + CZ[q]) : sz;
            cx = clampi(cx, 0, n - 1);
            cy = clampi(cy, 0, n - 1);
            cz = clampi(cz, 0, n - 1);
            int src = (cx * n + cy) * n + cz;
            if (is_obstacle(payload, src, in_ch)) {
                f[fq_base + q] = f_post[fq_base + q];
            } else {
                int src_base = src * KQ;
                f[fq_base + q] = f_post[src_base + q];
            }
            continue;
        }

        int src = (sx * n + sy) * n + sz;
        if (is_obstacle(payload, src, in_ch)) {
            f[fq_base + q] = f_post[fq_base + OPP[q]];
        } else {
            int src_base = src * KQ;
            f[fq_base + q] = f_post[src_base + q];
        }
    }
}

kernel void output_macro(
    __global const float* f,
    __global const float* payload,
    int in_ch,
    int out_ch,
    int cells,
    __global float* out
) {
    int cell = (int)get_global_id(0);
    if (cell >= cells) {
        return;
    }

    int out_base = cell * out_ch;
    if (is_obstacle(payload, cell, in_ch)) {
        for (int c = 0; c < out_ch; ++c) {
            out[out_base + c] = 0.0f;
        }
        return;
    }

    int fq_base = cell * KQ;
    float rho = 0.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f[fq_base + q];
        rho += fq;
        ux += fq * (float)CX[q];
        uy += fq * (float)CY[q];
        uz += fq * (float)CZ[q];
    }

    rho = clampf(rho, RHO_MIN, RHO_MAX);
    ux /= rho;
    uy /= rho;
    uz /= rho;

    out[out_base + 0] = ux;
    out[out_base + 1] = uy;
    out[out_base + 2] = uz;
    out[out_base + 3] = rho - 1.0f;
    for (int c = 4; c < out_ch; ++c) {
        out[out_base + c] = 0.0f;
    }
}
)CLC";

const char* cl_error_to_string(cl_int err) {
    switch (err) {
        case CL_SUCCESS: return "CL_SUCCESS";
        case CL_DEVICE_NOT_FOUND: return "CL_DEVICE_NOT_FOUND";
        case CL_DEVICE_NOT_AVAILABLE: return "CL_DEVICE_NOT_AVAILABLE";
        case CL_COMPILER_NOT_AVAILABLE: return "CL_COMPILER_NOT_AVAILABLE";
        case CL_MEM_OBJECT_ALLOCATION_FAILURE: return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
        case CL_OUT_OF_RESOURCES: return "CL_OUT_OF_RESOURCES";
        case CL_OUT_OF_HOST_MEMORY: return "CL_OUT_OF_HOST_MEMORY";
        case CL_BUILD_PROGRAM_FAILURE: return "CL_BUILD_PROGRAM_FAILURE";
        case CL_INVALID_VALUE: return "CL_INVALID_VALUE";
        case CL_INVALID_DEVICE: return "CL_INVALID_DEVICE";
        case CL_INVALID_BINARY: return "CL_INVALID_BINARY";
        case CL_INVALID_BUILD_OPTIONS: return "CL_INVALID_BUILD_OPTIONS";
        case CL_INVALID_PROGRAM: return "CL_INVALID_PROGRAM";
        case CL_INVALID_KERNEL_NAME: return "CL_INVALID_KERNEL_NAME";
        case CL_INVALID_KERNEL_DEFINITION: return "CL_INVALID_KERNEL_DEFINITION";
        case CL_INVALID_KERNEL: return "CL_INVALID_KERNEL";
        case CL_INVALID_MEM_OBJECT: return "CL_INVALID_MEM_OBJECT";
        case CL_INVALID_OPERATION: return "CL_INVALID_OPERATION";
        case CL_INVALID_COMMAND_QUEUE: return "CL_INVALID_COMMAND_QUEUE";
        case CL_INVALID_CONTEXT: return "CL_INVALID_CONTEXT";
        default: return "CL_UNKNOWN_ERROR";
    }
}

std::string read_device_name(cl_device_id device) {
    if (device == nullptr) {
        return "unknown";
    }
    size_t bytes = 0;
    if (clGetDeviceInfo(device, CL_DEVICE_NAME, 0, nullptr, &bytes) != CL_SUCCESS || bytes == 0) {
        return "unknown";
    }
    std::string name(bytes, '\0');
    if (clGetDeviceInfo(device, CL_DEVICE_NAME, bytes, name.data(), nullptr) != CL_SUCCESS) {
        return "unknown";
    }
    if (!name.empty() && name.back() == '\0') {
        name.pop_back();
    }
    return name;
}

void release_opencl_runtime() {
    if (g_opencl.k_output != nullptr) {
        clReleaseKernel(g_opencl.k_output);
        g_opencl.k_output = nullptr;
    }
    if (g_opencl.k_stream != nullptr) {
        clReleaseKernel(g_opencl.k_stream);
        g_opencl.k_stream = nullptr;
    }
    if (g_opencl.k_collide != nullptr) {
        clReleaseKernel(g_opencl.k_collide);
        g_opencl.k_collide = nullptr;
    }
    if (g_opencl.k_init != nullptr) {
        clReleaseKernel(g_opencl.k_init);
        g_opencl.k_init = nullptr;
    }
    if (g_opencl.program != nullptr) {
        clReleaseProgram(g_opencl.program);
        g_opencl.program = nullptr;
    }
    if (g_opencl.queue != nullptr) {
        clReleaseCommandQueue(g_opencl.queue);
        g_opencl.queue = nullptr;
    }
    if (g_opencl.context != nullptr) {
        clReleaseContext(g_opencl.context);
        g_opencl.context = nullptr;
    }
    g_opencl.platform = nullptr;
    g_opencl.device = nullptr;
    g_opencl.available = false;
    g_opencl.device_name.clear();
}

void release_context_gpu_buffers(ContextState& ctx) {
    if (ctx.d_output != nullptr) {
        clReleaseMemObject(ctx.d_output);
        ctx.d_output = nullptr;
    }
    if (ctx.d_f_post != nullptr) {
        clReleaseMemObject(ctx.d_f_post);
        ctx.d_f_post = nullptr;
    }
    if (ctx.d_f != nullptr) {
        clReleaseMemObject(ctx.d_f);
        ctx.d_f = nullptr;
    }
    if (ctx.d_payload != nullptr) {
        clReleaseMemObject(ctx.d_payload);
        ctx.d_payload = nullptr;
    }
    ctx.gpu_buffers_ready = false;
    ctx.gpu_initialized = false;
}

bool initialize_opencl_runtime() {
    if (g_opencl.available) {
        return true;
    }

    cl_uint platform_count = 0;
    cl_int err = clGetPlatformIDs(0, nullptr, &platform_count);
    if (err != CL_SUCCESS || platform_count == 0) {
        g_opencl.error = "No OpenCL platform";
        return false;
    }

    std::vector<cl_platform_id> platforms(platform_count);
    err = clGetPlatformIDs(platform_count, platforms.data(), nullptr);
    if (err != CL_SUCCESS) {
        g_opencl.error = std::string("clGetPlatformIDs failed: ") + cl_error_to_string(err);
        return false;
    }

    cl_device_id selected_device = nullptr;
    cl_platform_id selected_platform = nullptr;

    for (cl_platform_id platform : platforms) {
        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &selected_device, nullptr);
        if (err == CL_SUCCESS) {
            selected_platform = platform;
            break;
        }
    }

    if (selected_device == nullptr) {
        for (cl_platform_id platform : platforms) {
            err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_DEFAULT, 1, &selected_device, nullptr);
            if (err == CL_SUCCESS) {
                selected_platform = platform;
                break;
            }
        }
    }

    if (selected_device == nullptr || selected_platform == nullptr) {
        g_opencl.error = "No usable OpenCL device";
        return false;
    }

    cl_context context = clCreateContext(nullptr, 1, &selected_device, nullptr, nullptr, &err);
    if (err != CL_SUCCESS || context == nullptr) {
        g_opencl.error = std::string("clCreateContext failed: ") + cl_error_to_string(err);
        return false;
    }

    cl_command_queue queue = clCreateCommandQueue(context, selected_device, 0, &err);
    if (err != CL_SUCCESS || queue == nullptr) {
        clReleaseContext(context);
        g_opencl.error = std::string("clCreateCommandQueue failed: ") + cl_error_to_string(err);
        return false;
    }

    const char* src = kOpenClSource;
    const size_t src_len = std::strlen(kOpenClSource);
    cl_program program = clCreateProgramWithSource(context, 1, &src, &src_len, &err);
    if (err != CL_SUCCESS || program == nullptr) {
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        g_opencl.error = std::string("clCreateProgramWithSource failed: ") + cl_error_to_string(err);
        return false;
    }

    err = clBuildProgram(program, 1, &selected_device, "-cl-fast-relaxed-math", nullptr, nullptr);
    if (err != CL_SUCCESS) {
        size_t log_size = 0;
        std::string build_log;
        if (clGetProgramBuildInfo(program, selected_device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size) == CL_SUCCESS && log_size > 0) {
            build_log.resize(log_size);
            if (clGetProgramBuildInfo(program, selected_device, CL_PROGRAM_BUILD_LOG, log_size, build_log.data(), nullptr) == CL_SUCCESS) {
                if (!build_log.empty() && build_log.back() == '\0') {
                    build_log.pop_back();
                }
            } else {
                build_log.clear();
            }
        }
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        g_opencl.error = std::string("clBuildProgram failed: ") + cl_error_to_string(err) + (build_log.empty() ? "" : (" | " + build_log));
        return false;
    }

    cl_kernel k_init = clCreateKernel(program, "init_distributions", &err);
    if (err != CL_SUCCESS || k_init == nullptr) {
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        g_opencl.error = std::string("clCreateKernel(init_distributions) failed: ") + cl_error_to_string(err);
        return false;
    }

    cl_kernel k_collide = clCreateKernel(program, "collide_step", &err);
    if (err != CL_SUCCESS || k_collide == nullptr) {
        clReleaseKernel(k_init);
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        g_opencl.error = std::string("clCreateKernel(collide_step) failed: ") + cl_error_to_string(err);
        return false;
    }

    cl_kernel k_stream = clCreateKernel(program, "stream_step", &err);
    if (err != CL_SUCCESS || k_stream == nullptr) {
        clReleaseKernel(k_collide);
        clReleaseKernel(k_init);
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        g_opencl.error = std::string("clCreateKernel(stream_step) failed: ") + cl_error_to_string(err);
        return false;
    }

    cl_kernel k_output = clCreateKernel(program, "output_macro", &err);
    if (err != CL_SUCCESS || k_output == nullptr) {
        clReleaseKernel(k_stream);
        clReleaseKernel(k_collide);
        clReleaseKernel(k_init);
        clReleaseProgram(program);
        clReleaseCommandQueue(queue);
        clReleaseContext(context);
        g_opencl.error = std::string("clCreateKernel(output_macro) failed: ") + cl_error_to_string(err);
        return false;
    }

    g_opencl.context = context;
    g_opencl.queue = queue;
    g_opencl.program = program;
    g_opencl.k_init = k_init;
    g_opencl.k_collide = k_collide;
    g_opencl.k_stream = k_stream;
    g_opencl.k_output = k_output;
    g_opencl.platform = selected_platform;
    g_opencl.device = selected_device;
    g_opencl.available = true;
    g_opencl.device_name = read_device_name(selected_device);
    g_opencl.error.clear();
    return true;
}

bool ensure_context_gpu_buffers(ContextState& ctx) {
    if (!g_opencl.available || ctx.cells == 0) {
        return false;
    }
    if (ctx.gpu_buffers_ready) {
        return true;
    }

    const std::size_t payload_bytes = ctx.cells * static_cast<std::size_t>(g_cfg.input_channels) * sizeof(float);
    const std::size_t dist_bytes = ctx.cells * static_cast<std::size_t>(kQ) * sizeof(float);
    const std::size_t output_bytes = ctx.cells * static_cast<std::size_t>(g_cfg.output_channels) * sizeof(float);

    cl_int err = CL_SUCCESS;
    ctx.d_payload = clCreateBuffer(g_opencl.context, CL_MEM_READ_ONLY, payload_bytes, nullptr, &err);
    if (err != CL_SUCCESS || ctx.d_payload == nullptr) {
        release_context_gpu_buffers(ctx);
        return false;
    }

    ctx.d_f = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, dist_bytes, nullptr, &err);
    if (err != CL_SUCCESS || ctx.d_f == nullptr) {
        release_context_gpu_buffers(ctx);
        return false;
    }

    ctx.d_f_post = clCreateBuffer(g_opencl.context, CL_MEM_READ_WRITE, dist_bytes, nullptr, &err);
    if (err != CL_SUCCESS || ctx.d_f_post == nullptr) {
        release_context_gpu_buffers(ctx);
        return false;
    }

    ctx.d_output = clCreateBuffer(g_opencl.context, CL_MEM_WRITE_ONLY, output_bytes, nullptr, &err);
    if (err != CL_SUCCESS || ctx.d_output == nullptr) {
        release_context_gpu_buffers(ctx);
        return false;
    }

    ctx.gpu_buffers_ready = true;
    ctx.gpu_initialized = false;
    return true;
}

bool enqueue_kernel_1d(cl_kernel kernel, int cells) {
    const size_t global_size = static_cast<size_t>(cells);
    cl_int err = clEnqueueNDRangeKernel(g_opencl.queue, kernel, 1, nullptr, &global_size, nullptr, 0, nullptr, nullptr);
    return err == CL_SUCCESS;
}

bool opencl_step(ContextState& ctx, const float* payload, float* out, StepTiming& timing) {
    if (!g_opencl.available) {
        return false;
    }

    if (!ensure_context_gpu_buffers(ctx)) {
        return false;
    }

    const int cells_i32 = static_cast<int>(ctx.cells);
    const std::size_t payload_bytes = ctx.cells * static_cast<std::size_t>(g_cfg.input_channels) * sizeof(float);
    const std::size_t output_bytes = ctx.cells * static_cast<std::size_t>(g_cfg.output_channels) * sizeof(float);

    auto upload_begin = Clock::now();
    cl_int err = clEnqueueWriteBuffer(g_opencl.queue, ctx.d_payload, CL_TRUE, 0, payload_bytes, payload, 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        return false;
    }
    timing.payload_copy_ms += elapsed_ms(upload_begin, Clock::now());

    auto solver_begin = Clock::now();
    if (!ctx.gpu_initialized) {
        err = clSetKernelArg(g_opencl.k_init, 0, sizeof(cl_mem), &ctx.d_payload);
        err |= clSetKernelArg(g_opencl.k_init, 1, sizeof(int), &g_cfg.input_channels);
        err |= clSetKernelArg(g_opencl.k_init, 2, sizeof(int), &cells_i32);
        err |= clSetKernelArg(g_opencl.k_init, 3, sizeof(cl_mem), &ctx.d_f);
        if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_init, cells_i32)) {
            return false;
        }
        ctx.gpu_initialized = true;
    }

    const int tick_i32 = static_cast<int>(ctx.step_counter & 0x7FFFFFFF);

    err = clSetKernelArg(g_opencl.k_collide, 0, sizeof(cl_mem), &ctx.d_f);
    err |= clSetKernelArg(g_opencl.k_collide, 1, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_collide, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_collide, 3, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_collide, 4, sizeof(int), &tick_i32);
    err |= clSetKernelArg(g_opencl.k_collide, 5, sizeof(cl_mem), &ctx.d_f_post);
    if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_collide, cells_i32)) {
        return false;
    }

    err = clSetKernelArg(g_opencl.k_stream, 0, sizeof(cl_mem), &ctx.d_f_post);
    err |= clSetKernelArg(g_opencl.k_stream, 1, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_stream, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_stream, 3, sizeof(int), &ctx.n);
    err |= clSetKernelArg(g_opencl.k_stream, 4, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_stream, 5, sizeof(cl_mem), &ctx.d_f);
    if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_stream, cells_i32)) {
        return false;
    }

    err = clSetKernelArg(g_opencl.k_output, 0, sizeof(cl_mem), &ctx.d_f);
    err |= clSetKernelArg(g_opencl.k_output, 1, sizeof(cl_mem), &ctx.d_payload);
    err |= clSetKernelArg(g_opencl.k_output, 2, sizeof(int), &g_cfg.input_channels);
    err |= clSetKernelArg(g_opencl.k_output, 3, sizeof(int), &g_cfg.output_channels);
    err |= clSetKernelArg(g_opencl.k_output, 4, sizeof(int), &cells_i32);
    err |= clSetKernelArg(g_opencl.k_output, 5, sizeof(cl_mem), &ctx.d_output);
    if (err != CL_SUCCESS || !enqueue_kernel_1d(g_opencl.k_output, cells_i32)) {
        return false;
    }
    err = clFinish(g_opencl.queue);
    if (err != CL_SUCCESS) {
        return false;
    }
    timing.solver_ms += elapsed_ms(solver_begin, Clock::now());

    auto readback_begin = Clock::now();
    err = clEnqueueReadBuffer(g_opencl.queue, ctx.d_output, CL_TRUE, 0, output_bytes, out, 0, nullptr, nullptr);
    if (err != CL_SUCCESS) {
        return false;
    }
    timing.readback_ms += elapsed_ms(readback_begin, Clock::now());
    ctx.step_counter += 1;
    return true;
}

#else

struct OpenClRuntime {
    bool available = false;
    std::string error;
    std::string device_name;
};

OpenClRuntime g_opencl;

void release_context_gpu_buffers(ContextState&) {
}

void release_opencl_runtime() {
}

bool initialize_opencl_runtime() {
    g_opencl.error = "OpenCL disabled at build time";
    return false;
}

bool opencl_step(ContextState&, const float*, float*, StepTiming&) {
    return false;
}

#endif

void clear_context(ContextState& ctx) {
    release_context_gpu_buffers(ctx);
    ctx = ContextState{};
}

void clear_all_contexts() {
    for (auto& entry : g_contexts) {
        clear_context(entry.second);
    }
    g_contexts.clear();
}

void reset_runtime_state() {
    clear_all_contexts();
    release_opencl_runtime();
    g_cfg = Config{};
    reset_timing_stats();
}

void disable_opencl_runtime(const std::string& reason) {
    for (auto& entry : g_contexts) {
        release_context_gpu_buffers(entry.second);
    }
    release_opencl_runtime();
    g_cfg.opencl_enabled = false;
    g_cfg.runtime_info = kEnableSmagorinskyLES
        ? ("cpu|trt+les_fallback (" + reason + ")")
        : ("cpu|trt_fallback (" + reason + ")");
}

bool should_force_cpu_backend() {
    const char* env = std::getenv("AERO_LBM_CPU_ONLY");
    if (env == nullptr) {
        return false;
    }
    return std::strcmp(env, "1") == 0 || std::strcmp(env, "true") == 0 || std::strcmp(env, "TRUE") == 0;
}

void ensure_context_shape(ContextState& ctx, int n, std::size_t cells) {
    if (ctx.n == n && ctx.cells == cells) {
        return;
    }
    clear_context(ctx);
    ctx.n = n;
    ctx.cells = cells;
}

void run_cpu_step(ContextState& ctx, const float* packet, float* out) {
    if (ctx.f.empty() || ctx.f_post.empty() || ctx.cells == 0) {
        allocate_cpu_context(ctx, ctx.n);
    }

    ingest_payload(ctx, packet, g_cfg.input_channels);
    if (!ctx.cpu_initialized) {
        initialize_distributions(ctx);
    }
    collide(ctx);
    stream_and_bounce(ctx);
    write_output(ctx, out, g_cfg.output_channels);
    ctx.step_counter += 1;
}

}  // namespace

extern "C" {

JNIEXPORT jboolean JNICALL
Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeInit(
    JNIEnv*,
    jclass,
    jint grid_size,
    jint input_channels,
    jint output_channels
) {
    if (grid_size <= 0 || input_channels < 9 || output_channels < 4) {
        reset_runtime_state();
        return JNI_FALSE;
    }

    clear_all_contexts();
    reset_timing_stats();

    g_cfg.grid_size = static_cast<int>(grid_size);
    g_cfg.input_channels = static_cast<int>(input_channels);
    g_cfg.output_channels = static_cast<int>(output_channels);
    g_cfg.initialized = true;

    if (should_force_cpu_backend()) {
        g_cfg.opencl_enabled = false;
        g_cfg.runtime_info = kEnableSmagorinskyLES
            ? "cpu|trt+les (forced by AERO_LBM_CPU_ONLY)"
            : "cpu|trt (forced by AERO_LBM_CPU_ONLY)";
        return JNI_TRUE;
    }

    g_cfg.opencl_enabled = initialize_opencl_runtime();
    if (g_cfg.opencl_enabled) {
        g_cfg.runtime_info = kEnableSmagorinskyLES
            ? ("opencl|trt+les:" + g_opencl.device_name)
            : ("opencl|trt:" + g_opencl.device_name);
    } else {
        g_cfg.runtime_info = kEnableSmagorinskyLES
            ? ("cpu|trt+les (" + g_opencl.error + ")")
            : ("cpu|trt (" + g_opencl.error + ")");
    }
    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL
Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeStep(
    JNIEnv* env,
    jclass,
    jbyteArray payload,
    jint grid_size,
    jlong context_key,
    jfloatArray output_flow
) {
    const auto tick_begin = Clock::now();
    StepTiming timing;

    if (!g_cfg.initialized || payload == nullptr || output_flow == nullptr) {
        return JNI_FALSE;
    }

    const int n = static_cast<int>(grid_size);
    if (n <= 0 || n != g_cfg.grid_size) {
        return JNI_FALSE;
    }

    const std::size_t cells = static_cast<std::size_t>(n) * static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    const std::size_t payload_float_count = cells * static_cast<std::size_t>(g_cfg.input_channels);
    const std::size_t payload_bytes_expected = payload_float_count * sizeof(float);
    const jsize payload_len = env->GetArrayLength(payload);
    if (payload_len != static_cast<jsize>(payload_bytes_expected)) {
        return JNI_FALSE;
    }

    const std::size_t output_float_count = cells * static_cast<std::size_t>(g_cfg.output_channels);
    const jsize output_len = env->GetArrayLength(output_flow);
    if (output_len < static_cast<jsize>(output_float_count)) {
        return JNI_FALSE;
    }

    const auto copy_begin = Clock::now();
    jboolean payload_copy = JNI_FALSE;
    jbyte* payload_bytes = env->GetByteArrayElements(payload, &payload_copy);
    if (payload_bytes == nullptr) {
        return JNI_FALSE;
    }
    std::vector<float> packet(payload_float_count, 0.0f);
    std::memcpy(packet.data(), payload_bytes, payload_bytes_expected);
    env->ReleaseByteArrayElements(payload, payload_bytes, JNI_ABORT);
    timing.payload_copy_ms += elapsed_ms(copy_begin, Clock::now());

    jboolean out_copy = JNI_FALSE;
    jfloat* out = env->GetFloatArrayElements(output_flow, &out_copy);
    if (out == nullptr) {
        return JNI_FALSE;
    }

    ContextState& ctx = g_contexts[context_key];
    ensure_context_shape(ctx, n, cells);

    bool ok = false;
    if (g_cfg.opencl_enabled) {
        ok = opencl_step(ctx, packet.data(), out, timing);
        if (!ok) {
            disable_opencl_runtime("OpenCL step failed");
        }
    }

    if (!ok) {
        const auto solver_begin = Clock::now();
        run_cpu_step(ctx, packet.data(), out);
        timing.solver_ms += elapsed_ms(solver_begin, Clock::now());
        g_cfg.runtime_info = g_cfg.runtime_info.empty()
            ? (kEnableSmagorinskyLES ? "cpu|trt+les" : "cpu|trt")
            : g_cfg.runtime_info;
        ok = true;
    }

    env->ReleaseFloatArrayElements(output_flow, out, 0);
    timing.total_ms = elapsed_ms(tick_begin, Clock::now());
    record_timing(timing);
    return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeReleaseContext(
    JNIEnv*,
    jclass,
    jlong context_key
) {
    auto it = g_contexts.find(context_key);
    if (it == g_contexts.end()) {
        return;
    }
    clear_context(it->second);
    g_contexts.erase(it);
}

JNIEXPORT void JNICALL
Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeShutdown(
    JNIEnv*,
    jclass
) {
    reset_runtime_state();
}

JNIEXPORT jstring JNICALL
Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeRuntimeInfo(
    JNIEnv* env,
    jclass
) {
    const std::string info = g_cfg.runtime_info.empty() ? "uninitialized" : g_cfg.runtime_info;
    return env->NewStringUTF(info.c_str());
}

JNIEXPORT jstring JNICALL
Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeTimingInfo(
    JNIEnv* env,
    jclass
) {
    const std::string info = timing_info_string();
    return env->NewStringUTF(info.c_str());
}

}  // extern "C"
