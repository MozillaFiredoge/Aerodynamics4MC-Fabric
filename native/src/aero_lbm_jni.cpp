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
constexpr float kLatticeSoundSpeed = 0.57735026919f;   
constexpr float kMaxMach = 0.35f;                      
constexpr float kHardMaxLatticeSpeed = kLatticeSoundSpeed * kMaxMach;

constexpr float kRhoMin = 0.97f;
constexpr float kRhoMax = 1.03f;
constexpr float kPressureMin = -0.03f;
constexpr float kPressureMax = 0.03f;

// TRT 被彻底移除，替换为基于二阶 Hermite 的 RLBM 正则化模型
constexpr float kTauPlus = 0.515f;
constexpr float kTauPlusMin = 0.515f;
constexpr float kTauPlusMax = 0.80f;
constexpr float kSmagorinskyC = 0.10f;
constexpr float kSmagorinskyC2 = kSmagorinskyC * kSmagorinskyC;
constexpr float kSmagorinskyFactor = 3.0f * kSmagorinskyC2;
// constexpr float kLesSpeedThreshold = 0.0f;
constexpr float kFanAccel = 0.00120f;
constexpr float kFanForceScalePerSpeed = 0.5f;
constexpr float kFanForceScaleMax = 4.0f;
constexpr float kFanPulseAmp = 0.05f;
constexpr float kFanPulseFreq = 0.12f;
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

inline void clamp_velocity(float& ux, float& uy, float& uz) {
    const float speed2 = ux * ux + uy * uy + uz * uz;
    if (speed2 > kMaxSpeed * kMaxSpeed) {
        const float scale = kMaxSpeed / std::sqrt(speed2);
        ux *= scale;
        uy *= scale;
        uz *= scale;
    }
}

void apply_zou_he_pressure_outlet_xmax(const ContextState& ctx, std::size_t cell, std::array<float, kQ>& f_local) {
    const float rho_target = clampf(1.0f + ctx.ref_pressure[cell], kRhoMin, kRhoMax);
    float uy_target = ctx.ref_uy[cell];
    float uz_target = ctx.ref_uz[cell];
    float ux_target = -1.0f + (
        f_local[0] + f_local[3] + f_local[4] + f_local[5] + f_local[6] +
        f_local[15] + f_local[16] + f_local[17] + f_local[18] +
        2.0f * (f_local[1] + f_local[7] + f_local[9] + f_local[11] + f_local[13])
    ) / rho_target;
    clamp_velocity(ux_target, uy_target, uz_target);

    constexpr std::array<int, 5> kUnknownXMinus = {2, 8, 10, 12, 14};
    for (int q : kUnknownXMinus) {
        const int opp = kOpp[q];
        const float eq_q = feq(q, rho_target, ux_target, uy_target, uz_target);
        const float eq_opp = feq(opp, rho_target, ux_target, uy_target, uz_target);
        f_local[q] = eq_q + (f_local[opp] - eq_opp);
    }
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

        float rho = 0.0f, ux = 0.0f, uy = 0.0f, uz = 0.0f;
        for (int q = 0; q < kQ; ++q) {
            const float fq = ctx.f[dist_index(cell, q, ctx.cells)];
            rho += fq;
            ux += fq * kCx[q]; uy += fq * kCy[q]; uz += fq * kCz[q];
        }
        // rho = clampf(rho, kRhoMin, kRhoMax);
        ux /= rho; uy /= rho; uz /= rho;

        float fx = 0.0f;
        float fy = 0.0f;
        float fz = 0.0f;
        if (ctx.fan_mask[cell] > 0.0f) {
            float fan_norm = std::sqrt(ctx.fan_ux[cell] * ctx.fan_ux[cell] + ctx.fan_uy[cell] * ctx.fan_uy[cell] + ctx.fan_uz[cell] * ctx.fan_uz[cell]);
            if (fan_norm > 1e-8f) {
                float force_scale = std::max(0.0f, std::min(kFanForceScaleMax, fan_norm * kFanForceScalePerSpeed) * 
                    (1.0f + kFanPulseAmp * std::sin(kFanPulseFreq * ctx.step_counter + 0.037f * cell)));
                fx = ctx.fan_mask[cell] * kFanAccel * force_scale * ctx.fan_ux[cell] / fan_norm;
                fy = ctx.fan_mask[cell] * kFanAccel * force_scale * ctx.fan_uy[cell] / fan_norm;
                fz = ctx.fan_mask[cell] * kFanAccel * force_scale * ctx.fan_uz[cell] / fan_norm;
            }
        }

        ux += fx / rho * 0.5f; uy += fy / rho * 0.5f; uz += fz / rho * 0.5f;

        ux = (1.0f - kStateNudge) * ux + kStateNudge * ctx.ref_ux[cell];
        uy = (1.0f - kStateNudge) * uy + kStateNudge * ctx.ref_uy[cell];
        uz = (1.0f - kStateNudge) * uz + kStateNudge * ctx.ref_uz[cell];

        float speed2 = ux * ux + uy * uy + uz * uz;
        if (speed2 > kMaxSpeed * kMaxSpeed) {
            float scale = kMaxSpeed / std::sqrt(speed2);
            ux *= scale; uy *= scale; uz *= scale;
        }

        ctx.rho[cell] = rho; ctx.ux[cell] = ux; ctx.uy[cell] = uy; ctx.uz[cell] = uz;

        float qxx = 0.0f, qyy = 0.0f, qzz = 0.0f, qxy = 0.0f, qxz = 0.0f, qyz = 0.0f;
        std::array<float, kQ> f_eq_cache;
        for (int q = 0; q < kQ; ++q) {
            float eq = feq(q, rho, ux, uy, uz);
            f_eq_cache[q] = eq;
            float fneq = ctx.f[dist_index(cell, q, ctx.cells)] - eq;
            float cx = kCx[q], cy = kCy[q], cz = kCz[q];
            // Use Q = cc - cs^2 I (cs^2 = 1/3) so that q_aa is trace-free component
            float Qxx = cx * cx - 0.33333333f;
            float Qyy = cy * cy - 0.33333333f;
            float Qzz = cz * cz - 0.33333333f;
            float Qxy = cx * cy;
            float Qxz = cx * cz;
            float Qyz = cy * cz;
            qxx += Qxx * fneq; qyy += Qyy * fneq; qzz += Qzz * fneq;
            qxy += Qxy * fneq; qxz += Qxz * fneq; qyz += Qyz * fneq;
        }

        float tau_local = kTauPlus;
        if (kEnableSmagorinskyLES) {
            float q_norm2 = qxx * qxx + qyy * qyy + qzz * qzz + 2.0f * (qxy * qxy + qxz * qxz + qyz * qyz);
            float q_mag = std::sqrt(std::max(0.0f, q_norm2));
            float s_mag = (3.0f / (2.0f * std::max(1e-6f, rho) * kTauPlus)) * q_mag;
            tau_local = clampf(kTauPlus + kSmagorinskyFactor * s_mag, kTauPlusMin, kTauPlusMax);
        }
        float omega = 1.0f / tau_local;
        float uf = ux * fx + uy * fy + uz * fz;

        // Regularized LBM (RLBM) + Guo Forcing
        for (int q = 0; q < kQ; ++q) {
            float cx = kCx[q], cy = kCy[q], cz = kCz[q];
            
            // 投影提取纯净的二阶分布，杜绝奇偶震荡
            float Qxx = cx * cx - 0.33333333f;
            float Qyy = cy * cy - 0.33333333f;
            float Qzz = cz * cz - 0.33333333f;
            float Qxy = cx * cy, Qxz = cx * cz, Qyz = cy * cz;
            float f_neq_reg = 4.5f * kW[q] * (Qxx * qxx + Qyy * qyy + Qzz * qzz + 2.0f * (Qxy * qxy + Qxz * qxz + Qyz * qyz));

            float cu = 3.0f * (cx * ux + cy * uy + cz * uz);
            float cf = 3.0f * (cx * fx + cy * fy + cz * fz);
            float Sq = kW[q] * (cf - 3.0f * uf + cf * cu);
            float source = (1.0f - 0.5f * omega) * Sq;

            ctx.f_post[dist_index(cell, q, ctx.cells)] = f_eq_cache[q] + (1.0f - omega) * f_neq_reg + source;
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
                            // Out-of-domain links use bounce-back; x-max outlet gets repaired by Zou/He below.
                            f_local[q] = ctx.f_post[dist_index(cell, kOpp[q], ctx.cells)];
                            continue;
                        }

                        const std::size_t src = cell_index(sx, sy, sz, n);
                        if (ctx.obstacle[src]) {
                            f_local[q] = ctx.f_post[dist_index(cell, kOpp[q], ctx.cells)];
                        } else {
                            f_local[q] = ctx.f_post[dist_index(src, q, ctx.cells)];
                        }
                    }

                    if (x == n - 1 && y > 0 && y < n - 1 && z > 0 && z < n - 1) {
                        apply_zou_he_pressure_outlet_xmax(ctx, cell, f_local);
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
__constant float TAU_PLUS_BASE = 0.515f;
__constant float TAU_PLUS_MIN = 0.515f;
__constant float TAU_PLUS_MAX = 0.8f;
__constant float SMAG_FACTOR = 0.03f;
__constant float FAN_ACCEL = 0.00120f;
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

inline float clampf(float v, float lo, float hi) { return fmin(hi, fmax(lo, v)); }
inline int clampi(int v, int lo, int hi) { return min(hi, max(lo, v)); }
inline float feq(int q, float rho, float ux, float uy, float uz) {
    float cu = 3.0f * (CX[q] * ux + CY[q] * uy + CZ[q] * uz);
    float uu = ux * ux + uy * uy + uz * uz;
    return W[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

inline void clamp_velocity(__private float* ux, __private float* uy, __private float* uz) {
    float speed2 = (*ux) * (*ux) + (*uy) * (*uy) + (*uz) * (*uz);
    if (speed2 > MAX_SPEED * MAX_SPEED) {
        float scale = MAX_SPEED * rsqrt(speed2);
        *ux *= scale;
        *uy *= scale;
        *uz *= scale;
    }
}

inline void apply_zou_he_pressure_outlet_xmax(__private float* f_local, float rho_target, float uy_target, float uz_target) {
    float ux_target = -1.0f + (
        f_local[0] + f_local[3] + f_local[4] + f_local[5] + f_local[6] +
        f_local[15] + f_local[16] + f_local[17] + f_local[18] +
        2.0f * (f_local[1] + f_local[7] + f_local[9] + f_local[11] + f_local[13])
    ) / rho_target;
    clamp_velocity(&ux_target, &uy_target, &uz_target);

    const int unknown_x_minus[5] = {2, 8, 10, 12, 14};
    for (int i = 0; i < 5; ++i) {
        int q = unknown_x_minus[i];
        int opp = OPP[q];
        float eq_q = feq(q, rho_target, ux_target, uy_target, uz_target);
        float eq_opp = feq(opp, rho_target, ux_target, uy_target, uz_target);
        f_local[q] = eq_q + (f_local[opp] - eq_opp);
    }
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
        // SoA 内存布局: 杜绝显存带宽浪费
        f[q * cells + cell] = eq;
        f_post[q * cells + cell] = eq;
    }
}

// 核心优化：Stream 与 Collide 融合为一个 Kernel (Pull Scheme)，节约 50% 全局显存带宽
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

    // 1. STREAMING (Pull)
    for (int q = 0; q < KQ; ++q) {
        int opp = OPP[q];
        if (is_solid) {
            f_local[q] = f_read[opp * cells + cell]; 
            continue;
        }

        int sx = x - CX[q], sy = y - CY[q], sz = z - CZ[q];
        if (sx < 0 || sy < 0 || sz < 0 || sx >= n || sy >= n || sz >= n) {
            // Out-of-domain links use bounce-back; x-max outlet gets repaired by Zou/He below.
            f_local[q] = f_read[opp * cells + cell];
        } else {
            int src = (sx * n + sy) * n + sz;
            if (payload[src * in_ch + 0] > 0.5f) {
                f_local[q] = f_read[opp * cells + cell];
            } else {
                f_local[q] = f_read[q * cells + src];
            }
        }
    }

    if (is_solid) {
        for (int q = 0; q < KQ; ++q) f_write[q * cells + cell] = f_local[q];
        return;
    }

    if (x == n - 1 && y > 0 && y < n - 1 && z > 0 && z < n - 1) {
        float rho_target = clampf(1.0f + clampf(payload[base + 8], P_MIN, P_MAX), RHO_MIN, RHO_MAX);
        apply_zou_he_pressure_outlet_xmax(f_local, rho_target, payload[base + 6], payload[base + 7]);
    }

    // 2. MACROSCOPIC MOMENTS
    float rho = 0.0f, ux = 0.0f, uy = 0.0f, uz = 0.0f;
    for (int q = 0; q < KQ; ++q) {
        float fq = f_local[q];
        rho += fq;
        ux += fq * (float)CX[q]; uy += fq * (float)CY[q]; uz += fq * (float)CZ[q];
    }

    // rho = clampf(rho, RHO_MIN, RHO_MAX);
    float inv_rho = 1.0f / rho;
    ux *= inv_rho; uy *= inv_rho; uz *= inv_rho;

    // 3. FORCING & NUDGING
    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
    float fan = clampf(payload[base + 1], 0.0f, 1.0f);
    if (fan > 0.0f) {
        float fan_ux = payload[base + 2], fan_uy = payload[base + 3], fan_uz = payload[base + 4];
        float fan_norm = sqrt(fan_ux * fan_ux + fan_uy * fan_uy + fan_uz * fan_uz);
        if (fan_norm > 1e-8f) {
            float inv_norm = 1.0f / fan_norm;
            float force_scale = fmax(0.0f, fmin(FAN_FORCE_SCALE_MAX, fan_norm * FAN_FORCE_SCALE_PER_SPEED) * 
                (1.0f + FAN_PULSE_AMP * sin(FAN_PULSE_FREQ * (float)tick + 0.037f * (float)cell)));
            fx = fan * FAN_ACCEL * force_scale * fan_ux * inv_norm;
            fy = fan * FAN_ACCEL * force_scale * fan_uy * inv_norm;
            fz = fan * FAN_ACCEL * force_scale * fan_uz * inv_norm;
        }
    }

    ux += fx * inv_rho * 0.5f; uy += fy * inv_rho * 0.5f; uz += fz * inv_rho * 0.5f;
    ux = mix(ux, payload[base + 5], STATE_NUDGE);
    uy = mix(uy, payload[base + 6], STATE_NUDGE);
    uz = mix(uz, payload[base + 7], STATE_NUDGE);

    // float speed2 = ux * ux + uy * uy + uz * uz;
    // if (speed2 > MAX_SPEED * MAX_SPEED) {
    //     float scale = MAX_SPEED * rsqrt(speed2);
    //     ux *= scale; uy *= scale; uz *= scale;
    // }

    // 4. COLLISION (RLBM + Smagorinsky)
    float qxx = 0.0f, qyy = 0.0f, qzz = 0.0f, qxy = 0.0f, qxz = 0.0f, qyz = 0.0f;
    float f_eq_cache[KQ];

    for (int q = 0; q < KQ; ++q) {
        float eq = feq(q, rho, ux, uy, uz);
        f_eq_cache[q] = eq;
        float fneq = f_local[q] - eq;
        float cx = (float)CX[q], cy = (float)CY[q], cz = (float)CZ[q];
        float Qxx = cx * cx - 0.33333333f;
        float Qyy = cy * cy - 0.33333333f;
        float Qzz = cz * cz - 0.33333333f;
        float Qxy = cx * cy;
        float Qxz = cx * cz;
        float Qyz = cy * cz;
        qxx += Qxx * fneq; qyy += Qyy * fneq; qzz += Qzz * fneq;
        qxy += Qxy * fneq; qxz += Qxz * fneq; qyz += Qyz * fneq;

    }

    float tau_local = TAU_PLUS_BASE;
    if (LES_ENABLED) {
        float q_norm2 = qxx * qxx + qyy * qyy + qzz * qzz + 2.0f * (qxy * qxy + qxz * qxz + qyz * qyz);
        float q_mag = sqrt(fmax(0.0f, q_norm2));
        float s_mag = (3.0f / (2.0f * fmax(1e-6f, rho) * TAU_PLUS_BASE)) * q_mag;
        tau_local = clampf(TAU_PLUS_BASE + SMAG_FACTOR * s_mag, TAU_PLUS_MIN, TAU_PLUS_MAX);
    }
    float omega = 1.0f / tau_local;
    float uf = ux * fx + uy * fy + uz * fz;

    for (int q = 0; q < KQ; ++q) {
        float cx = (float)CX[q], cy = (float)CY[q], cz = (float)CZ[q];
        
        // 正则化重构非平衡态，彻底断绝奇偶震荡
        float Qxx = cx * cx - 0.33333333f;
        float Qyy = cy * cy - 0.33333333f;
        float Qzz = cz * cz - 0.33333333f;
        float Qxy = cx * cy, Qxz = cx * cz, Qyz = cy * cz;
        float f_neq_reg = 4.5f * W[q] * (Qxx * qxx + Qyy * qyy + Qzz * qzz + 2.0f * (Qxy * qxy + Qxz * qxz + Qyz * qyz));

        float cu = 3.0f * (cx * ux + cy * uy + cz * uz);
        float cf = 3.0f * (cx * fx + cy * fy + cz * fz);
        float Sq = W[q] * (cf - 3.0f * uf + cf * cu);
        float source = (1.0f - 0.5f * omega) * Sq;

        f_write[q * cells + cell] = f_eq_cache[q] + (1.0f - omega) * f_neq_reg + source;
    }
}

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

    // rho = clampf(rho, RHO_MIN, RHO_MAX);
    float inv_rho = 1.0f / rho;
    out[out_base + 0] = ux * inv_rho;
    out[out_base + 1] = uy * inv_rho;
    out[out_base + 2] = uz * inv_rho;
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
    g_cfg.runtime_info = kEnableSmagorinskyLES ? ("cpu|rlbm+les (" + reason + ")") : ("cpu|rlbm (" + reason + ")");
}

bool should_force_cpu_backend() {
    const char* env = std::getenv("AERO_LBM_CPU_ONLY");
    return env && (std::strcmp(env, "1") == 0 || std::strcmp(env, "true") == 0 || std::strcmp(env, "TRUE") == 0);
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

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeInit(
    JNIEnv*, jclass, jint grid_size, jint input_channels, jint output_channels
) {
    if (grid_size <= 0 || input_channels < 9 || output_channels < 4) { reset_runtime_state(); return JNI_FALSE; }
    clear_all_contexts(); reset_timing_stats();
    g_cfg.grid_size = grid_size; g_cfg.input_channels = input_channels; g_cfg.output_channels = output_channels; g_cfg.initialized = true;

    if (should_force_cpu_backend()) {
        g_cfg.opencl_enabled = false;
        g_cfg.runtime_info = kEnableSmagorinskyLES ? "cpu|rlbm+les (forced)" : "cpu|rlbm (forced)";
        return JNI_TRUE;
    }
    g_cfg.opencl_enabled = initialize_opencl_runtime();
    if (g_cfg.opencl_enabled) {
        g_cfg.runtime_info = kEnableSmagorinskyLES ? ("opencl|rlbm+les:" + g_opencl.device_name) : ("opencl|rlbm:" + g_opencl.device_name);
    } else {
        g_cfg.runtime_info = kEnableSmagorinskyLES ? ("cpu|rlbm+les (" + g_opencl.error + ")") : ("cpu|rlbm (" + g_opencl.error + ")");
    }
    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeStep(
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

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeReleaseContext(JNIEnv*, jclass, jlong context_key) {
    auto it = g_contexts.find(context_key);
    if (it != g_contexts.end()) { clear_context(it->second); g_contexts.erase(it); }
}

JNIEXPORT void JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeShutdown(JNIEnv*, jclass) { reset_runtime_state(); }
JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeRuntimeInfo(JNIEnv* env, jclass) { return env->NewStringUTF((g_cfg.runtime_info.empty() ? "uninitialized" : g_cfg.runtime_info).c_str()); }
JNIEXPORT jstring JNICALL Java_com_aerodynamics4mc_client_NativeLbmBridge_nativeTimingInfo(JNIEnv* env, jclass) { return env->NewStringUTF(timing_info_string().c_str()); }

}  // extern "C"
