#pragma once

#include <array>

namespace aero_lbm::hydro_core {

inline constexpr int kQ = 27;
inline constexpr std::array<int, 3> kVel1D = {-1, 0, 1};
inline constexpr std::array<float, 3> kW1D = {1.0f / 6.0f, 2.0f / 3.0f, 1.0f / 6.0f};

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

inline constexpr std::array<int, kQ> kCx = make_cx();
inline constexpr std::array<int, kQ> kCy = make_cy();
inline constexpr std::array<int, kQ> kCz = make_cz();
inline constexpr std::array<int, kQ> kOpp = make_opp();
inline constexpr std::array<float, kQ> kW = make_w();

struct TransportSpec {
    int nx;
    int ny;
    int nz;
    float dx_m;
    float dt_s;
    float molecular_nu_m2_s;
    float prandtl_air;
    float turbulent_prandtl;
};

struct HydroTransportCoefficients {
    float velocity_scale_m_s_per_lattice = 0.0f;
    float nu_molecular_lattice = 0.0f;
    float tau_shear_molecular = 0.0f;
};

inline constexpr float physical_diffusivity_to_lattice(float physical_diffusivity_m2_s, float dx_m, float dt_s) {
    return physical_diffusivity_m2_s * dt_s / (dx_m * dx_m);
}

inline constexpr float tau_from_lattice_diffusivity(float lattice_diffusivity) {
    return 0.5f + 3.0f * lattice_diffusivity;
}

bool valid_transport_spec(const TransportSpec& spec);
HydroTransportCoefficients derive_hydro_transport(const TransportSpec& spec);
float meters_per_second_to_lattice(float velocity_m_s, const TransportSpec& spec);
float lattice_to_meters_per_second(float velocity_lattice, const TransportSpec& spec);
float equilibrium_d3q27(int q, float rho, float ux, float uy, float uz);

}  // namespace aero_lbm::hydro_core
