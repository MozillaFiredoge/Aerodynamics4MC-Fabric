#pragma once

#include <array>

#include "aero_lbm_hydro_core.h"

namespace aero_lbm::thermal_core {

inline constexpr int kQ = 7;
inline constexpr std::array<int, kQ> kCx = {0, 1, -1, 0, 0, 0, 0};
inline constexpr std::array<int, kQ> kCy = {0, 0, 0, 1, -1, 0, 0};
inline constexpr std::array<int, kQ> kCz = {0, 0, 0, 0, 0, 1, -1};
inline constexpr std::array<int, kQ> kOpp = {0, 2, 1, 4, 3, 6, 5};
inline constexpr std::array<float, kQ> kW = {0.25f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f};
inline constexpr float kCs2 = 0.25f;

struct ThermalTransportCoefficients {
    float alpha_molecular_lattice = 0.0f;
    float tau_thermal_molecular = 0.0f;
};

inline constexpr float thermal_diffusivity_from_nu_pr(float nu, float pr) {
    return nu / pr;
}

inline constexpr float tau_from_alpha(float alpha_lattice) {
    return 0.5f + alpha_lattice / kCs2;
}

inline constexpr float runtime_boussinesq_beta(
    float gravity_m_s2,
    float thermal_expansion_per_k,
    float temperature_scale_k,
    float dt_s,
    float dx_m
) {
    return gravity_m_s2
        * thermal_expansion_per_k
        * temperature_scale_k
        * (dt_s * dt_s / dx_m);
}

ThermalTransportCoefficients derive_thermal_transport(const hydro_core::TransportSpec& spec);
float equilibrium_d3q7(int q, float temperature, float ux, float uy, float uz);

}  // namespace aero_lbm::thermal_core
