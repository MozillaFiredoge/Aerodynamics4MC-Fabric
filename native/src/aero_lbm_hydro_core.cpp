#include "aero_lbm_hydro_core.h"

#include <cmath>

namespace aero_lbm::hydro_core {

bool valid_transport_spec(const TransportSpec& spec) {
    return spec.nx > 0
        && spec.ny > 0
        && spec.nz > 0
        && std::isfinite(spec.dx_m)
        && std::isfinite(spec.dt_s)
        && std::isfinite(spec.molecular_nu_m2_s)
        && std::isfinite(spec.prandtl_air)
        && std::isfinite(spec.turbulent_prandtl)
        && spec.dx_m > 0.0f
        && spec.dt_s > 0.0f
        && spec.molecular_nu_m2_s > 0.0f
        && spec.prandtl_air > 0.0f
        && spec.turbulent_prandtl > 0.0f;
}

HydroTransportCoefficients derive_hydro_transport(const TransportSpec& spec) {
    HydroTransportCoefficients out{};
    if (!valid_transport_spec(spec)) {
        return out;
    }
    out.velocity_scale_m_s_per_lattice = spec.dx_m / spec.dt_s;
    out.nu_molecular_lattice = physical_diffusivity_to_lattice(spec.molecular_nu_m2_s, spec.dx_m, spec.dt_s);
    out.tau_shear_molecular = tau_from_lattice_diffusivity(out.nu_molecular_lattice);
    return out;
}

float meters_per_second_to_lattice(float velocity_m_s, const TransportSpec& spec) {
    if (!valid_transport_spec(spec)) {
        return 0.0f;
    }
    return velocity_m_s * spec.dt_s / spec.dx_m;
}

float lattice_to_meters_per_second(float velocity_lattice, const TransportSpec& spec) {
    if (!valid_transport_spec(spec)) {
        return 0.0f;
    }
    return velocity_lattice * spec.dx_m / spec.dt_s;
}

float equilibrium_d3q27(int q, float rho, float ux, float uy, float uz) {
    const float cu = 3.0f * (kCx[q] * ux + kCy[q] * uy + kCz[q] * uz);
    const float uu = ux * ux + uy * uy + uz * uz;
    return kW[q] * rho * (1.0f + cu + 0.5f * cu * cu - 1.5f * uu);
}

}  // namespace aero_lbm::hydro_core
