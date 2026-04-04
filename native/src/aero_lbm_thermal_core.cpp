#include "aero_lbm_thermal_core.h"

namespace aero_lbm::thermal_core {

ThermalTransportCoefficients derive_thermal_transport(const hydro_core::TransportSpec& spec) {
    ThermalTransportCoefficients out{};
    if (!hydro_core::valid_transport_spec(spec)) {
        return out;
    }
    const float alpha_m2_s = thermal_diffusivity_from_nu_pr(spec.molecular_nu_m2_s, spec.prandtl_air);
    out.alpha_molecular_lattice = hydro_core::physical_diffusivity_to_lattice(alpha_m2_s, spec.dx_m, spec.dt_s);
    out.tau_thermal_molecular = tau_from_alpha(out.alpha_molecular_lattice);
    return out;
}

float equilibrium_d3q7(int q, float temperature, float ux, float uy, float uz) {
    const float cu = static_cast<float>(kCx[q]) * ux
                   + static_cast<float>(kCy[q]) * uy
                   + static_cast<float>(kCz[q]) * uz;
    return kW[q] * temperature * (1.0f + cu / kCs2);
}

}  // namespace aero_lbm::thermal_core
