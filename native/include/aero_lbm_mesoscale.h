#pragma once

#include "aero_lbm_hydro_core.h"
#include "aero_lbm_thermal_core.h"

namespace aero_lbm::mesoscale {

struct DomainSpec {
    int nx;
    int ny;
    int nz;
    float dx_m;
    float dt_s;
    float molecular_nu_m2_s;
    float prandtl_air;
    float turbulent_prandtl;
};

struct TransportCoefficients {
    float velocity_scale_m_s_per_lattice;
    float nu_molecular_lattice;
    float alpha_molecular_lattice;
    float tau_shear_molecular;
    float tau_thermal_molecular;
};

bool valid(const DomainSpec& spec);
TransportCoefficients derive_transport(const DomainSpec& spec);
float meters_per_second_to_lattice(float velocity_m_s, const DomainSpec& spec);
float lattice_to_meters_per_second(float velocity_lattice, const DomainSpec& spec);

}  // namespace aero_lbm::mesoscale
