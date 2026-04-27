#ifndef AERO_SOLVER_CAPI_H
#define AERO_SOLVER_CAPI_H

#include <stdint.h>

#include "aero_lbm_capi.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    AERO_SOLVER_ABI_VERSION = 1,
    AERO_SOLVER_FLOW_CHANNELS = 4,
    AERO_SOLVER_STATUS_OK = 1,
    AERO_SOLVER_STATUS_ERROR = 0
};

typedef enum AeroSolverBoundaryMode {
    AERO_SOLVER_BOUNDARY_WIND_TUNNEL = 1,
    AERO_SOLVER_BOUNDARY_WIND_TUNNEL_PRESSURE_OUTLET = 2,
    AERO_SOLVER_BOUNDARY_CLOSED = 3,
    AERO_SOLVER_BOUNDARY_PERIODIC = 4
} AeroSolverBoundaryMode;

typedef struct AeroGridDesc {
    int nx;
    int ny;
    int nz;
    float dx;
    float dt;
} AeroGridDesc;

typedef struct AeroBoundaryDesc {
    int mode;
    float inlet_vx;
    float inlet_vy;
    float inlet_vz;
    float outlet_pressure;
    float density;
    float viscosity;
} AeroBoundaryDesc;

typedef struct AeroStepInput {
    const uint8_t* solid_mask;
    const float* prev_flow;
    AeroGridDesc grid;
    AeroBoundaryDesc boundary;
    int steps;
} AeroStepInput;

typedef struct AeroStepOutput {
    float* flow_out;
    float max_velocity;
    int status;
} AeroStepOutput;

AERO_LBM_CAPI_EXPORT void aero_solver_default_grid(AeroGridDesc* out_grid);
AERO_LBM_CAPI_EXPORT void aero_solver_default_boundary(AeroBoundaryDesc* out_boundary);

AERO_LBM_CAPI_EXPORT int aero_solver_create(
    int nx,
    int ny,
    int nz,
    float dx,
    float dt,
    long long* out_handle
);

AERO_LBM_CAPI_EXPORT int aero_solver_create_with_grid(
    const AeroGridDesc* grid,
    long long* out_handle
);

AERO_LBM_CAPI_EXPORT int aero_solver_set_solid_mask(
    long long handle,
    const uint8_t* solid_mask,
    int cell_count
);

AERO_LBM_CAPI_EXPORT int aero_solver_set_flow_state(
    long long handle,
    const float* flow,
    int value_count
);

AERO_LBM_CAPI_EXPORT int aero_solver_step_wind_tunnel(
    long long handle,
    const AeroBoundaryDesc* boundary,
    int steps,
    float* out_flow,
    int out_value_count
);

AERO_LBM_CAPI_EXPORT int aero_solver_advance_wind_tunnel(
    long long handle,
    const AeroBoundaryDesc* boundary,
    int steps
);

AERO_LBM_CAPI_EXPORT int aero_solver_run_wind_tunnel(
    const AeroStepInput* input,
    AeroStepOutput* output
);

AERO_LBM_CAPI_EXPORT void aero_solver_destroy(long long handle);
AERO_LBM_CAPI_EXPORT const char* aero_solver_last_error(void);
AERO_LBM_CAPI_EXPORT const char* aero_solver_runtime_info(void);

#ifdef __cplusplus
}
#endif

#endif
