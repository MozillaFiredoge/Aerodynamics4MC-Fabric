#ifndef AERO_LBM_ANALYSIS_CODEC_HPP
#define AERO_LBM_ANALYSIS_CODEC_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aero_lbm_analysis_codec {

bool compress_float_grid_3d(
    const float* values,
    int nx,
    int ny,
    int nz,
    double tolerance,
    std::vector<std::uint8_t>& out_bytes,
    std::string& out_error
);

bool decompress_float_grid_3d(
    const std::uint8_t* compressed,
    std::size_t compressed_size,
    int nx,
    int ny,
    int nz,
    float* out_values,
    std::string& out_error
);

}

#endif
