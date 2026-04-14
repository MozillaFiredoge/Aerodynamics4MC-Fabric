#include "aero_lbm_analysis_codec.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "zfp.h"

namespace aero_lbm_analysis_codec {
namespace {

bool valid_shape(int nx, int ny, int nz)
{
    return nx > 0 && ny > 0 && nz > 0;
}

}

bool compress_float_grid_3d(
    const float* values,
    int nx,
    int ny,
    int nz,
    double tolerance,
    std::vector<std::uint8_t>& out_bytes,
    std::string& out_error
)
{
    out_bytes.clear();
    out_error.clear();
    if (!values || !valid_shape(nx, ny, nz)) {
        out_error = "invalid_input";
        return false;
    }
    if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
        out_error = "invalid_tolerance";
        return false;
    }

    zfp_field* field = zfp_field_3d(const_cast<float*>(values), zfp_type_float, static_cast<size_t>(nx), static_cast<size_t>(ny), static_cast<size_t>(nz));
    if (!field) {
        out_error = "zfp_field_3d_failed";
        return false;
    }

    zfp_stream* stream = zfp_stream_open(nullptr);
    if (!stream) {
        zfp_field_free(field);
        out_error = "zfp_stream_open_failed";
        return false;
    }
    zfp_stream_set_accuracy(stream, tolerance);

    const size_t max_size = zfp_stream_maximum_size(stream, field);
    if (max_size == 0) {
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "zfp_maximum_size_failed";
        return false;
    }

    out_bytes.assign(max_size, 0);
    bitstream* bit_stream = stream_open(out_bytes.data(), out_bytes.size());
    if (!bit_stream) {
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "bitstream_open_failed";
        return false;
    }

    zfp_stream_set_bit_stream(stream, bit_stream);
    zfp_stream_rewind(stream);
    const size_t header_bits = zfp_write_header(stream, field, ZFP_HEADER_FULL);
    if (header_bits == 0) {
        stream_close(bit_stream);
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "zfp_write_header_failed";
        return false;
    }

    const size_t compressed_bytes = zfp_compress(stream, field);
    const size_t flushed_bytes = zfp_stream_flush(stream);
    const size_t used_bytes = std::max(compressed_bytes, flushed_bytes);
    if (used_bytes == 0 || used_bytes > out_bytes.size()) {
        stream_close(bit_stream);
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "zfp_compress_failed";
        return false;
    }

    out_bytes.resize(used_bytes);
    stream_close(bit_stream);
    zfp_stream_close(stream);
    zfp_field_free(field);
    return true;
}

bool decompress_float_grid_3d(
    const std::uint8_t* compressed,
    std::size_t compressed_size,
    int nx,
    int ny,
    int nz,
    float* out_values,
    std::string& out_error
)
{
    out_error.clear();
    if (!compressed || compressed_size == 0 || !out_values || !valid_shape(nx, ny, nz)) {
        out_error = "invalid_input";
        return false;
    }

    zfp_field* field = zfp_field_3d(out_values, zfp_type_float, static_cast<size_t>(nx), static_cast<size_t>(ny), static_cast<size_t>(nz));
    if (!field) {
        out_error = "zfp_field_3d_failed";
        return false;
    }

    zfp_stream* stream = zfp_stream_open(nullptr);
    if (!stream) {
        zfp_field_free(field);
        out_error = "zfp_stream_open_failed";
        return false;
    }

    bitstream* bit_stream = stream_open(const_cast<std::uint8_t*>(compressed), compressed_size);
    if (!bit_stream) {
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "bitstream_open_failed";
        return false;
    }

    zfp_stream_set_bit_stream(stream, bit_stream);
    zfp_stream_rewind(stream);
    const size_t header_bits = zfp_read_header(stream, field, ZFP_HEADER_FULL);
    if (header_bits == 0) {
        stream_close(bit_stream);
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "zfp_read_header_failed";
        return false;
    }

    if (field->type != zfp_type_float
        || field->nx != static_cast<size_t>(nx)
        || field->ny != static_cast<size_t>(ny)
        || field->nz != static_cast<size_t>(nz)) {
        stream_close(bit_stream);
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "zfp_header_mismatch";
        return false;
    }

    const size_t used_bytes = zfp_decompress(stream, field);
    if (used_bytes == 0) {
        stream_close(bit_stream);
        zfp_stream_close(stream);
        zfp_field_free(field);
        out_error = "zfp_decompress_failed";
        return false;
    }

    stream_close(bit_stream);
    zfp_stream_close(stream);
    zfp_field_free(field);
    return true;
}

}
