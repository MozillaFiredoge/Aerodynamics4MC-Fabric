cmake_minimum_required(VERSION 3.20)

set(zfp_source
    src/zfp.c
    src/bitstream.c
    src/encode1f.c
    src/encode1d.c
    src/encode1i.c
    src/encode1l.c
    src/decode1f.c
    src/decode1d.c
    src/decode1i.c
    src/decode1l.c
    src/encode2f.c
    src/encode2d.c
    src/encode2i.c
    src/encode2l.c
    src/decode2f.c
    src/decode2d.c
    src/decode2i.c
    src/decode2l.c
    src/encode3f.c
    src/encode3d.c
    src/encode3i.c
    src/encode3l.c
    src/decode3f.c
    src/decode3d.c
    src/decode3i.c
    src/decode3l.c
    src/encode4f.c
    src/encode4d.c
    src/encode4i.c
    src/encode4l.c
    src/decode4f.c
    src/decode4d.c
    src/decode4i.c
    src/decode4l.c
)

add_library(zfp STATIC ${zfp_source})
add_library(zfp::zfp ALIAS zfp)
set_target_properties(zfp PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(zfp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if (NOT WIN32)
    target_link_libraries(zfp PRIVATE m)
endif()
