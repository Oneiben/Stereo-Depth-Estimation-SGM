#ifndef SGM_HLS_H
#define SGM_HLS_H

#include <hls_math.h>
#include <ap_int.h>

/**
 * @file sgm_hls.h
 * @brief Hardware-oriented constraints and interface definitions for SGM IP Core.
 */

/* --- Hardware Image Geometry --- */
#define HEIGHT 240
#define WIDTH 272
#define MAX_DISP 16

/* --- SGM Energy Minimization Penalties --- */
#define P1_PENALTY 8   // Penalty for small disparity changes (neighbor +/- 1)
#define P2_PENALTY 128 // Penalty for large disparity discontinuities (> 1)

/**
 * @brief Top-level entry point for the Semi-Global Matching (SGM) hardware accelerator.
 * @param left_pixels  Input AXI-Master port for the reference image.
 * @param right_pixels   Input AXI-Master port for the target image.
 * @param disparity_out  Output AXI-Master port for the calculated disparity map.
 */
void sgm_hls(
    float left_pixels[HEIGHT * WIDTH],
    float right_pixels[HEIGHT * WIDTH],
    int disparity_out[HEIGHT * WIDTH]);

#endif
