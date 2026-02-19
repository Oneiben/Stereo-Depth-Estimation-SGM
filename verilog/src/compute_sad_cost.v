`timescale 1ns / 1ps

/**
 * @module compute_sad_cost_v
 * @brief Combinational logic to compute the Sum of Absolute Differences (SAD) for a single pixel pair.
 *
 * This module implements the matching cost C(p, d) at the pixel level. 
 * In the context of the SGM pipeline, this represents the absolute intensity 
 * difference between a pixel in the reference image and a shifted pixel in the target image.
 */
module compute_sad_cost_v (
    input  wire [7:0]  left_pixel_val,  // Intensity value from the reference (left) image
    input  wire [7:0]  right_pixel_val, // Intensity value from the target (right) image
    output wire [15:0] matching_cost    // Resulting Absolute Difference (zero-extended to 16-bit)
);

    // Implementation of the Absolute Difference: |left - right|
    // Result is zero-extended to 16 bits to prevent overflow in subsequent path aggregation stages.
    assign matching_cost = (left_pixel_val > right_pixel_val) ? 
                           (16'd0 + (left_pixel_val - right_pixel_val)) : 
                           (16'd0 + (right_pixel_val - left_pixel_val));

endmodule