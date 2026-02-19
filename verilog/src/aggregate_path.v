`timescale 1ns / 1ps

/**
 * @module aggregate_path_v
 * @brief Combinational logic for SGM 1D path cost aggregation.
 *
 * This module implements the recursive energy minimization formula:
 * L_r(p, d) = C(p, d) + min [ L_r(p-r, d), L_r(p-r, d-1)+P1, L_r(p-r, d+1)+P1, min_k(L_r(p-r, k))+P2 ] - min_k(L_r(p-r, k))
 *
 * It processes all disparity levels for a single pixel in parallel to meet 
 * high-throughput requirements of real-time stereo vision.
 */
module aggregate_path_v #(
    parameter MAX_DISP   = 16,  // Total number of disparity levels
    parameter P1_PENALTY = 8,   // Penalty for small disparity changes (+/- 1)
    parameter P2_PENALTY = 128  // Penalty for large disparity changes (> 1)
)(
    input  wire [(MAX_DISP*16)-1:0] matching_cost_flat,  // Flattened input cost volume C(p, d)
    input  wire [(MAX_DISP*16)-1:0] prev_path_cost_flat, // Flattened previous aggregated cost L_r(p-r, d)
    input  wire [15:0]              min_prev_path_cost,  // Pre-calculated min_k(L_r(p-r, k)) for normalization
    input  wire                     is_path_start,       // Control signal to reset aggregation at frame boundaries
    output wire [(MAX_DISP*16)-1:0] next_path_cost_flat, // Flattened current aggregated cost L_r(p, d)
    output reg  [15:0]              min_next_path_cost   // Computed min_k(L_r(p, k)) for the next iteration
);

    integer i;
    reg [15:0] prev_path_cost [0:MAX_DISP-1];
    reg [15:0] next_path_cost [0:MAX_DISP-1];
    reg [15:0] current_ad_cost [0:MAX_DISP-1];
    reg [15:0] min_transition_cost;

    always @(*) begin
        // Initialize minimum cost search for the current pixel
        min_next_path_cost = 16'hFFFF;

        // Unpack flattened input vectors into indexed arrays for parallel processing
        for (i = 0; i < MAX_DISP; i = i + 1) begin
            prev_path_cost[i]  = prev_path_cost_flat[i*16 +: 16];
            current_ad_cost[i] = matching_cost_flat[i*16 +: 16];
        end

        // Calculate aggregated path cost for each disparity level
        for (i = 0; i < MAX_DISP; i = i + 1) begin
            if (is_path_start) begin
                // At the start of a path, the aggregated cost is simply the matching cost
                next_path_cost[i] = current_ad_cost[i];
            end else begin
                // Identify the minimum transition cost from the previous pixel
                // Case 0: Disparity remains constant
                min_transition_cost = prev_path_cost[i];

                // Case 1 & 2: Disparity changes by +/- 1 (Smoothness penalty P1)
                if (i > 0 && (prev_path_cost[i-1] + P1_PENALTY) < min_transition_cost) 
                    min_transition_cost = prev_path_cost[i-1] + P1_PENALTY;
                
                if (i < MAX_DISP-1 && (prev_path_cost[i+1] + P1_PENALTY) < min_transition_cost) 
                    min_transition_cost = prev_path_cost[i+1] + P1_PENALTY;

                // Case 3: Disparity jump > 1 (Smoothness penalty P2)
                if ((min_prev_path_cost + P2_PENALTY) < min_transition_cost) 
                    min_transition_cost = min_prev_path_cost + P2_PENALTY;
                
                // Final recursive update with min-cost normalization to prevent bit-width overflow
                next_path_cost[i] = current_ad_cost[i] + (min_transition_cost - min_prev_path_cost);
            end

            // Update the minimum path cost tracker for the current pixel
            if (next_path_cost[i] < min_next_path_cost) 
                min_next_path_cost = next_path_cost[i];
        end
    end

    // Pack calculated path costs back into a flattened vector for output
    generate
        genvar j;
        for (j = 0; j < MAX_DISP; j = j + 1) begin : pack_path_output
            assign next_path_cost_flat[j*16 +: 16] = next_path_cost[j];
        end
    endgenerate

endmodule