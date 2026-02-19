`timescale 1ns / 1ps

/**
 * @module wta_selector_v
 * @brief Winner-Take-All (WTA) Disparity Selection logic.
 *
 * This module performs the final step of the SGM algorithm. It aggregates the costs 
 * from all active paths (L1 through L4) for each disparity level and identifies 
 * the disparity 'd' that results in the minimum total energy.
 *
 * Total Energy S(p, d) = sum(L_r(p, d)) for all paths r.
 */
module wta_selector_v #(
    parameter MAX_DISP = 16  // Maximum search range for disparity
)(
    input  wire [(MAX_DISP*16)-1:0] path1_cost_flat, // Aggregated cost from Path 1
    input  wire [(MAX_DISP*16)-1:0] path2_cost_flat, // Aggregated cost from Path 2
    input  wire [(MAX_DISP*16)-1:0] path3_cost_flat, // Aggregated cost from Path 3
    input  wire [(MAX_DISP*16)-1:0] path4_cost_flat, // Aggregated cost from Path 4
    output reg  [5:0]               best_disparity   // Final selected disparity [0 to MAX_DISP-1]
);

    integer d;
    reg [15:0] min_total_energy;
    reg [15:0] current_disparity_sum;

    

    always @(*) begin
        // Initialize with maximum possible 16-bit value
        min_total_energy = 16'hFFFF;
        best_disparity   = 6'd0;

        // Iterate through all disparity candidates to find the global minimum energy
        for (d = 0; d < MAX_DISP; d = d + 1) begin
            
            // Summation of all path costs for current disparity index 'd'
            // For 1-path or 2-path configurations, inactive path inputs are tied to 0.
            current_disparity_sum = path1_cost_flat[d*16 +: 16] + 
                                    path2_cost_flat[d*16 +: 16] + 
                                    path3_cost_flat[d*16 +: 16] + 
                                    path4_cost_flat[d*16 +: 16];

            // Update the winner if the current sum is lower than the previous minimum
            if (current_disparity_sum < min_total_energy) begin
                min_total_energy = current_disparity_sum;
                best_disparity   = d[5:0];
            end
        end
    end
    
endmodule