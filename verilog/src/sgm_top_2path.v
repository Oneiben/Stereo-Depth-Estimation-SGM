`timescale 1ns / 1ps

/**
 * @module sgm_top_2path_v
 * @brief Mid-range 2-path Semi-Global Matching (SGM) hardware accelerator.
 *
 * This module implements a dual-path aggregation architecture:
 * 1. Horizontal (Left -> Right) - Local register feedback.
 * 2. Vertical (Top -> Bottom) - Line buffer storage.
 *
 * This configuration offers a balance between hardware resource utilization (BRAM) 
 * and disparity map density, significantly reducing streaking artifacts compared 
 * to the 1-path version.
 */
module sgm_top_2path_v #(
    parameter FRAME_WIDTH  = 272,
    parameter FRAME_HEIGHT = 240,
    parameter MAX_DISP     = 16,
    parameter P1_PENALTY   = 8,
    parameter P2_PENALTY   = 128
)(
    input  wire        clk,           // System clock
    input  wire        rst,           // Synchronous reset
    input  wire [7:0]  left_pixel,    // Reference image pixel stream
    input  wire [7:0]  right_pixel,   // Target image pixel stream
    input  wire        pixel_valid,   // Input data valid qualifier
    output reg  [5:0]  disparity_out, // Computed disparity result
    output reg         valid_out      // Output data valid qualifier
);

    // --- Spatial Coordinate Counters ---
    reg [8:0] curr_x, curr_y;
    always @(posedge clk) begin
        if (rst) begin 
            curr_x <= 0; 
            curr_y <= 0; 
        end 
        else if (pixel_valid) begin
            if (curr_x == FRAME_WIDTH-1) begin 
                curr_x <= 0; 
                curr_y <= (curr_y == FRAME_HEIGHT-1) ? 0 : curr_y + 1; 
            end 
            else begin
                curr_x <= curr_x + 1;
            end
        end
    end

    // --- Search Range Row Buffer ---
    // Stores the current scanline of the target image for matching cost calculations
    reg [7:0] right_row_buffer [0:FRAME_WIDTH-1];
    always @(posedge clk) if (pixel_valid) right_row_buffer[curr_x] <= right_pixel;

    // --- Path Cost Memories ---
    // Horizontal path costs (L_r) stored in registers for immediate reuse
    reg [15:0] path_h_cost [0:MAX_DISP-1];
    reg [15:0] min_path_h;

    // Vertical path costs (L_r) require a line buffer to store values from the row above
    reg [15:0] line_buf_v [0:FRAME_WIDTH-1][0:MAX_DISP-1];
    reg [15:0] min_buf_v  [0:FRAME_WIDTH-1];

    // --- Interconnect Wiring ---
    wire [(MAX_DISP*16)-1:0] matching_cost_flat;
    wire [(MAX_DISP*16)-1:0] prev_path_h_flat, prev_path_v_flat;
    wire [(MAX_DISP*16)-1:0] next_path_h_flat, next_path_v_flat;
    wire [15:0]              next_min_h, next_min_v;
    wire [5:0]               selected_disparity;

    // --- Parallel Matching Cost & Buffer Flattening ---
    genvar i;
    generate
        for (i = 0; i < MAX_DISP; i = i + 1) begin : gen_matching_logic
            // Compute SAD for all disparities in the search range
            compute_sad_cost_v sad_core (
                .left_pixel_val(left_pixel),
                .right_pixel_val((curr_x >= i) ? right_row_buffer[curr_x-i] : 8'hFF),
                .matching_cost(matching_cost_flat[i*16 +: 16])
            );
            
            // Map 2D/Internal storage to 1D ports for the aggregator modules
            assign prev_path_h_flat[i*16 +: 16] = path_h_cost[i];
            assign prev_path_v_flat[i*16 +: 16] = line_buf_v[curr_x][i];
        end
    endgenerate

    // --- Path Aggregation Engines ---
    // Path 1: Horizontal (Left -> Right)
    aggregate_path_v #(MAX_DISP, P1_PENALTY, P2_PENALTY) path_h_engine (
        .matching_cost_flat(matching_cost_flat), .prev_path_cost_flat(prev_path_h_flat), 
        .min_prev_path_cost(min_path_h), .is_path_start(curr_x == 0), 
        .next_path_cost_flat(next_path_h_flat), .min_next_path_cost(next_min_h)
    );

    // Path 2: Vertical (Top -> Bottom)
    aggregate_path_v #(MAX_DISP, P1_PENALTY, P2_PENALTY) path_v_engine (
        .matching_cost_flat(matching_cost_flat), .prev_path_cost_flat(prev_path_v_flat), 
        .min_prev_path_cost(min_buf_v[curr_x]), .is_path_start(curr_y == 0), 
        .next_path_cost_flat(next_path_v_flat), .min_next_path_cost(next_min_v)
    );

    // --- Winner-Take-All (WTA) Disparity Selection ---
    // Sums Path 1 and Path 2 costs; Diagonal paths (3 & 4) are disabled
    wta_selector_v #(MAX_DISP) wta_core (
        .path1_cost_flat(next_path_h_flat), .path2_cost_flat(next_path_v_flat), 
        .path3_cost_flat({(MAX_DISP*16){1'b0}}), .path4_cost_flat({(MAX_DISP*16){1'b0}}), 
        .best_disparity(selected_disparity)
    );

    // --- Sequential Memory Update & Pipeline Sync ---
    integer k;
    always @(posedge clk) begin
        if (rst) begin
            valid_out <= 0; disparity_out <= 0; min_path_h <= 16'hFFFF;
            for(k=0; k<FRAME_WIDTH; k=k+1) min_buf_v[k] <= 16'hFFFF;
        end 
        else if (pixel_valid) begin
            // Store results for next cycle/pixel
            for (k = 0; k < MAX_DISP; k = k + 1) begin
                path_h_cost[k]       <= next_path_h_flat[k*16 +: 16];
                line_buf_v[curr_x][k]  <= next_path_v_flat[k*16 +: 16];
            end
            min_path_h        <= next_min_h; 
            min_buf_v[curr_x] <= next_min_v;

            disparity_out     <= selected_disparity;
            valid_out         <= 1;
        end 
        else begin
            valid_out <= 0;
        end
    end
endmodule