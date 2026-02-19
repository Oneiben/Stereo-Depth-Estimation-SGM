`timescale 1ns / 1ps

/**
 * @module sgm_tb_comparison_v
 * @brief Comprehensive Verilog Testbench for multi-path SGM architecture validation.
 *
 * This testbench simulates the 1-path, 2-path, and 4-path SGM hardware variants
 * simultaneously. It feeds a shared stereo image stream into all three Design Under Test (DUT)
 * modules and captures the resulting disparity maps into text files for post-simulation 
 * analysis in Python.
 */
module sgm_tb_comparison_v;

    // --- Clock and Simulation Timing ---
    reg clk = 0;
    always #5 clk = ~clk; // 100 MHz Simulation Clock

    // --- System Control and Interconnects ---
    reg        rst;
    reg  [7:0] left_pixel_in;
    reg  [7:0] right_pixel_in;
    reg        pixel_valid_in;

    // --- DUT Output Interfaces ---
    wire [5:0] disparity_1p, disparity_2p, disparity_4p;
    wire       valid_out_1p, valid_out_2p, valid_out_4p;

    // --- Image Geometry Parameters ---
    localparam FRAME_WIDTH  = 272;
    localparam FRAME_HEIGHT = 240;
    localparam TOTAL_PIXELS = FRAME_WIDTH * FRAME_HEIGHT;

    // --- Simulation Source Memories ---
    reg [7:0] memory_left  [0:TOTAL_PIXELS-1];
    reg [7:0] memory_right [0:TOTAL_PIXELS-1];

    // --- Flow Control and File I/O Handles ---
    integer in_pixel_ptr = 0;
    integer out_ptr_1p = 0, out_ptr_2p = 0, out_ptr_4p = 0;
    integer file_1p, file_2p, file_4p;

    // =========================================================================
    // DESIGN UNDER TEST (DUT) INSTANTIATIONS
    // =========================================================================

    // Variant 1: Horizontal Path only (Resource Optimized)
    sgm_top_1path_v dut_1path (
        .clk(clk), .rst(rst),
        .left_pixel(left_pixel_in), .right_pixel(right_pixel_in), .pixel_valid(pixel_valid_in),
        .disparity_out(disparity_1p), .valid_out(valid_out_1p)
    );

    // Variant 2: Horizontal + Vertical Paths (Standard Density)
    sgm_top_2path_v dut_2path (
        .clk(clk), .rst(rst),
        .left_pixel(left_pixel_in), .right_pixel(right_pixel_in), .pixel_valid(pixel_valid_in),
        .disparity_out(disparity_2p), .valid_out(valid_out_2p)
    );

    // Variant 3: 4-Path Aggregation (Full SGM Performance)
    sgm_top_4path_v dut_4path (
        .clk(clk), .rst(rst),
        .left_pixel(left_pixel_in), .right_pixel(right_pixel_in), .pixel_valid(pixel_valid_in),
        .disparity_out(disparity_4p), .valid_out(valid_out_4p)
    );

    // =========================================================================
    // INITIALIZATION & RESOURCE LOADING
    // =========================================================================
    initial begin
        $display(">>> Starting SGM Hardware Comparison Simulation...");
        
        // Load synthesized hex data into memory arrays
        $readmemh("../../../data/processed/left_pixels.hex",  memory_left);
        $readmemh("../../../data/processed/right_pixels.hex", memory_right);

        // Open result logs for software-hardware consistency check
        file_1p = $fopen("../../../results/verilog_disparity_1path.txt", "w");
        file_2p = $fopen("../../../results/verilog_disparity_2path.txt", "w");
        file_4p = $fopen("../../../results/verilog_disparity_4path.txt", "w");

        if (!file_1p || !file_2p || !file_4p) begin
            $display("CRITICAL ERROR: Failed to initialize output log files.");
            $finish;
        end

        // Hardware Reset Sequence
        rst = 1;
        pixel_valid_in = 0;
        #100 rst = 0;
    end

    // =========================================================================
    // INPUT DATA STREAMING (PIXEL FEEDER)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst) begin
            if (in_pixel_ptr < TOTAL_PIXELS) begin
                left_pixel_in  <= memory_left[in_pixel_ptr];
                right_pixel_in <= memory_right[in_pixel_ptr];
                pixel_valid_in <= 1'b1;
                in_pixel_ptr   <= in_pixel_ptr + 1;
            end 
            else begin
                pixel_valid_in <= 1'b0;
            end
        end
    end

    // =========================================================================
    // RESULT CAPTURE & DATA PERSISTENCE
    // =========================================================================
    
    // Log 1-Path results
    always @(posedge clk) begin
        if (valid_out_1p) begin
            $fwrite(file_1p, "%0d\n", disparity_1p);
            out_ptr_1p <= out_ptr_1p + 1;
        end
    end

    // Log 2-Path results
    always @(posedge clk) begin
        if (valid_out_2p) begin
            $fwrite(file_2p, "%0d\n", disparity_2p);
            out_ptr_2p <= out_ptr_2p + 1;
        end
    end

    // Log 4-Path results
    always @(posedge clk) begin
        if (valid_out_4p) begin
            $fwrite(file_4p, "%0d\n", disparity_4p);
            out_ptr_4p <= out_ptr_4p + 1;
        end
    end

    // =========================================================================
    // SIMULATION TEARDOWN
    // =========================================================================
    always @(posedge clk) begin
        // Monitor throughput and terminate simulation when all frames are processed
        if (out_ptr_1p == TOTAL_PIXELS && out_ptr_2p == TOTAL_PIXELS && out_ptr_4p == TOTAL_PIXELS) begin
            $display("-------------------------------------------------------");
            $display(" SGM CROSS-VALIDATION COMPLETE");
            $display(" 1-Path Processor: %0d pixels recorded", out_ptr_1p);
            $display(" 2-Path Processor: %0d pixels recorded", out_ptr_2p);
            $display(" 4-Path Processor: %0d pixels recorded", out_ptr_4p);
            $display("-------------------------------------------------------");
            
            $fclose(file_1p);
            $fclose(file_2p);
            $fclose(file_4p);
            $finish;
        end
    end

endmodule