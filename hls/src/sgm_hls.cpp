#include "sgm_hls.h"

/**
 * @brief Computes the initial matching cost volume using Absolute Difference (AD).
 * @param left_pixels   Flat input array of the reference (left) grayscale image.
 * @param right_pixels  Flat input array of the target (right) grayscale image.
 * @param cost_volume   Output 3D tensor storing C(p, d) for all pixels and disparities.
 */
void compute_sad_cost_hls(
    float left_pixels[HEIGHT * WIDTH],
    float right_pixels[HEIGHT * WIDTH],
    float cost_volume[HEIGHT][WIDTH][MAX_DISP])
{
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
#pragma HLS PIPELINE II = 1
            int pixel_idx = y * WIDTH + x;
            for (int d = 0; d < MAX_DISP; d++)
            {
                // Verify target pixel remains within image boundaries
                if (x - d >= 0)
                {
                    // Pixel-wise absolute difference calculation
                    cost_volume[y][x][d] = hls::fabs(left_pixels[pixel_idx] - right_pixels[y * WIDTH + (x - d)]);
                }
                else
                {
                    // Assign maximum penalty for out-of-bounds disparity shifts
                    cost_volume[y][x][d] = 1000.0f;
                }
            }
        }
    }
}

/**
 * @brief Aggregates cost along a 1D path according to the SGM energy minimization recursive formula.
 * * L_r(p, d) = C(p, d) + min [ L_r(p-r, d), L_r(p-r, d-1)+P1, L_r(p-r, d+1)+P1, min_k(L_r(p-r, k))+P2 ] - min_k(L_r(p-r, k))
 * * @param cost_volume     Input matching cost volume C(p, d).
 * @param path_cost_volume  Output aggregated cost volume L_r(p, d) for the current direction.
 * @param dir_y             Vertical direction component (dy).
 * @param dir_x             Horizontal direction component (dx).
 */
void aggregate_path_hls(
    float cost_volume[HEIGHT][WIDTH][MAX_DISP],
    float path_cost_volume[HEIGHT][WIDTH][MAX_DISP],
    int dir_y, int dir_x)
{
    // Determine iteration scan order based on the aggregation direction vector
    int y_start = (dir_y >= 0) ? 0 : HEIGHT - 1;
    int y_end = (dir_y >= 0) ? HEIGHT : -1;
    int y_step = (dir_y >= 0) ? 1 : -1;

    int x_start = (dir_x >= 0) ? 0 : WIDTH - 1;
    int x_end = (dir_x >= 0) ? WIDTH : -1;
    int x_step = (dir_x >= 0) ? 1 : -1;

    for (int y = y_start; y != y_end; y += y_step)
    {
        for (int x = x_start; x != x_end; x += x_step)
        {
#pragma HLS PIPELINE II = 1
            int prev_y = y - dir_y;
            int prev_x = x - dir_x;

            // Check if the previous pixel in the path is within the frame boundaries
            if (prev_y >= 0 && prev_y < HEIGHT && prev_x >= 0 && prev_x < WIDTH)
            {
                // Find the minimum aggregated cost at the previous pixel across all disparities for normalization
                float min_prev_aggregated = path_cost_volume[prev_y][prev_x][0];
                for (int i = 1; i < MAX_DISP; i++)
                {
                    if (path_cost_volume[prev_y][prev_x][i] < min_prev_aggregated)
                        min_prev_aggregated = path_cost_volume[prev_y][prev_x][i];
                }

                for (int d = 0; d < MAX_DISP; d++)
                {
                    // Case 0: No change in disparity
                    float cost_same = path_cost_volume[prev_y][prev_x][d];

                    // Case 1 & 2: Small disparity change (+/- 1) penalized by P1
                    float cost_step_down = (d > 0) ? path_cost_volume[prev_y][prev_x][d - 1] + P1_PENALTY : 2000.0f;
                    float cost_step_up = (d < MAX_DISP - 1) ? path_cost_volume[prev_y][prev_x][d + 1] + P1_PENALTY : 2000.0f;

                    // Case 3: Large disparity change (>1) penalized by P2
                    float cost_jump = min_prev_aggregated + P2_PENALTY;

                    // Select the minimum cost among all possible transitions
                    float min_transition_cost = cost_same;
                    if (cost_step_down < min_transition_cost)
                        min_transition_cost = cost_step_down;
                    if (cost_step_up < min_transition_cost)
                        min_transition_cost = cost_step_up;
                    if (cost_jump < min_transition_cost)
                        min_transition_cost = cost_jump;

                    // Update path cost: L_r(p, d) = C(p, d) + min_transition - min_prev_normalization
                    path_cost_volume[y][x][d] = cost_volume[y][x][d] + (min_transition_cost - min_prev_aggregated);
                }
            }
            else
            {
                // Boundary condition: Initialize path cost with raw matching cost
                for (int d = 0; d < MAX_DISP; d++)
                    path_cost_volume[y][x][d] = cost_volume[y][x][d];
            }
        }
    }
}

/**
 * @brief Top-level HLS entry point for Semi-Global Matching (SGM).
 * Performs matching cost calculation, 4-path aggregation, and Winner-Take-All disparity selection.
 */
void sgm_hls(
    float left_pixels[HEIGHT * WIDTH],
    float right_pixels[HEIGHT * WIDTH],
    int disparity_output[HEIGHT * WIDTH])
{
// AXI4-Master interfaces for high-bandwidth off-chip memory access
#pragma HLS INTERFACE m_axi port = left_pixels offset = slave bundle = gmem
#pragma HLS INTERFACE m_axi port = right_pixels offset = slave bundle = gmem
#pragma HLS INTERFACE m_axi port = disparity_output offset = slave bundle = gmem
// AXI4-Lite interface for IP core control and status
#pragma HLS INTERFACE s_axilite port = return bundle = control

    // On-chip memory allocation for cost volumes (requires BRAM/URAM resources)
    static float cost_volume[HEIGHT][WIDTH][MAX_DISP];
    static float path_left_to_right[HEIGHT][WIDTH][MAX_DISP];
    static float path_right_to_left[HEIGHT][WIDTH][MAX_DISP];
    static float path_top_to_bottom[HEIGHT][WIDTH][MAX_DISP];
    static float path_bottom_to_top[HEIGHT][WIDTH][MAX_DISP];

// Partitioning to allow parallel access to multiple disparity entries per clock cycle
#pragma HLS ARRAY_PARTITION variable = cost_volume cyclic factor = 8 dim = 3

    // 1. Matching Cost Computation
    compute_sad_cost_hls(left_pixels, right_pixels, cost_volume);

    // 2. 4-Path Cost Aggregation (Horizontal and Vertical directions)
    aggregate_path_hls(cost_volume, path_left_to_right, 0, 1);
    aggregate_path_hls(cost_volume, path_right_to_left, 0, -1);
    aggregate_path_hls(cost_volume, path_top_to_bottom, 1, 0);
    aggregate_path_hls(cost_volume, path_bottom_to_top, -1, 0);

    // 3. Final Summation and Winner-Take-All (WTA) Disparity Selection
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
#pragma HLS PIPELINE II = 1
            float min_total_cost = 1e9;
            int best_disparity = 0;

            for (int d = 0; d < MAX_DISP; d++)
            {
                // Combine costs from all four aggregation paths
                float total_aggregated_cost = path_left_to_right[y][x][d] +
                                              path_right_to_left[y][x][d] +
                                              path_top_to_bottom[y][x][d] +
                                              path_bottom_to_top[y][x][d];

                // Select disparity with the lowest total energy (WTA)
                if (total_aggregated_cost < min_total_cost)
                {
                    min_total_cost = total_aggregated_cost;
                    best_disparity = d;
                }
            }
            disparity_output[y * WIDTH + x] = best_disparity;
        }
    }
}
