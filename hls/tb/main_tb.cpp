#include "sgm_hls.h"
#include <fstream>
#include <iostream>
#include <string>

/**
 * @file main_tb.cpp
 * @brief Testbench for verifying the SGM HLS implementation against golden pixel data.
 */

// Define relative paths for portability across different build environments
#ifndef DATA_PATH
#define DATA_PATH "../../../data/processed/"
#endif

#ifndef RESULT_PATH
#define RESULT_PATH "../../../results/"
#endif

int main()
{
    // Allocate image buffers on the heap to avoid stack overflow during C-Simulation (Large Arrays)
    float *image_left_pixels = new float[HEIGHT * WIDTH];
    float *image_right_pixels = new float[HEIGHT * WIDTH];
    int *disparity_output = new int[HEIGHT * WIDTH];

    // Construct absolute/relative file paths for dataset and result logging
    std::string path_left_input = std::string(DATA_PATH) + "left_pixels.txt";
    std::string path_right_input = std::string(DATA_PATH) + "right_pixels.txt";
    std::string path_result_out = std::string(RESULT_PATH) + "hls_disparity.txt";

    // Initialize file input streams
    std::ifstream stream_left(path_left_input);
    std::ifstream stream_right(path_right_input);

    if (!stream_left.is_open() || !stream_right.is_open())
    {
        std::cerr << "CRITICAL ERROR: Input dataset not found!" << std::endl;
        std::cerr << "Missing sequence at: " << path_left_input << std::endl;
        return -1;
    }

    std::cout << ">>> Initializing SGM HLS Simulation (Resolution: " << WIDTH << "x" << HEIGHT << ")..." << std::endl;

    // Load pixel-stream data into memory buffers
    for (int i = 0; i < HEIGHT * WIDTH; i++)
    {
        stream_left >> image_left_pixels[i];
        stream_right >> image_right_pixels[i];
    }

    // Execute Top-Level IP Core Function (Under Test)
    sgm_hls(image_left_pixels, image_right_pixels, disparity_output);

    // Persist resulting disparity map to text for Python/RTL verification
    std::ofstream stream_out(path_result_out);
    for (int i = 0; i < HEIGHT * WIDTH; i++)
    {
        stream_out << disparity_output[i] << "\n";
    }

    std::cout << ">>> Simulation Complete. Hardware results saved to: " << path_result_out << std::endl;

    // Release heap-allocated resources
    delete[] image_left_pixels;
    delete[] image_right_pixels;
    delete[] disparity_output;

    return 0;
}