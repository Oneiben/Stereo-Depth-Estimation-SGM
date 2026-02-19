# --- Vivado HLS Synthesis and Automation Tcl Script ---

# 1. Project Initialization
# Creates a fresh project environment for the SGM IP Core
open_project -reset sgm_hls_proj
set_top sgm_hls

# 2. Design and Testbench File Registration
# design_files: SGM Core logic
# tb_files: C-Simulation testbench
add_files hls/src/sgm_hls.cpp
add_files hls/src/sgm_hls.h
add_files -tb hls/tb/main_tb.cpp

# 3. Target Configuration
# Targets the xc7z020 device with a 100MHz (10ns) clock constraint
open_solution -reset "solution1"
set_part {xc7z020clg400-1}
create_clock -period 10 -name default

# 4. Hardware Generation Flow
# Run functional C-level simulation
csim_design      

# Perform High-Level Synthesis (C++ to RTL)
csynth_design     

exit