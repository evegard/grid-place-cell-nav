// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#include <string>

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

enum MecGainMode {
    gain_mode_velocity,
    gain_mode_poisson_neuron,

    GAIN_MODE_COUNT
};

struct SimulationConf {
    bool live_plot;
    bool final_plot;
    bool lite_plot;
    std::string script_source;
};

struct ModelConf {
    int module_count;
    MecGainMode gain_mode;
    double gain_ratio;
    double initial_gain;
    bool alternative_motor_scaling;
    bool simplified_mec_diff;
    int direction_samples;
    int xy_samples;
    int mec_diff_offset;
    int sensor_count;
    double sensor_range;
    double place_cell_radius;
    double internal_motor_tuning;
};

// mec.h

#define MEC_SIZE 40

#define MAX_MEC_SPEED 120.0
#define FIXED_SPEED 20.0
#define MAX_MEC_GAIN ((MAX_MEC_SPEED) / (FIXED_SPEED) * 0.01)

#define BUMP_TRACKER_RADIUS 5

// numerical.h

#define REAL_ALIGNMENT 32
#define REAL_STRIDE 8

// simulation.h

#define STEPS_PER_SECOND 1000
#define PLOT_UPDATE_INTERVAL 100

#define DISTANCE_PER_TIMESTEP ((FIXED_SPEED) / (STEPS_PER_SECOND))

// motor.h

#define GRID_MOTOR_PLOT_RANGE 4.0
#define ALL_MOTORS_PLOT_RANGE 8.0
#define UI_MOTOR_PLOT_RANGE 2.0

#define _STRINGIFY_CONSTANT(arg) #arg
#define STRINGIFY_CONSTANT(arg) _STRINGIFY_CONSTANT(arg)

// model.h

#define SETTLE_STEPS 1000

// graph.h

#define PLACE_CONNECTION_STRENGTH 2

#endif
