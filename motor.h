// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#include <tuple>

#include "network.h"
#include "numerical.h"
#include "plot.h"

class MecDiffNetwork;
class MecDiffMotorInput;
class MotorMotorInput;

class MotorNetwork : public Network
{
    public:
        MotorNetwork(int direction_samples, double scaling_factor, bool normalize);
        void commit();

        int direction_samples;
        double scaling_factor;
        bool normalize;
        double normalization_spread = 2 * M_PI; // Set to a large-ish value, should
                                                // always be overridden anyways if
                                                // normalization is used
        double normalization_peak = 1.0;

        bool override_active = false;
        double override_direction = 0.0;
        double override_strength = 0.0;

        double direction = 0.0;
        double strength = 0.0;

    protected:
        void update_neuron_values();
        std::tuple<double, double> calculate_direction_and_strength(NeuronActivity activity);
};

class MecDiffMotorInput : public Input
{
    public:
        MecDiffMotorInput(
            MotorNetwork *motor_network,
            MecDiffNetwork *mec_diff_network);
        void add_inputs();

        MotorNetwork *motor_network;
        MecDiffNetwork *mec_diff_network;
};

class MotorMotorInput : public Input
{
    public:
        MotorMotorInput(
            MotorNetwork *efferent,
            MotorNetwork *afferent);
        void add_inputs();

        MotorNetwork *efferent;
        MotorNetwork *afferent;
};

class BorderMotorInput : public Input
{
    public:
        BorderMotorInput(MotorNetwork *efferent, Vector *border_sensors);
        void add_inputs();

        MotorNetwork *efferent;
        Vector *border_sensors;
};

class MotorNetworkPlot : public Plot
{
    public:
        MotorNetworkPlot(MotorNetwork *network,
            const char *color, const char *title, bool simplified, double plot_range);

    protected:
        void dump_plot_commands(std::ostream &gnuplot);
        MotorNetwork *network;
        const char *color;
        bool simplified;
        double plot_range;
};

#endif
