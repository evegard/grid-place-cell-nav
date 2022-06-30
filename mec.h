// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef MEC_H_INCLUDED
#define MEC_H_INCLUDED

#include <tuple>

#include "network.h"
#include "plot.h"
#include "main.h"

class MecRecurrentInput;

enum MecDirectionality { west, north, south, east };

class NeuralSheetNetwork : public Network
{
    public:
        NeuralSheetNetwork(real gain);

        real gain;
        real lambda, beta, gamma;

        inline int neuron_index_to_x(int i) { return i % MEC_SIZE; }
        inline int neuron_index_to_y(int i) { return i / MEC_SIZE; }
        inline int coords_to_neuron_index(int x, int y) { return y * MEC_SIZE + x; }

        int bump_x, bump_y;
        int bump_total_dx = 0, bump_total_dy = 0;
        bool bump_tracker_initialized;
        void initialize_bump_tracker();
        void update_bump_tracker();

    protected:
        std::tuple<real, int, int> calculate_disc_mass(int center_x, int center_y);
};

class MecNetwork : public NeuralSheetNetwork
{
    public:
        MecNetwork(real gain, MecGainMode gain_mode);
        MecGainMode gain_mode;
        real activation_probability;

        void update();
        bool should_update_neuron(int neuron_index);

        inline MecDirectionality directionality(int x, int y) {
            return (MecDirectionality)(2 * (y % 2) + (x % 2)); }

    protected:
        void update_neuron_values();
        MecRecurrentInput *recurrent_input;
        bool neurons_enabled[MEC_SIZE * MEC_SIZE];
};

class ConvolvedMecNetwork : public NeuralSheetNetwork
{
    public:
       ConvolvedMecNetwork(MecNetwork *afferent);

    protected:
        void update_neuron_values();
};

class MecConvolveInput : public Input
{
    public:
        MecConvolveInput(ConvolvedMecNetwork *efferent, MecNetwork *afferent);
        void add_inputs();

    protected:
        ConvolvedMecNetwork *efferent;
        MecNetwork *afferent;
};

class MecShiftedMaskInput : public Input
{
    public:
        MecShiftedMaskInput(Network *efferent, NeuralSheetNetwork *afferent);
        void initialize();
        void add_inputs();

    protected:
        NeuralSheetNetwork *afferent;
        Matrix *weights;

        std::pair<int, int> *shifts;
        real cached_sums[MEC_SIZE][MEC_SIZE];
        bool cached_sum_valid[MEC_SIZE][MEC_SIZE];

        virtual real get_weight(int x, int y) = 0;
        virtual std::pair<int, int> get_shift(int neuron_index) = 0;
};

class MecRecurrentInput : public MecShiftedMaskInput
{
    public:
        MecRecurrentInput(MecNetwork *network);

    protected:
        MecNetwork *afferent;
        real get_weight(int x, int y);
        std::pair<int, int> get_shift(int neuron_index);
};

class MecNetworkPlot : public Plot
{
    public:
        MecNetworkPlot(NeuralSheetNetwork *network, int number);

    protected:
        void dump_plot_commands(std::ostream &gnuplot);
        NeuralSheetNetwork *network;
};

#endif
