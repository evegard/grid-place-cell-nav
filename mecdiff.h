// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef MECDIFF_H_INCLUDED
#define MECDIFF_H_INCLUDED

#include <cmath>

#include "mec.h"
#include "network.h"
#include "numerical.h"

class MecDiffNetwork : public Network
{
    public:
        MecDiffNetwork(
            bool simplified,
            NeuralSheetNetwork *current, NeuralSheetNetwork *target,
            int direction_samples, int xy_samples, int offset);

        bool simplified;

        NeuralSheetNetwork *current;
        NeuralSheetNetwork *target;
        int direction_samples;
        int xy_samples;
        int offset;

        inline int direction_sample(int i) { return i % this->direction_samples; }
        inline int x_sample(int i) { return (i / this->direction_samples) % this->xy_samples; }
        inline int y_sample(int i) { return (i / this->direction_samples) / this->xy_samples; }

        inline real direction(int i) { return this->direction_sample(i) * 2 * M_PI / this->direction_samples; }
        inline int x(int i) { return this->x_sample(i) * MEC_SIZE / this->xy_samples; }
        inline int y(int i) { return this->y_sample(i) * MEC_SIZE / this->xy_samples; }

        inline int neuron_index(int direction, int x, int y)
            { return (y * this->xy_samples + x) * this->direction_samples + direction; }

    protected:
        void update_neuron_values();
};

class MecDiffCurrentInput : public MecShiftedMaskInput
{
    public:
        MecDiffCurrentInput(
            MecDiffNetwork *efferent,
            NeuralSheetNetwork *afferent);

    protected:
        MecDiffNetwork *efferent;

        real get_weight(int x, int y);
        std::pair<int, int> get_shift(int neuron_index);
};

class MecDiffTargetInput : public MecShiftedMaskInput
{
    public:
        MecDiffTargetInput(
            MecDiffNetwork *efferent,
            NeuralSheetNetwork *afferent,
            int offset);

    protected:
        MecDiffNetwork *efferent;
        int offset;

        real get_weight(int x, int y);
        std::pair<int, int> get_shift(int neuron_index);
};

class MecDiffSimplifiedInput : public Input
{
    public:
        MecDiffSimplifiedInput(
            MecDiffNetwork *efferent,
            NeuralSheetNetwork *afferent,
            int offset);
        void add_inputs();

    protected:
        NeuralSheetNetwork *afferent;
        int *input_indices;
};

#endif
