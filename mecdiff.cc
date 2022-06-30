// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "mecdiff.h"

#include <cstdlib>

MecDiffNetwork::MecDiffNetwork(
        bool simplified,
        NeuralSheetNetwork *current, NeuralSheetNetwork *target,
        int direction_samples, int xy_samples, int offset)
    : Network(direction_samples * xy_samples * xy_samples),
      simplified(simplified), current(current), target(target),
      direction_samples(direction_samples), xy_samples(xy_samples), offset(offset)
{
    if (simplified) {
        this->add_input(new MecDiffSimplifiedInput(this, current, 0));
        this->add_input(new MecDiffSimplifiedInput(this, target, offset));
    } else {
        this->add_input(new MecDiffCurrentInput(this, current));
        this->add_input(new MecDiffTargetInput(this, target, offset));
    }
}

void MecDiffNetwork::update_neuron_values()
{
    for (int i = 0; i < this->size; i++) {
        real input = this->neuron_inputs->values[i];
        if (this->simplified) {
            input -= 0.6;
        }
        if (input < 0) {
            input = 0.0;
        }
        this->neurons[next_activity]->values[i] = input;
    }
}

MecDiffCurrentInput::MecDiffCurrentInput(
        MecDiffNetwork *efferent, NeuralSheetNetwork *afferent)
    : MecShiftedMaskInput(efferent, afferent), efferent(efferent)
{
}

real MecDiffCurrentInput::get_weight(int x, int y)
{
    if (x > MEC_SIZE / 2) {
        x = MEC_SIZE - x;
    }
    if (y > MEC_SIZE / 2) {
        y = MEC_SIZE - y;
    }
    real distance_squared = x * x + y * y;
    return 0.25 * (exp(-this->afferent->beta * distance_squared) - 1);
}

std::pair<int, int> MecDiffCurrentInput::get_shift(int neuron_index)
{
    return std::pair<int, int>(
        this->efferent->x(neuron_index),
        this->efferent->y(neuron_index));
}

MecDiffTargetInput::MecDiffTargetInput(
        MecDiffNetwork *efferent, NeuralSheetNetwork *afferent, int offset)
    : MecShiftedMaskInput(efferent, afferent), efferent(efferent),
      offset(offset)
{
}

real MecDiffTargetInput::get_weight(int x, int y)
{
    if (x > MEC_SIZE / 2) {
        x = MEC_SIZE - x;
    }
    if (y > MEC_SIZE / 2) {
        y = MEC_SIZE - y;
    }
    real distance_squared = x * x + y * y;
    return exp(-this->afferent->beta * distance_squared);
}

std::pair<int, int> MecDiffTargetInput::get_shift(int neuron_index)
{
    real direction = this->efferent->direction(neuron_index);
    int x = round(this->efferent->x(neuron_index) + this->offset * cos(direction));
    int y = round(this->efferent->y(neuron_index) + this->offset * sin(direction));
    x = Periodic::modulo(x, MEC_SIZE);
    y = Periodic::modulo(y, MEC_SIZE);
    return std::pair<int, int>(x, y);
}

MecDiffSimplifiedInput::MecDiffSimplifiedInput(
        MecDiffNetwork *efferent, NeuralSheetNetwork *afferent, int offset)
    : Input(efferent), afferent(afferent)
{
    this->input_indices = new int[efferent->size];
    for (int neuron_index = 0; neuron_index < efferent->size; neuron_index++) {
        real direction = efferent->direction(neuron_index);
        int x = round(efferent->x(neuron_index) + offset * cos(direction));
        int y = round(efferent->y(neuron_index) + offset * sin(direction));
        x = Periodic::modulo(x, MEC_SIZE);
        y = Periodic::modulo(y, MEC_SIZE);
        int input_neuron_index = afferent->coords_to_neuron_index(x, y);
        this->input_indices[neuron_index] = input_neuron_index;
    }
}

void MecDiffSimplifiedInput::add_inputs()
{
    for (int neuron_index = 0; neuron_index < this->efferent->size; neuron_index++) {
        this->efferent->neuron_inputs->values[neuron_index] +=
            this->afferent->neurons[current_activity]->values[
                this->input_indices[neuron_index]];
    }
}
