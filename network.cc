// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "network.h"

#include "numerical.h"

Network::Network(int size)
    : size(size)
{
    for (int i = 0; i < NEURON_ACTIVITY_COUNT; i++) {
        this->neurons[i] = new Vector(this->size);
    }
    for (int i = 0; i < this->size; i++) {
        this->neurons[current_activity]->values[i] = Random::uniform() * 0.0001;
    }
    this->inputs = new std::vector<Input *>();
    this->neuron_inputs = new Vector(this->size);
}

Input *Network::add_input(Input *input)
{
    input->initialize();
    this->inputs->push_back(input);
    return input;
}

void Network::update()
{
    this->update_neuron_inputs();
    this->update_neuron_values();
}

void Network::commit()
{
    Vector *prev_neurons = this->neurons[current_activity];
    this->neurons[current_activity] = this->neurons[next_activity];
    this->neurons[next_activity] = prev_neurons;
}

void Network::update_and_commit()
{
    this->update();
    this->commit();
}

bool Network::should_update_neuron(int neuron_index)
{
    return true;
}

void Network::update_neuron_inputs()
{
    this->neuron_inputs->clear();
    for (Input *input : *this->inputs) {
        if (input->is_active()) {
            input->add_inputs();
        }
    }
}

Input::Input(Network *efferent)
    : efferent(efferent)
{
}

void Input::initialize()
{
}

void Input::set_active(bool active)
{
    this->active = active;
}

bool Input::is_active()
{
    return this->active;
}
