// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef NETWORK_H_INCLUDED
#define NETWORK_H_INCLUDED

#include <vector>

#include "numerical.h"

class Input;

enum NeuronActivity {
    current_activity,
    next_activity,
    original_activity,

    NEURON_ACTIVITY_COUNT
};

class Network
{
    public:
        Network(int size);
        Input *add_input(Input *input);
        virtual void update();
        virtual void commit();
        void update_and_commit();
        virtual bool should_update_neuron(int neuron_index);

        int size;
        Vector *neurons[NEURON_ACTIVITY_COUNT];
        std::vector<Input *> *inputs;
        Vector *neuron_inputs;

    protected:
        void update_neuron_inputs();
        virtual void update_neuron_values() = 0;
};

class Input
{
    public:
        Input(Network *efferent);
        virtual void initialize();
        virtual void add_inputs() = 0;
        void set_active(bool active);
        bool is_active();

    protected:
        Network *efferent;
        bool active = true;
};

#endif
