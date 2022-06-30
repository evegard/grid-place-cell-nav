// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "mec.h"

#include <cmath>
#include <cstdlib>

NeuralSheetNetwork::NeuralSheetNetwork(real gain)
    : Network(MEC_SIZE * MEC_SIZE),
      bump_tracker_initialized(false)
{
    this->gain = gain;
    this->lambda = MEC_SIZE * 15.0 / 40.0; // "Periodicity"
    this->beta = 3.0 / (this->lambda * this->lambda);
    this->gamma = 1.05 * this->beta;
}

void NeuralSheetNetwork::initialize_bump_tracker()
{
    // Initialize the bump tracker to the maximally activated neuron
    real max_activation = -1;
    for (int y = 0; y < MEC_SIZE; y++) {
        for (int x = 0; x < MEC_SIZE; x++) {
            int neuron_index = this->coords_to_neuron_index(x, y);
            real activation = this->neurons[current_activity]->values[neuron_index];
            if (activation > max_activation) {
                max_activation = activation;
                this->bump_x = x;
                this->bump_y = y;
            }
        }
    }
    // Run an initial update of the bump tracker to iteratively move the
    // bump location to the center of mass that maximizes disc mass, and
    // afterwards reset the (dx, dy) values back to (0, 0). This whole
    // step does not really appear to be necessary, as the (dx, dy)
    // values seem to stay at (0, 0). However, there is no guarantee for
    // this to always happen, and we therefore retain this step
    this->bump_tracker_initialized = true;
    this->update_bump_tracker();
    this->bump_total_dx = 0;
    this->bump_total_dy = 0;
}

void NeuralSheetNetwork::update_bump_tracker()
{
    if (!this->bump_tracker_initialized) {
        return;
    }
    while (true) {
        // Calculate the mass under the disc centered at the current bump
        // location, as well as the displacement to the center of mass
        real current_mass;
        int center_of_mass_dx, center_of_mass_dy;
        std::tie(current_mass, center_of_mass_dx, center_of_mass_dy) =
            this->calculate_disc_mass(this->bump_x, this->bump_y);
        // Calculate the (wrapped) coordinates for the center of mass, as our
        // candidate for the new bump location
        int center_of_mass_x = Periodic::modulo(this->bump_x + center_of_mass_dx, MEC_SIZE);
        int center_of_mass_y = Periodic::modulo(this->bump_y + center_of_mass_dy, MEC_SIZE);
        // Calculate the mass at the new potential bump location
        real new_mass;
        std::tie(new_mass, std::ignore, std::ignore) =
            this->calculate_disc_mass(center_of_mass_x, center_of_mass_y);
        // If the mass is increased by moving to the new bump location, update
        // the bump location as well as the counters for total bump displacement
        if (new_mass > current_mass) {
            this->bump_x = center_of_mass_x;
            this->bump_y = center_of_mass_y;
            this->bump_total_dx += center_of_mass_dx;
            this->bump_total_dy += center_of_mass_dy;
        } else {
            break;
        }
    }
}

std::tuple<real, int, int> NeuralSheetNetwork::calculate_disc_mass(int center_x, int center_y)
{
    real mass = 0.0, weighted_dx = 0.0, weighted_dy = 0;
    for (int dy = -BUMP_TRACKER_RADIUS; dy < BUMP_TRACKER_RADIUS + 1; dy++) {
        for (int dx = -BUMP_TRACKER_RADIUS; dx < BUMP_TRACKER_RADIUS + 1; dx++) {
            if (dx * dx + dy * dy > BUMP_TRACKER_RADIUS * BUMP_TRACKER_RADIUS) {
                continue;
            }
            int x = Periodic::modulo(center_x + dx, MEC_SIZE);
            int y = Periodic::modulo(center_y + dy, MEC_SIZE);
            int neuron_index = this->coords_to_neuron_index(x, y);
            real activation = this->neurons[current_activity]->values[neuron_index];
            mass += activation;
            weighted_dx += dx * activation;
            weighted_dy += dy * activation;
        }
    }
    int center_of_mass_dx = round(weighted_dx / mass);
    int center_of_mass_dy = round(weighted_dy / mass);
    return std::make_tuple(mass, center_of_mass_dx, center_of_mass_dy);
}

MecNetwork::MecNetwork(real gain, MecGainMode gain_mode)
    : NeuralSheetNetwork(gain), gain_mode(gain_mode),
      activation_probability(gain / MAX_MEC_GAIN)
{
    this->add_input(new MecRecurrentInput(this));
}

void MecNetwork::update()
{
    for (int i = 0; i < this->size; i++) {
        if (this->gain_mode == gain_mode_velocity) {
            this->neurons_enabled[i] = true;
        } else if (this->gain_mode == gain_mode_poisson_neuron) {
            this->neurons_enabled[i] = (Random::uniform() < this->activation_probability);
        }
    }
    Network::update();
}

bool MecNetwork::should_update_neuron(int neuron_index)
{
    return this->neurons_enabled[neuron_index];
}

void MecNetwork::update_neuron_values()
{
    for (int i = 0; i < this->size; i++) {
        if (this->neurons_enabled[i]) {
            real input = 1.0 + this->neuron_inputs->values[i];
            if (input < 0.0) {
                input = 0.0;
            }
            real output_change = (input - this->neurons[current_activity]->values[i]);
            output_change *= 0.1;
            this->neurons[next_activity]->values[i] =
                this->neurons[current_activity]->values[i] + output_change;
        } else {
            this->neurons[next_activity]->values[i] =
                this->neurons[current_activity]->values[i];
        }
    }
}

ConvolvedMecNetwork::ConvolvedMecNetwork(MecNetwork *afferent)
    : NeuralSheetNetwork(afferent->gain)
{
    this->add_input(new MecConvolveInput(this, afferent));
}

void ConvolvedMecNetwork::update_neuron_values()
{
    for (int neuron_index = 0; neuron_index < this->size; neuron_index++) {
        this->neurons[next_activity]->values[neuron_index] =
            this->neuron_inputs->values[neuron_index];
    }
}

MecConvolveInput::MecConvolveInput(
        ConvolvedMecNetwork *efferent, MecNetwork *afferent)
    : Input(efferent), efferent(efferent), afferent(afferent)
{
}

void MecConvolveInput::add_inputs()
{
    for (int y = 0; y < MEC_SIZE; y++) {
        for (int x = 0; x < MEC_SIZE; x++) {
            int afferent_index = this->afferent->coords_to_neuron_index(x, y);
            real afferent_value = this->afferent->neurons[current_activity]->values[afferent_index];
            for (int dy = 0; dy < 2; dy++) {
                for (int dx = 0; dx < 2; dx++) {
                    int efferent_x = (x + dx) % MEC_SIZE;
                    int efferent_y = (y + dy) % MEC_SIZE;
                    int efferent_index = this->efferent->coords_to_neuron_index(efferent_x, efferent_y);
                    this->efferent->neuron_inputs->values[efferent_index] += 0.25 * afferent_value;
                }
            }
        }
    }
}

MecShiftedMaskInput::MecShiftedMaskInput(
        Network *efferent, NeuralSheetNetwork *afferent)
    : Input(efferent), afferent(afferent)
{
}

void MecShiftedMaskInput::initialize()
{
    int w = MEC_SIZE;
    int h = MEC_SIZE;
    this->weights = new Matrix(2 * w, 2 * h);
    for (int y = 0; y < MEC_SIZE; y++) {
        for (int x = 0; x < MEC_SIZE; x++) {
            real weight = this->get_weight(x, y);
            this->weights->values[y + 0][x + 0] = weight;
            this->weights->values[y + 0][x + w] = weight;
            this->weights->values[y + h][x + 0] = weight;
            this->weights->values[y + h][x + w] = weight;
        }
    }
    this->shifts = new std::pair<int, int>[this->efferent->size];
    for (int neuron_index = 0; neuron_index < this->efferent->size; neuron_index++) {
        this->shifts[neuron_index] = this->get_shift(neuron_index);
    }
}

void MecShiftedMaskInput::add_inputs()
{
    // Each efferent neuron can separately specify its desired shift of the
    // connectivity profile. For MEC neurons this will be their own locations
    // in the neural sheet plus their directional-preference-offset, while for
    // MEC-diff neurons this depends on their sampled (x, y, direction) tuple.
    // Because multiple efferent neurons can share the same origin, we cache
    // the sum for each given origin. This cached sum is only considered valid
    // during the current execution of this method, therefore we reset the
    // cache validity here at the beginning of the method.
    for (int y = 0; y < MEC_SIZE; y++) {
        for (int x = 0; x < MEC_SIZE; x++) {
            this->cached_sum_valid[y][x] = false;
        }
    }
    for (int efferent_neuron = 0; efferent_neuron < this->efferent->size; efferent_neuron++) {
        if (!this->efferent->should_update_neuron(efferent_neuron)) {
            continue;
        }
        std::pair<int, int> shift = this->shifts[efferent_neuron];
        if (this->cached_sum_valid[shift.second][shift.first]) {
            this->efferent->neuron_inputs->values[efferent_neuron] +=
                this->cached_sums[shift.second][shift.first];
            continue;
        }
        int shift_x = MEC_SIZE - shift.first;
        int shift_y = MEC_SIZE - shift.second;

        real sum = 0.0;
        aligned_real *neurons = this->afferent->neurons[current_activity]->values;
        real *weights = &this->weights->values[shift_y][shift_x];
        for (int y = 0; y < MEC_SIZE; y++) {
            for (int x = 0; x < MEC_SIZE; x++) {
                sum += neurons[x] * weights[x];
            }
            neurons += MEC_SIZE;
            weights += MEC_SIZE * 2;
        }
        this->efferent->neuron_inputs->values[efferent_neuron] += sum;
        this->cached_sums[shift.second][shift.first] = sum;
        this->cached_sum_valid[shift.second][shift.first] = true;
    }
}

MecRecurrentInput::MecRecurrentInput(MecNetwork *network)
    : MecShiftedMaskInput(network, network), afferent(network)
{
}

real MecRecurrentInput::get_weight(int x, int y)
{
    if (x > MEC_SIZE / 2) {
        x = MEC_SIZE - x;
    }
    if (y > MEC_SIZE / 2) {
        y = MEC_SIZE - y;
    }
    real distance_squared = x * x + y * y;
    return exp(-this->afferent->gamma * distance_squared)
         - exp(-this->afferent->beta  * distance_squared);
}

std::pair<int, int> MecRecurrentInput::get_shift(int neuron_index)
{
    int x = this->afferent->neuron_index_to_x(neuron_index);
    int y = this->afferent->neuron_index_to_y(neuron_index);
    switch (this->afferent->directionality(x, y)) {
    case north: y--; break;
    case south: y++; break;
    case east: x--; break;
    case west: x++; break;
    }
    x = Periodic::modulo(x, MEC_SIZE);
    y = Periodic::modulo(y, MEC_SIZE);
    return std::pair<int, int>(x, y);
}

MecNetworkPlot::MecNetworkPlot(NeuralSheetNetwork *network, int number)
    : network(network)
{
    std::string plot_range = "[-0.5:" + std::to_string(MEC_SIZE - 1) + ".5]";
    this->set("xrange", plot_range.c_str());
    this->set("yrange", plot_range.c_str());
    this->set("size", "square");
    this->unset("xtics");
    this->unset("ytics");
    this->set("cbrange", "[0:0.6]");
    this->unset("colorbox");
    this->set("border", "");
    this->set("margins", "0,5,1,5");
    std::string title_string =
        "\"{/:Bold=14 Grid module " + std::to_string(number) +
        "}\\n@g_{&{g}" + std::to_string(number) +
        "} = " + std::to_string(network->gain) + "\"";
    this->set("title", title_string.c_str());

    int object_number = 5;
    for (auto blob : std::vector<std::tuple<double, int, int>> {
            std::make_tuple(0.04, 6, -1),
            std::make_tuple(0.04, 4, number),
            std::make_tuple(0.02, 6, number) }) {
        std::string blob_key = "object " + std::to_string(object_number++);
        std::string blob_value = "circle at graph -0.025,1.25 size graph " +
            std::to_string(std::get<0>(blob)) + " noclip fill solid border lt " +
            std::to_string(std::get<2>(blob)) + " lw " +
            std::to_string(std::get<1>(blob));
        this->set(blob_key.c_str(), blob_value.c_str());
    }
}

void MecNetworkPlot::dump_plot_commands(std::ostream &stream)
{
    int origin_bump_x = Periodic::modulo(
        this->network->bump_x - this->network->bump_total_dx, MEC_SIZE);
    int origin_bump_y = Periodic::modulo(
        this->network->bump_y - this->network->bump_total_dy, MEC_SIZE);
    int bump_radius = BUMP_TRACKER_RADIUS;

    stream << "set object 1 circle at " << origin_bump_x << "," << origin_bump_y
        << " size " << bump_radius << " noclip front lw 6 fc rgb 'black';" << std::endl;
    stream << "set object 2 circle at " << origin_bump_x << "," << origin_bump_y
        << " size " << bump_radius << " noclip front lw 4 fc rgb 'green';" << std::endl;

    stream << "set object 3 circle at " << this->network->bump_x << "," << this->network->bump_y
        << " size " << bump_radius << " noclip front lw 6 fc rgb 'black';" << std::endl;
    stream << "set object 4 circle at " << this->network->bump_x << "," << this->network->bump_y
        << " size " << bump_radius << " noclip front lw 4 fc rgb 'yellow';" << std::endl;

    stream << "plot '-' matrix with image notitle, "
           << "'-' with vectors nohead lc rgb 'black' lw 4 notitle, "
           << "'-' with vectors nohead lc rgb 'white' lw 2 notitle;" << std::endl;
    for (int y = 0; y < MEC_SIZE; y++) {
        for (int x = 0; x < MEC_SIZE; x++) {
            int neuron_index = this->network->coords_to_neuron_index(x, y);
            stream << this->network->neurons[current_activity]->values[neuron_index] << " ";
        }
        stream << std::endl;
    }
    stream << std::endl << "e" << std::endl;
    for (int i = 0; i < 2; i++) {
        double x = this->network->bump_x;
        double y = this->network->bump_y;
        double direction = atan2(
            -this->network->bump_total_dy,
            -this->network->bump_total_dx);
        double length = sqrt(
            pow(this->network->bump_total_dx, 2) +
            pow(this->network->bump_total_dy, 2));

        do {
            double dx = length * cos(direction);
            double dy = length * sin(direction);

            double t[] = {
                dx >= 0 ? HUGE_VAL :          (- 0.5 - x) / dx, // Left
                dx <= 0 ? HUGE_VAL : (MEC_SIZE - 0.5 - x) / dx, // Right
                dy >= 0 ? HUGE_VAL :          (- 0.5 - y) / dy, // Bottom
                dy <= 0 ? HUGE_VAL : (MEC_SIZE - 0.5 - y) / dy, // Top
            };
            double min_t = HUGE_VAL;
            double segment_length = length;
            for (int t_index = 0; t_index < 4; t_index++) {
                if (t[t_index] >= 0 && t[t_index] < 1 && t[t_index] < min_t) {
                    min_t = t[t_index];
                    segment_length = t[t_index] * length;
                }
            }

            double segment_dx = segment_length * cos(direction);
            double segment_dy = segment_length * sin(direction);
            stream << x << " " << y << " " << segment_dx << " " << segment_dy << std::endl;

            length -= segment_length;
            x += segment_dx;
            y += segment_dy;
            for (int t_index = 0; t_index < 4; t_index++) {
                if (t[t_index] == min_t) {
                    switch (t_index) {
                    case 0: x += MEC_SIZE; break; // Hit left wall, jump to right wall
                    case 1: x -= MEC_SIZE; break; // Hit right wall, jump to left wall
                    case 2: y += MEC_SIZE; break; // Hit bottom wall, jump to top wall
                    case 3: y -= MEC_SIZE; break; // Hit top wall, jump to bottom wall
                    }
                }
            }
        } while (length > 0);
        stream << "e" << std::endl;
    }
    for (int i = 0; i < 4; i++) {
        stream << "unset object " << (i + 1) << ";" << std::endl;
    }
}
