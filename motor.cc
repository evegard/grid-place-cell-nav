// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "motor.h"

#include <cassert>
#include <cmath>

#include "mecdiff.h"

MotorNetwork::MotorNetwork(int direction_samples, double scaling_factor, bool normalize)
    : Network(direction_samples), direction_samples(direction_samples),
      scaling_factor(scaling_factor), normalize(normalize)
{
    // Flip current_activity and next_activity. current_activity starts with
    // random initial conditions, whereas next_activity is zeroed. We want the
    // latter for the motor neurons.
    this->commit();
}

void MotorNetwork::commit()
{
    Network::commit();
    std::tie(this->direction, this->strength) =
        this->calculate_direction_and_strength(current_activity);
}

std::tuple<double, double> MotorNetwork::calculate_direction_and_strength(NeuronActivity activity)
{
    double x = 0.0, y = 0.0;
    for (int i = 0; i < this->direction_samples; i++) {
        double value = this->neurons[activity]->values[i];
        double angle = i * 2 * M_PI / direction_samples;
        x += value * std::cos(angle);
        y += value * std::sin(angle);
    }
    return std::make_tuple(
        std::atan2(y, x),
        std::sqrt(std::pow(x, 2) + std::pow(y, 2)));

}

void MotorNetwork::update_neuron_values()
{
    for (int i = 0; i < this->direction_samples; i++) {
        this->neurons[next_activity]->values[i] = this->neuron_inputs->values[i];
        if (this->neurons[next_activity]->values[i] < 0.0) {
            this->neurons[next_activity]->values[i] = 0.0;
        }
    }
    if (this->normalize) {
        double final_direction, final_strength;
        std::tie(final_direction, final_strength) =
            this->calculate_direction_and_strength(next_activity);
        if (this->override_active) {
            final_direction = this->override_direction;
            final_strength = this->override_strength;
        }
        final_strength = (final_strength > 0.0 ? 1.0 : 0.0);

        double peak_activation = 0.0;
        for (int i = 0; i < this->direction_samples; i++) {
            double direction = i * 2 * M_PI / direction_samples;
            double direction_difference = std::atan2(
                std::sin(direction - final_direction),
                std::cos(direction - final_direction));
            this->neurons[next_activity]->values[i] = final_strength * std::exp(
                -pow(direction_difference, 2) / (2 * pow(this->normalization_spread, 2)));
            peak_activation = MAX(peak_activation,
                this->neurons[next_activity]->values[i]);
        }
        double rescaling = peak_activation > 0.0 ? this->normalization_peak / peak_activation : 0.0;
        for (int i = 0; i < this->direction_samples; i++) {
            this->neurons[next_activity]->values[i] *= rescaling;
        }
    }
}

MecDiffMotorInput::MecDiffMotorInput(
        MotorNetwork *motor_network, MecDiffNetwork *mec_diff_network)
    : Input(motor_network), motor_network(motor_network),
      mec_diff_network(mec_diff_network)
{
    assert(motor_network->direction_samples == mec_diff_network->direction_samples);
}

void MecDiffMotorInput::add_inputs()
{
    for (int y = 0; y < this->mec_diff_network->xy_samples; y++) {
        for (int x = 0; x < this->mec_diff_network->xy_samples; x++) {
            for (int direction = 0; direction < this->mec_diff_network->direction_samples; direction++) {
                int index = this->mec_diff_network->neuron_index(direction, x, y);
                auto value = this->mec_diff_network->neurons[current_activity]->values[index];
                this->motor_network->neuron_inputs->values[direction] += value;
            }
        }
    }
}

MotorMotorInput::MotorMotorInput(
        MotorNetwork *efferent, MotorNetwork *afferent)
    : Input(efferent), efferent(efferent), afferent(afferent)
{
    assert(efferent->direction_samples == afferent->direction_samples);
}

void MotorMotorInput::add_inputs()
{
    for (int direction = 0; direction < this->efferent->direction_samples; direction++) {
        this->efferent->neuron_inputs->values[direction] +=
            this->afferent->neurons[current_activity]->values[direction] * this->afferent->scaling_factor;
    }
}

BorderMotorInput::BorderMotorInput(MotorNetwork *efferent, Vector *border_sensors)
    : Input(efferent), efferent(efferent), border_sensors(border_sensors)
{
}

void BorderMotorInput::add_inputs()
{
    for (int direction = 0; direction < this->efferent->direction_samples; direction++) {
        this->efferent->neuron_inputs->values[direction] -= this->border_sensors->values[direction];
    }
}

MotorNetworkPlot::MotorNetworkPlot(MotorNetwork *network,
        const char *color, const char *title, bool simplified, double plot_range)
    : network(network), color(color), simplified(simplified), plot_range(plot_range)
{
    this->set("polar", "");
    this->set("size", "square");
    std::string rrange;
    rrange += "[0:";  rrange += std::to_string(plot_range); rrange += "]";
    this->set("rrange", rrange.c_str());
    this->set("zeroaxis", "");
    this->set("margins", "0,0,0,0");
    this->unset("xtics");
    this->unset("ytics");
    this->unset("rtics");
    this->unset("raxis");
    this->unset("border");
    if (title != nullptr) {
        this->set("ylabel", title);
    }
}

void MotorNetworkPlot::dump_plot_commands(std::ostream &stream)
{
    double direction = this->network->direction;
    double strength = this->network->strength;
    const char *arrow_properties = "";
    if (this->simplified && strength > 0) {
        strength = this->plot_range;
    } else if (strength > this->plot_range) {
        strength = this->plot_range;
        arrow_properties = "nohead";
    }
    stream << "set arrow 1 length " << strength
        << " angle " << (direction * 180 / M_PI)
        << " front lw 2 " << arrow_properties << ";" << std::endl;
    stream << "set object 1 circle at 0,0 size " << this->plot_range
        << " fill empty border lc rgb 'black';" << std::endl;

    if (this->simplified) {
        stream << "plot 1/0 notitle;" << std::endl;
    } else {
        stream << "plot '-' with filledcurves above r=0 lc rgb '"
            << this->color << "' notitle;" << std::endl;
        for (int i = 0; i < this->network->direction_samples + 1; i++) {
            // Loop to (directions+1), i.e. wrap around, to get the polar line
            // connected back to itself at 360/0 degrees
            double value = this->network->neurons[current_activity]->
                values[i % this->network->direction_samples];
            double angle = i * 2 * M_PI / this->network->direction_samples;
            stream << angle << " " << value << std::endl;
        }
        stream << std::endl << "e" << std::endl;
    }

    stream << "unset arrow 1;" << std::endl;
    stream << "unset object 1;" << std::endl;
}
