// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "model.h"

#include "mecdiff.h"
#include "mec.h"
#include "motor.h"
#include "numerical.h"

#include <iostream>

Model::Model(struct ModelConf conf)
    : conf(conf)
{
    this->final_motor = new MotorNetwork(this->conf.direction_samples, 1.0, false);

    for (int i = 0; i < this->conf.module_count; i++) {
        real current_gain = this->conf.initial_gain / pow(this->conf.gain_ratio, i);

        this->mec_fixed.push_back(new MecNetwork(current_gain, this->conf.gain_mode));
        this->mec_moving.push_back(new MecNetwork(current_gain, this->conf.gain_mode));
        this->mec_fixed_convolved.push_back(new ConvolvedMecNetwork(this->mec_fixed[i]));
        this->mec_moving_convolved.push_back(new ConvolvedMecNetwork(this->mec_moving[i]));

        this->velocity_inputs.push_back(new VelocityInput(this->mec_moving[i]));
        this->mec_moving[i]->add_input(this->velocity_inputs[i]);

        this->mec_diff.push_back(new MecDiffNetwork(this->conf.simplified_mec_diff,
            this->mec_moving_convolved[i], this->mec_fixed_convolved[i],
            this->conf.direction_samples, this->conf.xy_samples, this->conf.mec_diff_offset));

        // Calculating motor scaling factors for the modules, we want the
        // factor for the largest-scaled grid module (i == conf.module_count - 1)
        // to be 1.0. To simplify this logic, we use the variable n that counts
        // in the opposite direction of i, so that (n == 0) corresponds to the
        // largest-scaled module and (n == conf.module_count - 1) to (i == 0).
        int n = this->conf.module_count - 1 - i;
        real motor_scaling_factor;
        if (this->conf.alternative_motor_scaling) {
            // Due to Stemmler et al. (2015), Sci. Adv.
            real denominator = 0.0;
            for (int l = 0; l <= n; l++) {
                denominator += pow(this->conf.gain_ratio, 2 * l);
            }
            motor_scaling_factor = pow(this->conf.gain_ratio, n) / denominator;
        } else {
            motor_scaling_factor = 1.0 / pow(this->conf.gain_ratio, n);
        }

        this->mec_motor.push_back(new MotorNetwork(
            this->conf.direction_samples, motor_scaling_factor, false));
        this->mec_motor[i]->add_input(new MecDiffMotorInput(
            this->mec_motor[i], this->mec_diff[i]));
        this->final_motor->add_input(new MotorMotorInput(
            this->final_motor, this->mec_motor[i]));
    }

    this->place_graph = new PlaceGraph(this->conf.place_cell_radius);
    this->border_sensors = new Vector(this->conf.sensor_count);

    this->first_normalized_motor = new MotorNetwork(this->conf.sensor_count, 1.0, true);
    this->first_inhibited_motor = new MotorNetwork(this->conf.sensor_count, 1.0, false);
    this->second_normalized_motor = new MotorNetwork(this->conf.sensor_count, 1.0, true);
    this->second_inhibited_motor = new MotorNetwork(this->conf.sensor_count, 1.0, false);

    this->first_inhibited_motor->add_input(new MotorMotorInput(
        this->first_inhibited_motor, this->first_normalized_motor));
    this->second_normalized_motor->add_input(new MotorMotorInput(
        this->second_normalized_motor, this->first_inhibited_motor));
    this->second_inhibited_motor->add_input(new MotorMotorInput(
        this->second_inhibited_motor, this->second_normalized_motor));

    this->first_border_motor_input = this->first_inhibited_motor->add_input(
        new BorderMotorInput(this->first_inhibited_motor, this->border_sensors));
    this->second_border_motor_input = this->second_inhibited_motor->add_input(
        new BorderMotorInput(this->second_inhibited_motor, this->border_sensors));
}

void Model::settle()
{
    for (int i = 0; i < this->conf.module_count; i++) {
        MecGainMode previous_gain_mode = this->mec_moving[i]->gain_mode;
        this->mec_moving[i]->gain_mode = gain_mode_velocity;
        for (int t = 0; t < SETTLE_STEPS; t++) {
            this->mec_moving[i]->update();
            this->mec_moving[i]->commit();
        }
        this->mec_moving[i]->gain_mode = previous_gain_mode;
        this->mec_moving_convolved[i]->update();
        this->mec_moving_convolved[i]->commit();
        this->mec_moving_convolved[i]->initialize_bump_tracker();
    }

    for (int i = 0; i < this->conf.module_count; i++) {
        this->mec_fixed_convolved[i]->neurons[current_activity]->copy_from(
            this->mec_moving_convolved[i]->neurons[current_activity]);
        this->mec_moving_convolved[i]->initialize_bump_tracker();
    }

    this->first_normalized_motor->override_active = true;
    this->first_normalized_motor->override_direction = 0.0;
    this->first_normalized_motor->override_strength = 0.0;

    this->first_normalized_motor->update_and_commit();
    this->first_inhibited_motor->update_and_commit();
    this->second_normalized_motor->update_and_commit();
    this->second_inhibited_motor->update_and_commit();
}

void Model::simulate_timestep()
{
    for (int i = 0; i < this->conf.module_count; i++) {
        this->velocity_inputs[i]->set_velocity(
            this->input.speed * std::cos(this->input.heading),
            this->input.speed * std::sin(this->input.heading));
        this->mec_moving[i]->update_and_commit();
        this->mec_moving_convolved[i]->update_and_commit();
        this->mec_moving_convolved[i]->update_bump_tracker();
    }

    this->place_graph->update(this);

    if (this->input.motor_mode == grid_decoder_mode) {
        for (int i = 0; i < this->conf.module_count; i++) {
            this->mec_diff[i]->update_and_commit();
            this->mec_motor[i]->update_and_commit();
        }
        this->final_motor->update_and_commit();
    }

    this->output.halted = true;
    this->output.heading = this->input.heading;
    if (this->input.motor_mode != halt_mode) {
        if (this->input.motor_mode == grid_decoder_mode) {
            if (this->place_graph->output.subgoal_visible) {
                this->first_normalized_motor->override_direction = this->place_graph->output.subgoal_direction;
                this->first_normalized_motor->override_strength = 1.0;
            } else {
                this->first_normalized_motor->override_direction = this->final_motor->direction;
                this->first_normalized_motor->override_strength = this->final_motor->strength;
            }
        } else if (this->input.motor_mode == last_heading_mode) {
            this->first_normalized_motor->override_direction = this->input.heading;
            this->first_normalized_motor->override_strength = 1.0;
        } else if (this->input.motor_mode == forced_mode) {
            this->first_normalized_motor->override_direction = 0.0;
            this->first_normalized_motor->override_strength = 1.0;
        }
        this->first_normalized_motor->override_direction += this->input.motor_offset;

        bool border_cells_active = (this->input.motor_mode != forced_mode);
        this->first_border_motor_input->set_active(border_cells_active);
        this->second_border_motor_input->set_active(border_cells_active);

        this->first_normalized_motor->normalization_spread = this->input.motor_tuning;
        this->second_normalized_motor->normalization_spread = this->conf.internal_motor_tuning;

        this->first_normalized_motor->update_and_commit();
        this->first_inhibited_motor->update_and_commit();
        this->second_normalized_motor->update_and_commit();
        this->second_inhibited_motor->update_and_commit();

        if (this->first_normalized_motor->strength > 0.0 &&
                this->second_normalized_motor->strength > 0.0) {
            this->confidence = std::sqrt(
                this->first_inhibited_motor->strength /
                this->first_normalized_motor->strength *
                this->second_inhibited_motor->strength /
                this->second_normalized_motor->strength);
        } else {
            this->confidence = 0.0;
        }

        this->output.halted = (this->confidence < this->input.confidence_threshold);
        if (this->second_inhibited_motor->strength > 0.0) {
            this->output.heading = this->second_inhibited_motor->direction;
        }
    }

    this->output.speed = this->output.halted ? 0.0 : FIXED_SPEED;
}

VelocityInput::VelocityInput(MecNetwork *efferent)
    : Input(efferent), efferent(efferent), velocity_x(0.0), velocity_y(0.0)
{
}

void VelocityInput::set_velocity(real x, real y)
{
    this->velocity_x = x;
    this->velocity_y = y;
}

void VelocityInput::add_inputs()
{
    for (int y = 0; y < MEC_SIZE; y++) {
        for (int x = 0; x < MEC_SIZE; x++) {
            real contribution = 0.0;
            switch (this->efferent->directionality(x, y)) {
            case north: contribution = this->velocity_y; break;
            case south: contribution = -this->velocity_y; break;
            case east: contribution = this->velocity_x; break;
            case west: contribution = -this->velocity_x; break;
            }
            if (this->efferent->gain_mode == gain_mode_velocity) {
                contribution *= this->efferent->gain;
            } else {
                contribution *= MAX_MEC_GAIN;
            }
            contribution *= 0.10315;

            int neuron_index = this->efferent->coords_to_neuron_index(x, y);
            this->efferent->neuron_inputs->values[neuron_index] += contribution;
        }
    }
}

AllMotorsPlot::AllMotorsPlot(Model *model)
    : model(model)
{
    this->set("size", "square");
    this->set("zeroaxis", "");
    this->set("margins", "0,0,0,0");
    this->set("xrange", "[-" STRINGIFY_CONSTANT(ALL_MOTORS_PLOT_RANGE)
        ":" STRINGIFY_CONSTANT(ALL_MOTORS_PLOT_RANGE) "]");
    this->set("yrange", "[-" STRINGIFY_CONSTANT(ALL_MOTORS_PLOT_RANGE)
        ":" STRINGIFY_CONSTANT(ALL_MOTORS_PLOT_RANGE) "]");
    this->unset("xtics");
    this->unset("ytics");
    this->unset("border");
    this->set("ylabel", "\"{/=14 Final goal direction as}\\n{/=14 decoded from grid cells}\"");
    this->set("object 1", "circle at 0,0 size " STRINGIFY_CONSTANT(ALL_MOTORS_PLOT_RANGE)
        " noclip fill empty border lc rgb 'black'");
}

void AllMotorsPlot::dump_plot_commands(std::ostream &stream)
{
    stream << "plot ";
    for (int i = this->model->conf.module_count - 1; i >= 0; i--) {
        stream << "'-' with line lw 4 lt " << (i + 1) << " notitle, ";
    }
    stream << "1/0 notitle;" << std::endl;

    real current_x = 0.0, current_y = 0.0;
    for (int i = this->model->conf.module_count - 1; i >= 0; i--) {
        real direction = this->model->mec_motor[i]->direction;
        real strength = this->model->mec_motor[i]->strength * this->model->mec_motor[i]->scaling_factor;
        real next_x = current_x + strength * cos(direction);
        real next_y = current_y + strength * sin(direction);
        stream << current_x << " " << current_y << std::endl
               << next_x << " " << next_y << std::endl
               << "e" << std::endl;
        current_x = next_x;
        current_y = next_y;
    }
}

BorderSensorsPlot::BorderSensorsPlot(Vector *border_sensors)
    : border_sensors(border_sensors)
{
    this->set("polar", "");
    this->set("size", "square");
    this->set("rrange", "[0:" STRINGIFY_CONSTANT(UI_MOTOR_PLOT_RANGE) "]");
    this->set("zeroaxis", "");
    this->set("margins", "0,0,0,0");
    this->unset("xtics");
    this->unset("ytics");
    this->unset("rtics");
    this->unset("raxis");
    this->unset("border");
    this->set("ylabel", "\"{/=14 Border cells that}\\n{/=14 inhibit motor cells}\"");
    this->set("object 1", "circle at 0,0 size " STRINGIFY_CONSTANT(UI_MOTOR_PLOT_RANGE) " noclip fill empty border lc rgb 'black'");
}

void BorderSensorsPlot::dump_plot_commands(std::ostream &stream)
{
    stream << "plot '-' with filledcurves above r=0 lc rgb 'blue' notitle;" << std::endl;
    for (int i = 0; i < this->border_sensors->size + 1; i++) {
        // Loop to (size+1), i.e. wrap around, to get the polar line
        // connected back to itself at 360/0 degrees
        double value = this->border_sensors->values[i % this->border_sensors->size];
        double angle = i * 2 * M_PI / this->border_sensors->size;
        stream << angle << " " << value << std::endl;
    }
    stream << std::endl << "e" << std::endl;
}
