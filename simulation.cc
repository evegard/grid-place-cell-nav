// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "simulation.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>

#include "mec.h"
#include "mecdiff.h"
#include "model.h"
#include "motor.h"
#include "numerical.h"
#include "arena.h"
#include "ui.h"

Simulation::Simulation(Agent *agent, struct SimulationConf conf)
    : agent(agent), conf(conf)
{
    this->arena = Arena::load_arena("MULTIPOLYGON()");
    this->plot = new SimulationPlot(this, conf.lite_plot);
    this->plot->plot_sink = conf.live_plot ? pipe_plot_sink : stdout_plot_sink;
    if (conf.script_source == "") {
        this->script = &std::cin;
    } else {
        this->script = new std::fstream(conf.script_source, std::ios::in);
    }
}

bool Simulation::step()
{
    // If the agent just performed a state transition, we potentially want to
    // plot this location for this transition
    if (this->agent->active_state != this->agent->previous_state) {
        this->plot->report_agent_state_transition(this->x, this->y,
            this->agent->previous_state, this->agent->active_state);
    }

    // Update border sensor inputs to the model
    this->arena->update_sensors(this->x, this->y,
        this->agent->model->conf.sensor_range,
        this->agent->model->border_sensors->values,
        this->agent->model->border_sensors->size);

    // Update inputs to the agent and execute the current agent state (which in
    // turn invokes a timestep update of the model)
    this->agent->input = {
        .x = this->x,
        .y = this->y,
        .heading = this->heading,
        .speed = this->speed,
        .goto_x = this->goto_x,
        .goto_y = this->goto_y,
        .reward_id = this->reward_id,
    };
    this->agent->execute();

    this->heading = Periodic::double_modulo(this->agent->output.heading, 2 * M_PI);
    this->speed = this->agent->output.speed;

    // Add the current agent and replay place cells to the "raster plot"
    this->plot->report_place_cell(agent_raster,
        this->global_timestep, this->agent->model->place_graph->agent_cell);
    this->plot->report_place_cell(replay_raster,
        this->global_timestep, this->agent->model->place_graph->replay_cell);

    // Update the plot if the number of timesteps since the current simulation
    // phase started is a multiple of PLOT_UPDATE_INTERVAL, or if this is the
    // first timestep in a new agent state, or if the agent state is
    // replay_episode_state, which is a loop we want to show
    if (this->global_timestep % PLOT_UPDATE_INTERVAL == 0 ||
            this->agent->previous_state != this->agent->next_previous_state ||
            this->agent->previous_state == replay_episode_state) {
        this->plot->append_trajectory(this->x, this->y, false);
        if (this->conf.live_plot) {
            this->plot->show();
        }
    }

    // Update path length
    this->path_length_in_current_trial_phase += this->speed / STEPS_PER_SECOND;

    // Update ground truth coordinates
    double ax = this->x, ay = this->y;
    this->x += this->speed * cos(this->heading) / STEPS_PER_SECOND;
    this->y += this->speed * sin(this->heading) / STEPS_PER_SECOND;
    double bx = this->x, by = this->y;

    // Increment timestep counters
    this->global_timestep++;

    // Continue current simulation loop if agent still has an active state
    bool continue_loop = (this->agent->active_state != no_state);

    // Check for any fence intersections; end loop if hit
    for (auto iter = this->fences.begin(); iter != this->fences.end(); iter++) {
        if (iter->second->line_intersects(ax, ay, bx, by)) {
            std::cerr << "Agent hit fence \"" << iter->first << "\"" << std::endl;
            continue_loop = false;
        }
    }

    // Check for arena intersection; quit if hit
    if (this->arena->line_intersects(ax, ay, bx, by)) {
        std::cerr << "Agent hit arena between " << ax << "," << ay << " "
            << "and " << bx << "," << by << "!" << std::endl;
        exit(1);
    }

    return continue_loop;
}

int Simulation::run()
{
    // Set up initial values for simulation variables

    this->global_timestep = 0;

    this->x = 0.0;
    this->y = 0.0;
    this->heading = 0.0;
    this->speed = 0.0;
    this->reward_id = 0;

    // Read simulation commands from stdin until done

    std::string command, last_command;
    int repetitions = 1;
    while ((*this->script) >> command) {
        if (command == last_command) {
            std::cerr << "\033[F\033[K";
        } else {
            repetitions = 1;
        }
        std::cerr << "Running " << command;
        if (repetitions > 1) {
            std::cerr << " (" << repetitions << "x)";
        }
        std::cerr << std::endl;
        if (command == "goto") {
            (*this->script) >> this->goto_x >> this->goto_y;
            double goto_distance = std::sqrt(
                std::pow(this->goto_x - this->x, 2) +
                std::pow(this->goto_y - this->y, 2));
            if (goto_distance >= DISTANCE_PER_TIMESTEP) {
                this->agent->active_state = forced_move_state;
                while (this->step());
            }
        } else if (command == "place-agent") {
            (*this->script) >> this->x >> this->y >> this->heading;
        } else if (command == "trigger-reward") {
            std::string reward_name;
            (*this->script) >> reward_name;
            this->reward_id = this->get_reward_id(reward_name);
            this->agent->active_state = receive_reward_state;
            while (this->step());
            this->reward_id = 0;
        } else if (command == "seek-reward") {
            std::string reward_name;
            (*this->script) >> reward_name;
            int timestep_limit;
            (*this->script) >> timestep_limit;
            this->reward_id = this->get_reward_id(reward_name);
            this->agent->active_state = initiate_navigation_state;

            this->plot->report_endpoint_location(start_endpoint, this->x, this->y);
            while (timestep_limit-- > 0 && this->step() &&
                !this->agent->model->place_graph->output.at_goal);
            this->plot->report_endpoint_location(end_endpoint, this->x, this->y);

            std::cerr << "Successful in reaching reward \"" << reward_name << "\"? "
                << (this->agent->model->place_graph->output.at_goal ? "YES" : "NO") << std::endl;
            PlaceCell *reward_cell = this->agent->model->place_graph->
                reward_locations[this->reward_id];
            std::cerr << "(Final distance to reward \"" << reward_name << "\" was "
                << std::sqrt(
                    std::pow(this->x - reward_cell->x, 2) +
                    std::pow(this->y - reward_cell->y, 2))
                << ")" << std::endl;;

            this->reward_id = 0;
        } else if (command == "set-arena") {
            std::string wkt_string;
            std::getline(*this->script, wkt_string);
            this->arena = Arena::load_arena(wkt_string.c_str());
            this->plot->update_arena();
        } else if (command == "set-trial-phase") {
            // The current coordinates are the final ones for the last trajectory
            this->plot->append_trajectory(this->x, this->y, true);

            std::string phase_color, phase_title;
            (*this->script) >> phase_color;
            std::getline(*this->script, phase_title);
            if (phase_title[0] == ' ') {
                phase_title = phase_title.substr(1);
            }
            this->plot->new_trajectory(phase_color, phase_title);

            this->report_path_length_at_end_of_trial_phase();
            this->path_length_in_current_trial_phase = 0.0;
            this->current_trial_phase = phase_title;

            // The current coordinates are also the inital ones for the new trajectory
            this->plot->append_trajectory(this->x, this->y, false);
        } else if (command == "set-title") {
            std::string plot_title;
            std::getline(*this->script, plot_title);
            this->plot->set_title(plot_title);
        } else if (command == "set-origin") {
            this->plot->update_origin(this->x, this->y);
        } else if (command == "set-arena-size") {
            double arena_size;
            (*this->script) >> arena_size;
            this->plot->set_arena_size(arena_size);
        } else if (command == "set-scale-bars") {
            int scale_bars;
            (*this->script) >> scale_bars;
            this->plot->set_scale_bars(scale_bars);
        } else if (command == "add-label") {
            double label_x, label_y;
            std::string label_text;
            (*this->script) >> label_x >> label_y;
            std::getline(*this->script, label_text);
            if (label_text[0] == ' ') {
                label_text = label_text.substr(1);
            }
            this->plot->add_label(label_x, label_y, label_text);
        } else if (command == "set-fence") {
            std::string fence_name, fence_wkt;
            (*this->script) >> fence_name;
            std::getline(*this->script, fence_wkt);
            this->fences[fence_name] = Arena::load_arena(fence_wkt.c_str());
        } else {
            std::cerr << "Unknown script command "
                << "\"" << command << "\"!" << std::endl;
            return 1;
        }
        last_command = command;
        repetitions++;
    }
    // Make sure to save the current coordinates as the final
    // coordinates for the current trajectory
    this->plot->append_trajectory(this->x, this->y, true);
    if (this->conf.live_plot || this->conf.final_plot) {
        this->plot->show();
    }
    this->report_path_length_at_end_of_trial_phase();
    return 0;
}

int Simulation::get_reward_id(std::string reward_name)
{
    if (this->reward_ids.count(reward_name) == 0) {
        int reward_id = this->reward_ids.size() + 1;
        this->reward_ids[reward_name] = reward_id;
    }
    return this->reward_ids[reward_name];
}

void Simulation::report_path_length_at_end_of_trial_phase()
{
    if (this->current_trial_phase == "") {
        return;
    }
    std::cerr << "Path length at end of \"" << this->current_trial_phase << "\": "
        << this->path_length_in_current_trial_phase << std::endl;
}
