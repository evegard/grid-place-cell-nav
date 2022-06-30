// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "graph.h"

#include <cassert>
#include <cmath>
#include <deque>

#include "main.h"
#include "model.h"

PlaceCell::PlaceCell(int index, double x, double y)
    : index(index), x(x), y(y) {}

void PlaceCell::capture_grid_state_from_model(Model *model)
{
    for (int i = 0; i < model->conf.module_count; i++) {
        Vector *grid_module_copy = new Vector(
            model->mec_moving_convolved[i]->neurons[current_activity]->size);
        grid_module_copy->copy_from(
            model->mec_moving_convolved[i]->neurons[current_activity]);
        this->grid_state.push_back(grid_module_copy);
    }
}

void PlaceCell::transfer_grid_state_to_decoder(Model *model)
{
    assert(this->grid_state.size() == model->conf.module_count);
    for (int i = 0; i < model->conf.module_count; i++) {
        model->mec_fixed_convolved[i]->neurons[current_activity]->copy_from(
            this->grid_state[i]);
    }
}

void PlaceCell::weaken_neighbor(PlaceCell *neighbor)
{
    for (auto iter = this->neighbors.begin();
            iter != this->neighbors.end(); ++iter) {
        if ((*iter).first == neighbor) {
            if (--(*iter).second <= 0) {
                this->neighbors.erase(iter);
                break;
            }
        }
    }
}

double PlaceCell::distance(double x, double y)
{
    return std::sqrt(std::pow(x - this->x, 2) + std::pow(y - this->y, 2));
}

double PlaceCell::direction(double x, double y)
{
    return std::atan2(this->y - y, this->x - x);
}

PlaceGraph::PlaceGraph(double place_cell_radius) : place_cell_radius(place_cell_radius) {}

void PlaceGraph::update(Model *model)
{
    // First, "visit" the current location -- retrieve the place cell closest to
    // the current location, and if that cell is too far away, create a new one

    PlaceCell *closest_cell = nullptr;
    double closest_dist = HUGE_VAL;
    for (PlaceCell *current_cell : this->cells) {
        double current_dist = current_cell->distance(this->input.x, this->input.y);
        if (closest_cell == nullptr || current_dist < closest_dist) {
            closest_cell = current_cell;
            closest_dist = current_dist;
        }
    }
    if (this->input.form_place_cells && (
                closest_cell == nullptr ||
                closest_dist > 2 * this->place_cell_radius)) {
        PlaceCell *new_cell = new PlaceCell(this->cells.size(), this->input.x, this->input.y);
        new_cell->capture_grid_state_from_model(model);
        this->cells.push_back(new_cell);
        closest_cell = new_cell;
    }

    // Make sure there is a connection between the current place cell and the
    // previously visited one

    if (this->agent_cell && this->agent_cell != closest_cell) {
        bool already_connected = false;
        for (auto neighbor_strength_pair : closest_cell->neighbors) {
            PlaceCell *neighbor = neighbor_strength_pair.first;
            if (neighbor == this->agent_cell) {
                already_connected = true;
                break;
            }
        }
        if (!already_connected) {
            closest_cell->neighbors.push_back(
                std::make_pair(this->agent_cell, PLACE_CONNECTION_STRENGTH));
            this->agent_cell->neighbors.push_back(
                std::make_pair(closest_cell, PLACE_CONNECTION_STRENGTH));
        }
    }
    this->agent_cell = closest_cell;

    // Now that we have ensured there is an active place cell for the current
    // location, we can store this location for the current reward, if any

    if (this->input.save_reward) {
        assert(this->input.reward_id > 0);
        assert(this->agent_cell != nullptr);
        this->reward_locations[this->input.reward_id] = this->agent_cell;
    }

    // Check whether we want to weaken the last synapse crossed by the replay

    if (this->input.weaken_synapse) {
        if (this->replay_cell && this->replay_cell->replay_source) {
            this->replay_cell->weaken_neighbor(this->replay_cell->replay_source);
            this->replay_cell->replay_source->weaken_neighbor(this->replay_cell);
        }
    }

    // Clear the replay-related output signals, regardless of whether we do a
    // replay update in this timestep or not

    this->output.replay_terminated = false;

    // Check whether we've been requested to update the replay, whether it be
    // to reset the replay location, to propagate the replay location or both

    bool perform_replay_update = (
        this->input.reset_replay_to != maintain_current_node ||
        this->input.propagate_replay_towards != maintain_current_node);

    if (perform_replay_update) {
        // Make sure we have a reward cell for our ultimate goal location

        assert(this->input.reward_id > 0);
        assert(this->reward_locations.count(this->input.reward_id) != 0);
        this->reward_cell = this->reward_locations[this->input.reward_id];

        // Reset the replay location if requested to

        if (this->input.reset_replay_to != maintain_current_node) {
            this->replay_cell =
                (this->input.reset_replay_to == goal_node
                    ? this->reward_cell : this->agent_cell);
        }

        // If we've been requested to propagate the replay, we need to perform
        // a BFS towards the current replay location in order to set up the
        // predecessor pointers correcly

        if (this->input.propagate_replay_towards != maintain_current_node) {
            // The BFS will _start_ from the node in whose direction we want
            // the replay to propagate, so the logic here is in a sense "backwards"

            PlaceCell *bfs_start =
                (this->input.propagate_replay_towards == goal_node
                    ? this->reward_cell : this->agent_cell);
            PlaceCell *bfs_goal = this->replay_cell;

            // Perform breadth-first-search

            for (PlaceCell *cell : this->cells) {
                cell->bfs_predecessor = nullptr;
                cell->replay_source = nullptr;
            }
            bfs_start->bfs_predecessor = bfs_start;
            std::deque<PlaceCell *> fifo;
            fifo.push_back(bfs_start);
            while (fifo.size() > 0) {
                PlaceCell *current_bfs_cell = fifo[0];
                fifo.pop_front();
                for (auto neighbor_strength_pair : current_bfs_cell->neighbors) {
                    PlaceCell *neighbor = neighbor_strength_pair.first;
                    if (neighbor->bfs_predecessor == nullptr) {
                        fifo.push_back(neighbor);
                        neighbor->bfs_predecessor = current_bfs_cell;
                    }
                }
            }

            // If the BFS was able to reach the replay cell, then we have
            // somewhere to propagate

            if (this->replay_cell->bfs_predecessor != nullptr) {
                this->replay_cell->bfs_predecessor->replay_source = this->replay_cell;
                this->replay_cell = this->replay_cell->bfs_predecessor;

                // The replay "terminates" at this point if the new replay cell
                // is where the BFS originated, i.e. is the endpoint we wanted
                // the replay to propagate towards

                this->output.replay_terminated =
                    (this->replay_cell == this->replay_cell->bfs_predecessor);
            } else {
                // The BFS didn't reach the replay cell, so the replay is terminated
                this->output.replay_terminated = true;
            }
        }

        // Make the subgoal cell project its grid state back to the grid decoder
        this->replay_cell->transfer_grid_state_to_decoder(model);
    }

    // Update the output variables indicating whether we have currently reached
    // the goal and/or the subgoal location

    this->output.at_goal = (this->reward_cell != nullptr) &&
        (this->reward_cell->distance(this->input.x, this->input.y) <= this->place_cell_radius);
    this->output.at_subgoal = (this->replay_cell != nullptr) &&
        (this->replay_cell->distance(this->input.x, this->input.y) <= this->place_cell_radius);
    this->output.subgoal_visible = (this->replay_cell != nullptr) &&
        (this->replay_cell->distance(this->input.x, this->input.y) <= 3 * this->place_cell_radius);
    this->output.subgoal_direction = (!this->output.subgoal_visible ? 0.0 :
        this->replay_cell->direction(this->input.x, this->input.y));
}

void PlaceGraph::plot_place_cells(std::ostream &stream)
{
    stream << "# Start of place graph" << std::endl;
    for (PlaceCell *cell : this->cells) {
        stream << "set object circle "
            << "center " << cell->x << "," << cell->y << " "
            << "size " << this->place_cell_radius << " "
            << "fill empty border "
            << (cell == this->replay_cell
                    ? "lc rgb 'red' lw 3"
                    : "lc rgb 'dark-gray'") << ";" << std::endl;
        for (auto neighbor_strength_pair : cell->neighbors) {
            PlaceCell *other_cell = neighbor_strength_pair.first;
            // Compare the pointer address to only emit one line per pair
            if (cell < other_cell) {
                stream << "set arrow nohead from "
                    << cell->x << "," << cell->y << " to "
                    << other_cell->x << "," << other_cell->y << " "
                    << "lw 1 lc rgb 'dark-gray';" << std::endl;
            }
        }
    }
    stream << "# End of place graph" << std::endl;
}
