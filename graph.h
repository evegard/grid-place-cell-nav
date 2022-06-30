// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED

#include "numerical.h"

#include <map>
#include <ostream>
#include <tuple>
#include <utility>
#include <vector>

enum ReplayMode { maintain_current_node, goal_node, agent_node };

class Model;

class PlaceCell
{
    public:
        PlaceCell(int index, double x, double y);
        void capture_grid_state_from_model(Model *model);
        void transfer_grid_state_to_decoder(Model *model);
        void weaken_neighbor(PlaceCell *neighbor);
        double distance(double x, double y);
        double direction(double x, double y);

        int index;
        double x, y;
        std::vector<std::pair<PlaceCell *, int>> neighbors;
        PlaceCell *bfs_predecessor = nullptr;
        PlaceCell *replay_source = nullptr;
        std::vector<Vector *> grid_state;
};

class PlaceGraph
{
    public:
        PlaceGraph(double place_cell_radius);
        void update(Model *model);

        // Input variables

        struct {
            double x, y;
            int reward_id = 0;
            bool save_reward = false;

            bool form_place_cells = true;
            bool weaken_synapse = false;
            ReplayMode reset_replay_to = maintain_current_node;
            ReplayMode propagate_replay_towards = maintain_current_node;
        } input;

        // Output variables

        struct {
            bool at_goal = false; // FIXME: simulation.cc should keep track of this itself
            bool subgoal_visible = false;
            double subgoal_direction = 0.0;

            bool at_subgoal = false;
            bool replay_terminated = false;
        } output;

        // Internal

        std::vector<PlaceCell *> cells;
        std::map<int, PlaceCell *> reward_locations;
        PlaceCell *agent_cell = nullptr;
        PlaceCell *reward_cell = nullptr; // FIXME: Shouldn't be necessary, re at_goal above
        PlaceCell *replay_cell = nullptr;
        double place_cell_radius;

        void plot_place_cells(std::ostream &stream);
};

#endif
