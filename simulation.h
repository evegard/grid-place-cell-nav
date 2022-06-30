// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef SIMULATION_H_INCLUDED
#define SIMULATION_H_INCLUDED

#include "agent.h"
#include "arena.h"
#include "graph.h"
#include "main.h"
#include "mec.h"
#include "numerical.h"

#include <vector>
#include <map>
#include <string>

class Simulation
{
    friend class SimulationPlot;
    friend class SimulationArenaComponentPlot;

    public:
        Simulation(Agent *agent, struct SimulationConf conf);
        int run();

    protected:
        // Parameters given to the constructor
        Agent *agent;
        struct SimulationConf conf;

        // Function called on each timestep to update model and simulation
        bool step();

        // Main simulation values. All of these are initialized in run()
        int global_timestep;
        double x, y, heading, speed;

        // For the new version of the simulator
        Arena *arena;
        int reward_id;
        double goto_x, goto_y;
        std::map<std::string, int> reward_ids;
        std::istream *script = nullptr;

        int get_reward_id(std::string reward_name);

        std::string current_trial_phase;
        double path_length_in_current_trial_phase = 0.0;
        void report_path_length_at_end_of_trial_phase();
        std::map<std::string, Arena *> fences;

        // Plotting
        class SimulationPlot *plot = nullptr;
};

#endif
