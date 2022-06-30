// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include <getopt.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "model.h"
#include "simulation.h"
#include "plot.h"
#include "mec.h"
#include "main.h"

int usage(char *argv0)
{
    std::cerr << std::endl;
    std::cerr << "Usage: " << argv0 << " --modules=N --agent=A OPTIONS..." << std::endl;
    std::cerr << std::endl;
    std::cerr << "  --modules=N\t\tUse N grid modules (mandatory)." << std::endl;
    std::cerr << "  --agent=A\t\tUse A as the agent type (mandatory). Valid options:" << std::endl;
    std::cerr << "           \t\t  vector" << std::endl;
    std::cerr << "           \t\t  deflect" << std::endl;
    std::cerr << "           \t\t  combined" << std::endl;
    std::cerr << "           \t\t  narrow" << std::endl;
    std::cerr << "           \t\t  strict" << std::endl;
    std::cerr << "           \t\t  noresume" << std::endl;
    std::cerr << "           \t\t  notopo" << std::endl;
    std::cerr << "           \t\t  place" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  --script=S\t\tUse file S as the simulation script instead of stdin." << std::endl;
    std::cerr << "  --live-plot\t\tSend plots to the ./plot_pipe FIFO at regular intervals." << std::endl;
    std::cerr << "  --final-plot\t\tDump the final plot on stdout upon termination." << std::endl;
    std::cerr << "  --lite-plot\t\tLite version of the plot." << std::endl;
    std::cerr << "  --field-size=N\tUse N as the place field radius." << std::endl;
    return 1;
}

int main(int argc, char **argv)
{
    int getopt_simconf_live_plot = 0;
    int getopt_simconf_final_plot = 0;
    int getopt_simconf_lite_plot = 0;
    std::string getopt_agent_type;

    struct SimulationConf simconf = {
        .live_plot = false, // Will be overwritten to (bool)getopt_simconf_live_plot
        .final_plot = false, // Will be overwritten to (bool)getopt_simconf_final_plot
        .lite_plot = false, // Will be overwritten to (bool)getopt_simconf_lite_plot
        .script_source = "",
    };
    struct ModelConf modconf = {
        .module_count = 0,
        .gain_mode = gain_mode_poisson_neuron,
        .gain_ratio = 1.5,
        .initial_gain = MAX_MEC_GAIN,
        .alternative_motor_scaling = false,
        .simplified_mec_diff = false,
        .direction_samples = 28,
        .xy_samples = 9,
        .mec_diff_offset = 7,
        .sensor_count = 72,
        .sensor_range = 25.0,
        .place_cell_radius = 7.0,
        .internal_motor_tuning = 0.1,
    };

    struct option options[] = {
        { "live-plot", no_argument, &getopt_simconf_live_plot, 1 },
        { "final-plot", no_argument, &getopt_simconf_final_plot, 1 },
        { "lite-plot", no_argument, &getopt_simconf_lite_plot, 1 },

        { "modules", required_argument, nullptr, 1 },
        { "agent", required_argument, nullptr, 2 },
        { "script", required_argument, nullptr, 3 },
        { "field-size", required_argument, nullptr, 4 },

        { 0, 0, 0, 0 }
    };

    int option;
    while ((option = getopt_long(argc, argv, "", options, nullptr)) != -1) {
        switch (option) {
        case 1: modconf.module_count = std::stoi(optarg); break;
        case 2: getopt_agent_type = optarg; break;
        case 3: simconf.script_source = optarg; break;
        case 4: modconf.place_cell_radius = std::stod(optarg); break;
        }
    }

    simconf.live_plot = (bool)getopt_simconf_live_plot;
    simconf.final_plot = (bool)getopt_simconf_final_plot;
    simconf.lite_plot = (bool)getopt_simconf_lite_plot;

    if (modconf.module_count <= 0) {
        std::cerr << "Error: Module count (--modules=N) must be greater than zero." << std::endl;
        return usage(argv[0]);
    }

    Model *model = new Model(modconf);
    Agent *agent;

    if (getopt_agent_type == "vector") {
        agent = new VectorAgent(model);
    } else if (getopt_agent_type == "deflect") {
        agent = new DeflectAgent(model);
    } else if (getopt_agent_type == "combined") {
        agent = new CombinedAgent(model);
    } else if (getopt_agent_type == "narrow") {
        agent = new CombinedNarrowAgent(model);
    } else if (getopt_agent_type == "strict") {
        agent = new CombinedStrictAgent(model);
    } else if (getopt_agent_type == "noresume") {
        agent = new NoResumeCombinedStrictAgent(model);
    } else if (getopt_agent_type == "notopo") {
        agent = new NoTopoCombinedStrictAgent(model);
    } else if (getopt_agent_type == "place") {
        agent = new PlaceAgent(model);
    } else {
        std::cerr << "Error: Invalid agent type." << std::endl;
        return usage(argv[0]);
    }

    std::cerr << "Module count: " << modconf.module_count << std::endl;
    std::cerr << "Agent type: " << getopt_agent_type << std::endl;
    std::cerr << "Place field radius: " << modconf.place_cell_radius << std::endl;

    Simulation *simulation = new Simulation(agent, simconf);
    model->settle();
    return simulation->run();
}
