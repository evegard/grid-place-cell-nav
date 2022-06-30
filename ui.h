// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef UI_H_INCLUDED
#define UI_H_INCLUDED

#include <ostream>
#include <string>
#include <map>
#include <tuple>

#include "plot.h"
#include "simulation.h"
#include "arena.h"

enum Endpoint { start_endpoint, end_endpoint, ENDPOINT_COUNT };
enum Raster { agent_raster, replay_raster, RASTER_COUNT };

class SimulationArenaComponentPlot : public ComponentPlot
{
    public:
        SimulationArenaComponentPlot(Simulation *simulation, double right_border);
        void update_arena(Arena *arena);
        void dump_plot_commands(std::ostream &stream);
        void new_trajectory(std::string color, std::string title);
        void append_trajectory(double x, double y, bool trajectory_finished);
        void report_agent_state_transition(double x, double y, State from_state, State to_state);
        void report_endpoint_location(Endpoint endpoint, double x, double y);
        void set_arena_size(double size);
        void set_scale_bars(int scale_bars);
        void add_label(double label_x, double label_y, std::string label_text);

        std::string plot_title;

    protected:
        Simulation *simulation;
        PlotComponent *current_trajectory = nullptr;
        std::map<std::tuple<State, State>, PlotComponent *> agent_state_transitions;
        PlotComponent *endpoint_locations[ENDPOINT_COUNT] = { nullptr };
        double arena_size = 200.0;
        int scale_bars = 10;
        double right_border;
        double lo_bound = 1, hi_bound = 200.0 - 1;
        bool point_components_initialized = false;
        bool inverted_arena_rendering = false;
        std::vector<std::tuple<double, double, std::string>> labels;

        void plot_arena_polygons(std::ostream &stream);
        void plot_arena_labels(std::ostream &stream);
        void plot_agent(std::ostream &stream,
            double x, double y, double direction);
        void plot_status(std::ostream &stream, double row,
            const char *title, const char *value);
};

class PolarArenaComponentPlot : public ComponentPlot
{
    public:
        PolarArenaComponentPlot();
        void update_arena(Arena *arena);
        void set_origin(double x, double y);
        void new_trajectory(std::string color, std::string title);
        void append_trajectory(double x, double y);
        void report_endpoint_location(Endpoint endpoint, double x, double y);
        void set_arena_size(double size);

    protected:
        PlotComponent *arena_lines;
        PlotComponent *current_trajectory;
        PlotComponent *endpoint_locations[ENDPOINT_COUNT] = { nullptr };
        bool point_components_initialized = false;
        double origin_x = 0.0, origin_y = 0.0;
        bool first_point = true;
        double last_x = 0.0, last_y = 0.0;
        double arena_size = 200.0;
};

class RasterPlot : public ComponentPlot
{
    public:
        RasterPlot();
        void report_place_cell(Raster raster, int timestep, PlaceCell *place_cell);

    protected:
        PlotComponent *rasters[RASTER_COUNT] = { nullptr };
        int last_index[RASTER_COUNT] = { -1 };
};

class SimulationPlot : public MultiPlot
{
    public:
        SimulationPlot(Simulation *simulation, bool lite);
        void new_trajectory(std::string color, std::string title);
        void append_trajectory(double x, double y, bool trajectory_finished);
        void update_arena();
        void update_origin(double x, double y);
        void set_title(std::string title);
        void report_agent_state_transition(double x, double y, State from_state, State to_state);
        void report_endpoint_location(Endpoint endpoint, double x, double y);
        void report_place_cell(Raster raster, int timestep, PlaceCell *place_cell);
        void set_arena_size(double size);
        void set_scale_bars(int scale_bars);
        void add_label(double label_x, double label_y, std::string label_text);

    protected:
        Simulation *simulation;
        SimulationArenaComponentPlot *simulation_arena_plot = nullptr;
        PolarArenaComponentPlot *polar_arena_plot = nullptr;
        RasterPlot *raster_plot = nullptr;
};

#endif
