// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef PLOT_H_INCLUDED
#define PLOT_H_INCLUDED

#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "numerical.h"

class PlotComponent;

enum PlotSink { no_plot_sink, stdout_plot_sink, pipe_plot_sink };

class Plot
{
    public:
        void set(const char *key, const char *value);
        void unset(const char *key);
        void dump_to_stream(std::ostream &stream);

        void set_terminal(const char *terminal);
        void show();

        PlotSink plot_sink = no_plot_sink;

    protected:
        virtual void dump_plot_commands(std::ostream &stream) = 0;
        std::map<std::string, std::tuple<bool, std::string>> settings;
        std::string terminal;
        std::ostream *pipe = nullptr;
};

class ComponentPlot : public Plot
{
    public:
        PlotComponent *add_plot_component(const char *name, const char *plot_command);

    protected:
        void dump_plot_commands(std::ostream &stream);
        std::vector<PlotComponent *> plot_components;
};

class PlotComponent : public std::stringstream
{
    public:
        PlotComponent(const char *name, const char *plot_command);
        void reset();
        const char *get_plot_command();
        void dump_to_stream(std::ostream &stream);

    protected:
        std::string name;
        std::string plot_command;
};

class MultiPlot : public Plot
{
    public:
        void add_plot(double x, double y, double width, double height, Plot *plot);

    protected:
        void dump_plot_commands(std::ostream &stream);
        std::vector<std::tuple<double, double, double, double, Plot *>> plots;
};

#endif
