// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "plot.h"

#include <fstream>
#include <iostream>

void Plot::set(const char *key, const char *value)
{
    this->settings[key] = (value != nullptr
        ? std::make_tuple(true, value)
        : std::make_tuple(false, ""));
}

void Plot::unset(const char *key)
{
    this->set(key, nullptr);
}

void Plot::dump_to_stream(std::ostream &stream)
{
    // Print settings
    for (auto iter = this->settings.begin(); iter != this->settings.end(); iter++) {
        bool valid_value;
        std::string value;
        std::tie(valid_value, value) = iter->second;
        if (valid_value) {
            stream << "set " << iter->first << " " << value << ";" << std::endl;
        } else {
            stream << "unset " << iter->first << ";" << std::endl;
        }
    }

    // Call the child class to dump the specific plot commands
    this->dump_plot_commands(stream);

    // Clear all settings
    for (auto iter = this->settings.begin(); iter != this->settings.end(); iter++) {
        bool valid_value;
        std::tie(valid_value, std::ignore) = iter->second;
        if (valid_value) {
            stream << "unset " << iter->first << ";" << std::endl;
        } else {
            stream << "set " << iter->first << ";" << std::endl;
        }
    }
}

void Plot::set_terminal(const char *terminal)
{
    this->terminal = terminal;
}

void Plot::show()
{
    if (!this->pipe) {
        if (this->plot_sink == stdout_plot_sink) {
            this->pipe = &std::cout;
        } else if (this->plot_sink == pipe_plot_sink) {
            this->pipe = new std::fstream("./plot_pipe", std::ios::out);
        } else {
            return;
        }
    }
    (*pipe) << "set terminal " << this->terminal << ";" << std::endl;
    this->dump_to_stream(*pipe);
    (*pipe) << std::endl << "quit;" << std::endl;
}

PlotComponent *ComponentPlot::add_plot_component(const char *name, const char *plot_command)
{
    PlotComponent *plot_component = new PlotComponent(name, plot_command);
    this->plot_components.push_back(plot_component);
    return plot_component;
}

void ComponentPlot::dump_plot_commands(std::ostream &stream)
{
    // Print plot command
    stream << "plot " ;
    for (PlotComponent *plot_component : this->plot_components) {
        if (plot_component->tellp() == 0) {
            continue;
        }
        stream << "'-' " << plot_component->get_plot_command() << ", ";
    }
    stream << "1/0 notitle;" << std::endl;

    // Print data for each component
    for (PlotComponent *plot_component : this->plot_components) {
        if (plot_component->tellp() == 0) {
            continue;
        }
        plot_component->dump_to_stream(stream);
        stream << "e" << std::endl;
    }
}

PlotComponent::PlotComponent(const char *name, const char *plot_command)
    : std::stringstream(std::ios::in | std::ios::out | std::ios::app),
      name(name), plot_command(plot_command)
{
}

void PlotComponent::reset()
{
    this->str("");
}

const char *PlotComponent::get_plot_command()
{
    return this->plot_command.c_str();
}

void PlotComponent::dump_to_stream(std::ostream &stream)
{
    // Seek read and write pointers of the underlying buffer to the beginning
    this->seekp(0, std::ios::beg);
    this->seekg(0, std::ios::beg);

    // Feed the underlying buffer to the output stream
    stream << "# Start of data for component " << this->name << std::endl;
    stream << this->rdbuf();
    stream << "# End of data for component " << this->name << std::endl;

    // Reset read and write pointers of the underlying buffer to the end
    this->seekp(0, std::ios::end);
    this->seekg(0, std::ios::end);
}

void MultiPlot::add_plot(double x, double y, double width, double height, Plot *plot)
{
    this->plots.push_back(std::make_tuple(x, y, width, height, plot));
}

void MultiPlot::dump_plot_commands(std::ostream &stream)
{
    stream << "set multiplot;" << std::endl;
    for (auto plot_tuple : this->plots) {
        double x, y, width, height;
        Plot *plot;
        std::tie(x, y, width, height, plot) = plot_tuple;
        stream << "set size " << width << "," << height << ";" << std::endl;
        stream << "set origin " << x << "," << y << ";" << std::endl;
        plot->dump_to_stream(stream);
    }
    stream << "unset multiplot;" << std::endl;
}
