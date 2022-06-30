// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "ui.h"
#include "polar.h"

SimulationArenaComponentPlot::SimulationArenaComponentPlot(
            Simulation *simulation, double right_border)
    : simulation(simulation), right_border(right_border),
      plot_title(" Unknown experiment")
{
    this->set("size", "square");
    this->set_arena_size(200);
    this->unset("xtics");
    this->unset("ytics");
    this->unset("border");
    this->set("key", "font ',14' at graph -0.05,0.99 right top");

    this->new_trajectory("black", "Unknown");

    for (int i = 0; i < ENDPOINT_COUNT; i++) {
        this->endpoint_locations[i] = nullptr;
    }
}

void SimulationArenaComponentPlot::set_arena_size(double size)
{
    this->arena_size = size;
    this->lo_bound = 1;
    this->hi_bound = this->arena_size - 1;

    std::string xrange, yrange;
    xrange += "[0:";  xrange += std::to_string(size); xrange += "]; # Arena xrange";
    yrange += "[-"; yrange += std::to_string(size); yrange += ":0]; # Arena yrange";
    this->set("xrange", xrange.c_str());
    this->set("yrange", yrange.c_str());
}

void SimulationArenaComponentPlot::set_scale_bars(int scale_bars)
{
    this->scale_bars = scale_bars;
}

void SimulationArenaComponentPlot::add_label(double label_x, double label_y, std::string label_text)
{
    this->labels.push_back(std::make_tuple(label_x, label_y, label_text));
}

void SimulationArenaComponentPlot::report_agent_state_transition(
        double x, double y, State from_state, State to_state)
{
    auto transition = std::make_tuple(from_state, to_state);
    if (this->agent_state_transitions.count(transition) > 0) {
        (*this->agent_state_transitions[transition]) << x << " " << y << std::endl;
    }
}

void SimulationArenaComponentPlot::report_endpoint_location(Endpoint endpoint, double x, double y)
{
    if (!this->point_components_initialized) {
        this->endpoint_locations[start_endpoint] =
            this->add_plot_component("arena start points",
                "with point pt 7 ps 1.5 lw 3 lc rgb 'blue' title 'Start of return attempt'");
        this->endpoint_locations[end_endpoint] =
            this->add_plot_component("arena end points",
                "with point pt 7 ps 1.5 lw 3 lc rgb 'red' title 'End of return attempt'");
        this->agent_state_transitions[
            std::make_tuple(approach_subgoal_state, replay_episode_state)] =
                this->add_plot_component("replay episode locations",
                    "with point pt 8 ps 2 lw 3 lc rgb 'red' title 'Replay episode'");
        this->agent_state_transitions[
            std::make_tuple(approach_subgoal_state, topological_step_state)] =
                this->add_plot_component("topological step locations",
                    "with point pt 10 ps 2 lw 3 lc rgb 'dark-green' title 'Topological step'");
        this->agent_state_transitions[
            std::make_tuple(replay_episode_state, exploration_state)] =
                this->add_plot_component("exploration start locations",
                    "with point pt 4 ps 1.5 lw 3 lc rgb 'dark-violet' title 'Enter exploration phase'");
        this->agent_state_transitions[
            std::make_tuple(exploration_state, initiate_navigation_state)] =
                this->add_plot_component("exploration end locations",
                    "with point pt 12 ps 2 lw 3 lc rgb 'brown' title 'Leave exploration phase'");
        this->point_components_initialized = true;
    }
    if (this->endpoint_locations[endpoint] != nullptr) {
        (*this->endpoint_locations[endpoint]) << x << " " << y << std::endl;
    }
}

void SimulationArenaComponentPlot::dump_plot_commands(std::ostream &stream)
{
    this->plot_arena_polygons(stream);
    this->plot_arena_labels(stream);
    this->simulation->agent->model->place_graph->plot_place_cells(stream);
    this->plot_agent(stream, this->simulation->x, this->simulation->y, this->simulation->heading);

    std::string timestep = std::to_string(this->simulation->global_timestep);
    this->plot_status(stream, 0, "Timestep", timestep.c_str());
    this->plot_status(stream, 1, "Agent state",
        state_labels[this->simulation->agent->previous_state]);
    std::string confidence = std::to_string(
        (int)(this->simulation->agent->model->confidence * 100)) + "%";
    this->plot_status(stream, 2, "Confidence", confidence.c_str());

    stream << "# Agent state is \""
        << state_labels[this->simulation->agent->previous_state]
        << "\"" << std::endl;

    stream << "set label at screen " << (this->right_border / 2) << ",1 "
        << "center offset screen 0,-0.03 "
        << "'{/:Bold=26 " << this->plot_title << " }';" << std::endl;
    stream << "set label at screen " << (this->right_border / 2) << ",1 "
        << "center offset screen 0,-0.055 "
        << "'{/=22 " << this->simulation->agent->label << "}';" << std::endl;

    ComponentPlot::dump_plot_commands(stream);

    stream << "unset label;" << std::endl;
    stream << "unset object;" << std::endl;
    stream << "unset arrow;" << std::endl;
}

void SimulationArenaComponentPlot::new_trajectory(std::string color, std::string title)
{
    std::string plot_command = "with lines lw 2 "
        "lc rgb '" + color + "' title '" + title + "'";
    std::string component_name = "arena trajectory \"" + title + "\"";
    this->current_trajectory = this->add_plot_component(
        component_name.c_str(), plot_command.c_str());
}

void SimulationArenaComponentPlot::append_trajectory(double x, double y, bool trajectory_finished)
{
    if (trajectory_finished && this->current_trajectory->tellp() == 0) {
        return;
    }
    (*this->current_trajectory) << x << " " << y << std::endl;
}

void SimulationArenaComponentPlot::update_arena(Arena *arena)
{
    this->inverted_arena_rendering = false;
    for (auto polygon : arena->polygons) {
        for (auto point : polygon) {
            double x, y;
            std::tie(x, y) = point;
            if (x < this->lo_bound || x > this->hi_bound ||
                    -y < this->lo_bound || -y > this->hi_bound) {
                this->inverted_arena_rendering = true;
                return;
            }
        }
    }
}

void SimulationArenaComponentPlot::plot_arena_polygons(std::ostream &stream)
{
    stream << "# Start of arena definition" << std::endl;
    for (auto polygon : this->simulation->arena->polygons) {
        if (!this->inverted_arena_rendering) {
            stream << "set object polygon from ";
            bool not_first = false;
            for (auto point : polygon) {
                double x, y;
                std::tie(x, y) = point;
                stream << (not_first++ ? "to " : "") << x << "," << y << " ";
            }
            stream << "fillstyle solid "
                << "border linecolor rgb 'black' linewidth 2 "
                << "fillcolor rgb 'light-gray';" << std::endl;
        } else {
            double last_x = 0.0, last_y = 0.0;
            bool not_first = false;
            for (auto point : polygon) {
                double x, y;
                std::tie(x, y) = point;
                if (not_first++ &&
                         x >= this->lo_bound &&  x <= this->hi_bound &&
                        -y >= this->lo_bound && -y <= this->hi_bound &&
                         last_x >= this->lo_bound &&  last_x <= this->hi_bound &&
                        -last_y >= this->lo_bound && -last_y <= this->hi_bound) {
                    stream << "set arrow nohead "
                        << "from " << last_x << "," << last_y << " "
                        << "to " << x << "," << y << " "
                        << "linewidth 2;" << std::endl;
                }
                last_x = x;
                last_y = y;
            }
        }
    }
    stream << "# End of arena definition" << std::endl;
    stream << "# Start of scale bars" << std::endl;
    //stream << "set arrow nohead from graph 0.025,0.025 "
    //    << "rto first 0," << this->scale_bars << " front linewidth 4;" << std::endl;
    stream << "set arrow nohead from graph 0.025,0.025 "
        << "rto first " << this->scale_bars << ",0 front linewidth 4;" << std::endl;
    //stream << "set label at graph 0.025,0.025 "
    //    << "offset graph -0.02, first (0.5 * " << this->scale_bars << ") "
    //    << "center rotate by 90 '" << this->scale_bars << " cm';" << std::endl;
    stream << "set label at graph 0.025,0.025 "
        << "offset first (0.5 * " << this->scale_bars << "), graph -0.02 "
        << "center '" << this->scale_bars << " cm';" << std::endl;
    stream << "# End of scale bars" << std::endl;
}

void SimulationArenaComponentPlot::plot_arena_labels(std::ostream &stream)
{
    stream << "# Start of arena labels" << std::endl;
    for (auto label : this->labels) {
        double label_x, label_y;
        std::string label_text;
        std::tie(label_x, label_y, label_text) = label;
        stream << "set label at " << label_x << "," << label_y << " "
            << "'" << label_text << "' center font ',21';" << std::endl;
    }
    stream << "# End of arena labels" << std::endl;
}

void SimulationArenaComponentPlot::plot_agent(std::ostream &stream,
    double x, double y, double direction)
{
    stream << "# Start of agent cartoon" << std::endl;
    for (auto ellipse : std::vector<std::tuple<double, double, double>>{
            std::make_tuple(-6,  6, 1),
            std::make_tuple( 0, 12, 6),
            std::make_tuple( 6,  6, 6) }) {
        double shift, length, width;
        std::tie(shift, length, width) = ellipse;
        stream << "set object ellipse center "
            << (x + shift * std::cos(direction)) << ","
            << (y + shift * std::sin(direction)) << " "
            << "size " << length << "," << width << " "
            << "angle " << (180 / M_PI * direction) << " "
            << "fill solid border lc rgb 'black' fc rgb 'dark-gray' "
            << "front;" << std::endl;
    }
    stream << "# End of agent cartoon" << std::endl;
}

void SimulationArenaComponentPlot::plot_status(std::ostream &stream,
    double row, const char *title, const char *value)
{
    stream << "set label at screen 0,0.91 "
        << "offset character 3, screen " << (-0.06 * row) << " "
        << "'{/=17 " << title << ":}';" << std::endl;
    stream << "set label at screen 0,0.89 "
        << "offset character 3, screen " << (-0.06 * row) << " "
        << "'{/:Bold=17 " << value << "}';" << std::endl;
}

PolarArenaComponentPlot::PolarArenaComponentPlot()
{
    this->set("xrange", "[3*pi:-pi]");
    this->set("xtics", "('-pi' -pi, '0' 0, 'pi' pi, '2pi' 2*pi, '3pi' 3*pi)");
    this->set("ytics", "scale 0");
    this->set("xlabel", "'Direction from the goal point'");
    this->set("ylabel", "'Distance'");
    this->set_arena_size(200);
    this->set("size", "nosquare");
    this->set("border", "");
    this->arena_lines = this->add_plot_component("polar arena",
        "with vectors nohead lc rgb 'black' notitle");
    this->new_trajectory("black", "Unknown");
}

void PolarArenaComponentPlot::report_endpoint_location(Endpoint endpoint, double x, double y)
{
    if (!this->point_components_initialized) {
        this->endpoint_locations[start_endpoint] =
            this->add_plot_component("polar start points",
                "with point pt 7 ps 2 lc rgb 'blue' title 'Start'");
        this->endpoint_locations[end_endpoint] =
            this->add_plot_component("polar end points",
                "with point pt 7 ps 2 lc rgb 'red' title 'End'");
        this->point_components_initialized = true;
    }
    if (this->endpoint_locations[endpoint] != nullptr) {
        emit_transformed_point(*this->endpoint_locations[endpoint],
            x - this->origin_x, y - this->origin_y);
    }
}

void PolarArenaComponentPlot::set_arena_size(double size)
{
    this->arena_size = size;
    std::string yrange;
    yrange += "[0:"; yrange += std::to_string(size * std::sqrt(2.0)); yrange += "]";
    this->set("yrange", yrange.c_str());
}

void PolarArenaComponentPlot::new_trajectory(std::string color, std::string title)
{
    std::string plot_command = "with vectors nohead lw 2 "
        "lc rgb '" + color + "' notitle";
    std::string component_name = "polar trajectory \"" + title + "\"";
    this->current_trajectory = this->add_plot_component(
        component_name.c_str(), plot_command.c_str());
    this->first_point = true;
    this->last_x = this->last_y = 0.0;
}

void PolarArenaComponentPlot::append_trajectory(double x, double y)
{
    if (!this->first_point) {
        emit_transformed_line(*this->current_trajectory, 1,
            last_x - this->origin_x, last_y - this->origin_y,
            x - this->origin_x, y - this->origin_y);
    }
    this->first_point = false;
    last_x = x;
    last_y = y;
}

void PolarArenaComponentPlot::set_origin(double x, double y)
{
    this->origin_x = x;
    this->origin_y = y;
}

void PolarArenaComponentPlot::update_arena(Arena *arena)
{
    double lo_bound = 1;
    double hi_bound = this->arena_size - 1;
    this->arena_lines->reset();
    for (auto line : arena->lines) {
        double ax, ay, bx, by;
        std::tie(ax, ay, bx, by) = line;
        if (ax < lo_bound || ax > hi_bound ||
                -ay < lo_bound || -ay > hi_bound ||
                bx < lo_bound || bx > hi_bound ||
                -by < lo_bound || -by > hi_bound) {
            continue;
        }
        int polar_segments = std::sqrt(
            std::pow(ax - bx, 2) +
            std::pow(ay - by, 2)) / 10.0;
        polar_segments = MAX(polar_segments, 1);
        emit_transformed_line(*this->arena_lines, polar_segments,
            ax - this->origin_x, ay - this->origin_y,
            bx - this->origin_x, by - this->origin_y);
    }
}

RasterPlot::RasterPlot()
{
    this->set("border", "");
    this->set("xlabel", "'Timestep'");
    this->set("ylabel", "'Place cell'");
    this->set("xtics", "");
    this->set("ytics", "");
    this->set("xrange", "[0:*]");
    this->set("yrange", "[0:*]");
    this->set("key", "default top left reverse box opaque samplen 0");
    this->rasters[agent_raster] =
        this->add_plot_component("agent raster",
            "with point pt 7 ps 1 lc rgb 'blue' title 'Agent'");
    this->rasters[replay_raster] =
        this->add_plot_component("replay raster",
            "with point pt 7 ps 1 lc rgb 'red' title 'Replay'");
    this->last_index[agent_raster] = -1;
    this->last_index[replay_raster] = -1;
}

void RasterPlot::report_place_cell(Raster raster, int timestep, PlaceCell *place_cell)
{
    if (place_cell == nullptr || place_cell->index == this->last_index[raster]) {
        return;
    }
    (*this->rasters[raster]) << timestep << " " << place_cell->index << std::endl;
    this->last_index[raster] = place_cell->index;
}

SimulationPlot::SimulationPlot(Simulation *simulation, bool lite)
    : simulation(simulation)
{
    int block_size = 240;
    int mec_columns = (int)ceil(simulation->agent->model->conf.module_count / 4.0);
    double blocks_height = 4.25;
    double blocks_width = 5 + mec_columns;
    double BH = 1.0 / blocks_height;
    double BW = 1.0 / blocks_width;

    this->simulation_arena_plot = new SimulationArenaComponentPlot(simulation, 5 * BW);
    this->add_plot(2 * BW, 1 * BH, 3 * BW, 3 * BH, this->simulation_arena_plot);

    if (!lite) {
        this->polar_arena_plot = new PolarArenaComponentPlot();
        this->add_plot(2.5 * BW, 0 * BH, 2.5 * BW, 1 * BH, this->polar_arena_plot);

        this->raster_plot = new RasterPlot();
        this->add_plot(0 * BW, 0 * BH, 2.5 * BW, 1 * BH, this->raster_plot);

        for (int i = 0; i < simulation->agent->model->conf.module_count; i++) {
            int row = i / mec_columns;
            int col = i % mec_columns;
            this->add_plot(
                (5 + col) * BW, (3 - row) * BH, 1 * BW, 1 * BH,
                new MecNetworkPlot(simulation->agent->model->mec_moving_convolved[i], i + 1));
            this->add_plot(
                (5 + col + 0.7) * BW, (3 - row + 0.7) * BH, 0.25 * BW, 0.25 * BH,
                new MotorNetworkPlot(simulation->agent->model->mec_motor[i], "red", nullptr, false, GRID_MOTOR_PLOT_RANGE));
        }
    }

    this->add_plot(0.325 * BW, (1.05 + 2 * 2 / 3.0) * BH, BW * 0.55, BH * 2/3,
        new MotorNetworkPlot(simulation->agent->model->final_motor, "black",
            "\"{/=14 Direction of goal vector}\\n{/=14decoded from grid cells}\"", true, GRID_MOTOR_PLOT_RANGE));
    this->add_plot(1.225 * BW, (1.05 + 2 * 2 / 3.0) * BH, BW * 0.55, BH * 2/3,
        new BorderSensorsPlot(simulation->agent->model->border_sensors));
    this->add_plot(0.325 * BW, (1.05 + 1 * 2 / 3.0) * BH, BW * 0.55, BH * 2/3,
        new MotorNetworkPlot(simulation->agent->model->first_normalized_motor, "orange",
            "\"{/=14 Motor network #1,}\\n{/=14before inhibition}\"", false, UI_MOTOR_PLOT_RANGE));
    this->add_plot(1.225 * BW, (1.05 + 1 * 2 / 3.0) * BH, BW * 0.55, BH * 2/3,
        new MotorNetworkPlot(simulation->agent->model->first_inhibited_motor, "red",
            "\"{/=14 Motor network #1,}\\n{/=14after inhibition}\"", false, UI_MOTOR_PLOT_RANGE));
    this->add_plot(0.325 * BW, (1.05 + 0 * 2 / 3.0) * BH, BW * 0.55, BH * 2/3,
        new MotorNetworkPlot(simulation->agent->model->second_normalized_motor, "orange",
            "\"{/=14 Motor network #2,}\\n{/=14before inhibition}\"", false, UI_MOTOR_PLOT_RANGE));
    this->add_plot(1.225 * BW, (1.05 + 0 * 2 / 3.0) * BH, BW * 0.55, BH * 2/3,
        new MotorNetworkPlot(simulation->agent->model->second_inhibited_motor, "red",
            "\"{/=14 Motor network #2,}\\n{/=14after inhibition}\"", false, UI_MOTOR_PLOT_RANGE));
    std::string terminal = "png font 'Nimbus Sans' fontscale " +
        std::to_string(block_size / 300.0) + " size " +
        std::to_string((int)(block_size * blocks_width)) + ", " +
        std::to_string((int)(block_size * blocks_height));
    this->set_terminal(terminal.c_str());
}

void SimulationPlot::new_trajectory(std::string color, std::string title)
{
    this->simulation_arena_plot->new_trajectory(color, title);
    if (this->polar_arena_plot) {
        this->polar_arena_plot->new_trajectory(color, title);
    }
}

void SimulationPlot::append_trajectory(double x, double y, bool trajectory_finished)
{
    this->simulation_arena_plot->append_trajectory(x, y, trajectory_finished);
    if (this->polar_arena_plot) {
        this->polar_arena_plot->append_trajectory(x, y);
    }
}

void SimulationPlot::update_arena()
{
    this->simulation_arena_plot->update_arena(this->simulation->arena);
    if (this->polar_arena_plot) {
        this->polar_arena_plot->update_arena(this->simulation->arena);
    }
}

void SimulationPlot::update_origin(double x, double y)
{
    if (this->polar_arena_plot) {
        this->polar_arena_plot->set_origin(x, y);
        this->polar_arena_plot->update_arena(this->simulation->arena);
    }
}

void SimulationPlot::set_title(std::string title)
{
    this->simulation_arena_plot->plot_title = title;
}

void SimulationPlot::report_agent_state_transition(
        double x, double y, State from_state, State to_state)
{
    this->simulation_arena_plot->report_agent_state_transition(x, y, from_state, to_state);
}

void SimulationPlot::report_endpoint_location(Endpoint endpoint, double x, double y)
{
    this->simulation_arena_plot->report_endpoint_location(endpoint, x, y);
    if (this->polar_arena_plot) {
        this->polar_arena_plot->report_endpoint_location(endpoint, x, y);
    }
}

void SimulationPlot::report_place_cell(Raster raster, int timestep, PlaceCell *place_cell)
{
    if (this->raster_plot) {
        this->raster_plot->report_place_cell(raster, timestep, place_cell);
    }
}

void SimulationPlot::set_arena_size(double size)
{
    this->simulation_arena_plot->set_arena_size(size);
    if (this->polar_arena_plot) {
        this->polar_arena_plot->set_arena_size(size);
    }
}

void SimulationPlot::set_scale_bars(int scale_bars)
{
    this->simulation_arena_plot->set_scale_bars(scale_bars);
}

void SimulationPlot::add_label(double label_x, double label_y, std::string label_text)
{
    this->simulation_arena_plot->add_label(label_x, label_y, label_text);
}
