// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED

#include <vector>
#include <utility>

#include "numerical.h"
#include "network.h"
#include "mec.h"
#include "main.h"
#include "plot.h"
#include "graph.h"
#include "motor.h"

class MecDiffNetwork;
class MotorNetwork;
class BorderMotorInput;
class VelocityInput;

enum MotorMode { halt_mode, forced_mode, grid_decoder_mode, last_heading_mode };

class Model
{
    public:
        Model(struct ModelConf conf);

        void settle();
        void simulate_timestep();

        struct ModelConf conf;

        struct {
            double heading;
            double speed;
            MotorMode motor_mode;
            double motor_tuning;
            double motor_offset;
            double confidence_threshold;
        } input;

        struct {
            double heading;
            double speed;
            bool halted;
        } output;

        PlaceGraph *place_graph;
        Vector *border_sensors;

        std::vector<VelocityInput *> velocity_inputs;
        std::vector<MecNetwork *> mec_fixed;
        std::vector<MecNetwork *> mec_moving;
        std::vector<ConvolvedMecNetwork *> mec_fixed_convolved;
        std::vector<ConvolvedMecNetwork *> mec_moving_convolved;
        std::vector<MecDiffNetwork *> mec_diff;
        std::vector<MotorNetwork *> mec_motor;
        MotorNetwork *final_motor;

        MotorNetwork *first_normalized_motor;
        MotorNetwork *first_inhibited_motor;
        MotorNetwork *second_normalized_motor;
        MotorNetwork *second_inhibited_motor;

        Input *first_border_motor_input;
        Input *second_border_motor_input;

        double confidence;
};

class VelocityInput : public Input
{
    public:
        VelocityInput(MecNetwork *efferent);
        void add_inputs();
        void set_velocity(real x, real y);

        MecNetwork *efferent;

        real velocity_x;
        real velocity_y;
};

class AllMotorsPlot : public Plot
{
    public:
        AllMotorsPlot(Model *model);

    protected:
        void dump_plot_commands(std::ostream &gnuplot);
        Model *model;
};

class BorderSensorsPlot : public Plot
{
    public:
        BorderSensorsPlot(Vector *border_sensors);

    protected:
        void dump_plot_commands(std::ostream &gnuplot);
        Vector *border_sensors;
};

#endif
