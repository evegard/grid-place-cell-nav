// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef ARENA_H_INCLUDED
#define ARENA_H_INCLUDED

#include <vector>
#include <tuple>

#include "numerical.h"

class Arena
{
    public:
        static Arena *load_arena(const char *wkt_string);
        virtual void update_sensors(double x, double y,
            double range, real *sensors, int sensor_count) = 0;
        virtual bool line_intersects(double ax, double ay, double bx, double by) = 0;
        std::vector<std::tuple<double, double, double, double>> lines;
        std::vector<std::vector<std::tuple<double, double>>> polygons;
};

#endif
