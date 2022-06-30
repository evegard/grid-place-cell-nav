// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "polar.h"

#include <cmath>
#include <utility>

void emit_vector(std::ostream &stream, double t1, double r1, double t2, double r2)
{
    stream << t1 << " " << r1 << " "
        << (t2 - t1) << " " << (r2 - r1) << std::endl;
    stream << (t1 + 2 * M_PI) << " " << r1 << " "
        << (t2 - t1) << " " << (r2 - r1) << std::endl;
}

inline double theta(double x, double y)
{
    return std::atan2(y, x);
}

inline double radius(double x, double y)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

void emit_transformed_segment(std::ostream &stream,
    double x1, double y1, double x2, double y2)
{
    double t1 = theta(x1, y1);
    double t2 = theta(x2, y2);
    double r1 = radius(x1, y1);
    double r2 = radius(x2, y2);

    if (t2 > t1) {
        std::swap(t1, t2);
        std::swap(r1, r2);
    }

    if (std::abs(t2 - t1) <= M_PI) {
        emit_vector(stream, t1, r1, t2, r2);
    } else {
        double dt = t2 + 2 * M_PI - t1;
        double dr = r2 - r1;
        double rx = r1 + dr * (M_PI - t1) / dt;
        emit_vector(stream, t1, r1, M_PI, rx);
        emit_vector(stream, -M_PI, rx, t2, r2);
    }
}

void emit_transformed_line(std::ostream &stream, int segments,
    double x1, double y1, double x2, double y2)
{
    for (int segment = 0; segment < segments; segment++) {
        emit_transformed_segment(stream,
            x1 + (x2 - x1) * segment / segments,
            y1 + (y2 - y1) * segment / segments,
            x1 + (x2 - x1) * (segment + 1) / segments,
            y1 + (y2 - y1) * (segment + 1) / segments);
    }
}

void emit_transformed_point(std::ostream &stream, double x, double y)
{
    stream << theta(x, y) << " " << radius(x, y) << std::endl;
    stream << (theta(x, y) + 2 * M_PI) << " " << radius(x, y) << std::endl;
}
