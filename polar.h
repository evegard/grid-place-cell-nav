// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef POLAR_H_INCLUDED
#define POLAR_H_INCLUDED

#include <ostream>

void emit_transformed_line(std::ostream &stream, int segments,
    double x1, double y1, double x2, double y2);
void emit_transformed_point(std::ostream &stream, double x, double y);

#endif
