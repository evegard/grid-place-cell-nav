// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#ifndef NUMERICAL_H_INCLUDED
#define NUMERICAL_H_INCLUDED

#include <cmath>
#include <random>

#include "main.h"

typedef float real;
typedef real aligned_real __attribute__((aligned(REAL_ALIGNMENT)));

class Matrix
{
    public:
        Matrix(int width, int height);
        Matrix(int width, int height, real initial_value);

        int width;
        int height;
        real **values;
        real *raw_values;
};

class Vector
{
    public:
        Vector(int size);
        Vector(int size, real initial_value);

        void clear();
        void copy_from(Vector *other);
        real sum();

        int size;
        aligned_real *values;
};

class Random
{
    public:
        static double uniform();
        static double normal();

    protected:
        static bool initialized;
        static void initialize();

        static std::mt19937 *engine;
        static std::uniform_real_distribution<real> *uniform_distribution;
        static std::normal_distribution<real> *normal_distribution;
};

class Periodic
{
    public:
        static inline int modulo(int value, int period)
        {
            return value - period * (int)floor(1.0 * value / period);
        }

        static inline double double_modulo(double value, double period)
        {
            return value - period * (int)floor(1.0 * value / period);
        }
};

#endif
