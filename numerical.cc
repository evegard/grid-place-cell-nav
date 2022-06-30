// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "numerical.h"

#include <cassert>

int round_up_to_nearest_multiple(int size, int multiple)
{
    int rest = size % multiple;
    if (rest == 0) {
        return size;
    } else {
        return size + (multiple - rest);
    }
}

Matrix::Matrix(int width, int height)
    : Matrix(width, height, 0.0)
{
}

Matrix::Matrix(int width, int height, real initial_value)
    : width(width), height(height)
{
    this->raw_values = new real[width * height];
    this->values = new real *[height];
    for (int y = 0; y < height; y++) {
        this->values[y] = &this->raw_values[y * width];
        for (int x = 0; x < width; x++) {
            this->values[y][x] = initial_value;
        }
    }
}

Vector::Vector(int size)
    : Vector(size, 0.0)
{
}

Vector::Vector(int size, real initial_value)
    : size(size)
{
    int allocation_result = posix_memalign(
        (void **)&this->values,
        REAL_ALIGNMENT,
        round_up_to_nearest_multiple(size, REAL_STRIDE) * sizeof(real));
    for (int x = 0; x < size; x++) {
        this->values[x] = initial_value;
    }
}

void Vector::clear()
{
    for (int x = 0; x < this->size; x++) {
        this->values[x] = 0.0;
    }
}

void Vector::copy_from(Vector *other)
{
    assert(this->size == other->size);
    for (int x = 0; x < this->size; x++) {
        this->values[x] = other->values[x];
    }
}

real Vector::sum()
{
    real sum = 0.0;
    for (int x = 0; x < this->size; x++) {
        sum += this->values[x];
    }
    return sum;
}

bool Random::initialized = false;
std::mt19937 *Random::engine = nullptr;
std::uniform_real_distribution<real> *Random::uniform_distribution = nullptr;
std::normal_distribution<real> *Random::normal_distribution = nullptr;

double Random::uniform()
{
    if (!Random::initialized) {
        Random::initialize();
    }
    return (*Random::uniform_distribution)(*Random::engine);
}

double Random::normal()
{
    if (!Random::initialized) {
        Random::initialize();
    }
    return (*Random::normal_distribution)(*Random::engine);
}

void Random::initialize()
{
    std::random_device random_device;
    Random::engine = new std::mt19937(random_device());
    Random::uniform_distribution = new std::uniform_real_distribution<real>();
    Random::normal_distribution = new std::normal_distribution<real>();
    Random::initialized = true;
}
