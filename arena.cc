// Navigating with grid and place cells in cluttered environments
// Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
//
// Licensed under the EUPL-1.2-or-later.
// Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
// Author: Vegard Edvardsen (https://github.com/evegard).

#include "arena.h"

#include <fstream>
#include <ostream>
#include <string>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::polygon<point_t> polygon_t;

class BoostGeometryArena : public Arena
{
    public:
        BoostGeometryArena(const char *wkt_string);
        virtual void update_sensors(double x, double y,
            double range, real *sensors, int sensor_count);
        virtual bool line_intersects(double ax, double ay, double bx, double by);

    protected:
        bg::model::multi_polygon<polygon_t> multipolygon;
};

Arena *Arena::load_arena(const char *wkt_string)
{
    return new BoostGeometryArena(wkt_string);
}

BoostGeometryArena::BoostGeometryArena(const char *wkt_string)
{
    bg::read_wkt(wkt_string, this->multipolygon);

    for (polygon_t polygon : this->multipolygon) {
        this->polygons.push_back({});
        point_t last_point;
        bool not_first_point = false;
        for (point_t point : bg::exterior_ring(polygon)) {
            this->polygons.back().push_back(std::make_tuple(
                bg::get<0>(point), bg::get<1>(point)));
            if (not_first_point++) {
                this->lines.push_back(std::make_tuple(
                    bg::get<0>(last_point),
                    bg::get<1>(last_point),
                    bg::get<0>(point),
                    bg::get<1>(point)));
            }
            last_point = point;
        }
    }
}

void BoostGeometryArena::update_sensors(double x, double y,
    double range, real *sensors, int sensor_count)
{
    point_t start_point(x, y);

    for (int sensor = 0; sensor < sensor_count; sensor++) {
        double sensor_direction = sensor * (2 * M_PI / sensor_count);
        point_t end_point(
            x + range * std::cos(sensor_direction),
            y + range * std::sin(sensor_direction));

        bg::model::linestring<point_t> sensor_beam;
        bg::append(sensor_beam, start_point);
        bg::append(sensor_beam, end_point);

        std::vector<point_t> intersection_points;
        bg::intersection(this->multipolygon, sensor_beam, intersection_points);

        bool got_intersection = false;
        double closest_distance = HUGE_VAL;
        point_t closest_intersection;
        for (point_t intersection_point : intersection_points) {
            double intersection_distance = bg::distance(
                start_point, intersection_point);
            if (!got_intersection ||
                    intersection_distance < closest_distance) {
                got_intersection = true;
                closest_distance = intersection_distance;
                closest_intersection = intersection_point;
            }
        }

        sensors[sensor] = 0.0;
        if (got_intersection) {
            sensors[sensor] = 2.0 * std::exp(-5.0 * (closest_distance / range));
        }
    }
}

bool BoostGeometryArena::line_intersects(double ax, double ay, double bx, double by)
{
    bg::model::linestring<point_t> line;
    bg::append(line, point_t(ax, ay));
    bg::append(line, point_t(bx, by));
    return bg::intersects(this->multipolygon, line);
}
