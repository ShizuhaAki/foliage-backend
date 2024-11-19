//
// Created by lilyw on 11/12/2024.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <algorithm>

namespace Foliage::Geometry {

    struct Position {
        double latitude;
        double longitude;
        bool operator==(const Position & r) const {
            return latitude == r.latitude && longitude == r.longitude;
        }
    };
    struct BoundingBox {
        Position min_position;
        Position max_position;
        explicit BoundingBox(Position min_position = {-1, -1}, Position max_position = {-1, -1}): min_position(min_position), max_position(max_position){}
        BoundingBox(const Position center, const double radius) {
            this->min_position = (Position){ center.latitude - radius, center.longitude - radius };
            this->max_position = (Position){ center.latitude + radius, center.longitude + radius };
        }
        bool contains(Position p) const;
        bool intersects(const BoundingBox &b) const;
    };

    double compute_distance(Geometry::Position start, Geometry::Position goal);
}

#endif //GEOMETRY_H
