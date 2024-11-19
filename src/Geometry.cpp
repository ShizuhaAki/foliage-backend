//
// Created by lilyw on 11/12/2024.
//

#include "Geometry.h"
#include <cmath>

namespace Foliage::Geometry {
    bool BoundingBox::contains(Position p) const {
        return min_position.longitude <= p.longitude && min_position.latitude <= p.latitude &&
            max_position.longitude >= p.longitude && max_position.latitude >= p.latitude;
    }
    bool BoundingBox::intersects(const BoundingBox& b) const {
        bool no_overlap = max_position.latitude < b.min_position.latitude ||
                          min_position.latitude > b.max_position.latitude ||
                          max_position.longitude < b.min_position.longitude ||
                          min_position.longitude > b.max_position.longitude;

        // If no overlap, they don't intersect
        return !no_overlap;
    }
    double compute_distance(Geometry::Position start, Geometry::Position goal) {
        return std::sqrt((start.latitude - goal.latitude) * (start.latitude - goal.latitude) + (start.longitude - goal.longitude)
               * (start.longitude - goal.longitude));
    }
}