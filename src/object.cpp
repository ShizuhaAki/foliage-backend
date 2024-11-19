//
// Created by lilyw on 11/14/2024.
//

#include "object.h"

namespace Foliage::ObjectType {
    [[nodiscard]] Geometry::BoundingBox Way::get_bounding_box() const {
        auto minlat = std::numeric_limits<double>::max();
        auto maxlat = std::numeric_limits<double>::min();
        auto minlon = std::numeric_limits<double>::max();
        auto maxlon = std::numeric_limits<double>::min();
        for (const auto &node: nodes) {
            minlat = std::min(minlat, node->position.latitude);
            maxlat = std::max(maxlat, node->position.latitude);
            minlon = std::min(minlon, node->position.longitude);
            maxlon = std::max(maxlon, node->position.longitude);
        }
        return Geometry::BoundingBox({minlat, minlon}, {maxlat, maxlon});
    }

    void Node::compute_neighbors() {
        for (const auto& way : ways) {
            // Ensure way has more than one node
            if (way->nodes.size() < 2) continue;

            // Iterate through the nodes in the way to find adjacent nodes
            for (size_t i = 0; i < way->nodes.size() - 1; ++i) {
                auto current_node = way->nodes[i];
                auto next_node = way->nodes[i + 1];

                // Check if current node is this node or if next node is a neighbor
                if (current_node.get() == this || next_node.get() == this) {
                    auto neighbor = (current_node.get() == this) ? next_node : current_node;

                    // Calculate the distance between the nodes
                    double distance = compute_distance(this->position, neighbor->position);

                    // Store the distance and the way's tags in the neighbors map
                    neighbors[neighbor].distance = distance;
                    neighbors[neighbor].tags = way->tags;
                }
            }
        }
    }
}
