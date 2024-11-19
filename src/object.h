//
// Created by lilyw on 11/12/2024.
//

#ifndef OBJECT_H
#define OBJECT_H
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <string>
#include <numeric>

#include "Geometry.h"
#include "AbstractDocument.h"

namespace Foliage::ObjectType {
    struct Node;
    struct NeighborInfo {
        double distance;  // Distance to the neighboring node
        std::unordered_map<std::string, std::string> tags;  // Tags of the way that led to this neighbor
    };
    struct Object {
        std::unordered_map<std::string, std::string> tags;
        int importance_level = 0;
        int64_t id;

        virtual ~Object() = default;

        [[nodiscard]] bool is_valid() const { return id != -1; }

        explicit Object(const int64_t id = -1): id(id) {
        }
    };
    struct Way : Object {
        std::vector<std::shared_ptr<const Node> > nodes;

        explicit Way(int64_t id = -1): Object(id) {
            nodes.clear();
            importance_level = 0;
        }

        [[nodiscard]] Geometry::BoundingBox get_bounding_box() const;
    };

    struct Node : Object, std::enable_shared_from_this<Node> {
        Geometry::Position position{};
        std::unordered_set<std::shared_ptr<const Way> > ways; // back reference to ways
        std::unordered_map<std::shared_ptr<const Node>, NeighborInfo> neighbors;

        explicit Node(int64_t id = -1): Object(id) {
            ways.clear();
            importance_level = 0;
        }

        void compute_neighbors();
        Node(const Node &nd) = default;
    };


}
#endif //OBJECT_H
