//
// Created by lilyw on 11/13/2024.
//

#ifndef LAYEREDASTARPATHFINDER_H
#define LAYEREDASTARPATHFINDER_H
#include <queue>

#include "AbstractPathfinder.h"


namespace Foliage::Pathfinder {
    class LayeredAStarPathfinder : AbstractPathfinder {
    public:
        std::vector<std::shared_ptr<const ObjectType::Node> > get_path(
            Geometry::Position start,
            Geometry::Position end,
            std::map<std::string, std::string> preferences
        ) override;

        struct PathfinderNode {
            std::shared_ptr<const ObjectType::Node> node_object;
            double f_score = 0.0, g_score = 0.0;
            std::shared_ptr<PathfinderNode> came_from_start = nullptr;
            std::shared_ptr<PathfinderNode> came_from_goal = nullptr;
            std::string current_highway;
            bool operator<(const PathfinderNode& rhs) const {
                return f_score < rhs.f_score;
            }
        };

        struct NodeWayPair {
            double way_weight;
            ObjectType::NeighborInfo way;
            std::shared_ptr<PathfinderNode> node;
        };

        LayeredAStarPathfinder(std::shared_ptr<Util::QuadTree> qtree = nullptr): qtree(qtree) {
        }

        ~LayeredAStarPathfinder() override {
        }

        static double get_way_weight(ObjectType::NeighborInfo way);

        std::shared_ptr<Util::QuadTree> qtree;

        [[nodiscard]] std::shared_ptr<ObjectType::Node> find_closest_node_on_highway(
            Geometry::Position position, double search_radius = 0.005) const;


        static std::vector<LayeredAStarPathfinder::NodeWayPair>
        get_neighbors(
            const std::shared_ptr<const ObjectType::Node> &node,
            const std::map<std::string, std::string> &preferences,
            int layer,
            std::unordered_map<int64_t, std::shared_ptr<PathfinderNode> > &node_map
        );

        static void expand_neighbors(
            const std::shared_ptr<PathfinderNode> &current_node,
            std::priority_queue<std::shared_ptr<PathfinderNode>, std::vector<std::shared_ptr<PathfinderNode>>,
            std::function<bool(const std::shared_ptr<PathfinderNode> &, const std::shared_ptr<PathfinderNode>
                &)>> &open_set,
            std::unordered_set<int64_t> &closed_set_ids,
            std::unordered_map<int64_t, std::shared_ptr<PathfinderNode>> &node_map,
            const std::map<std::string, std::string> &preferences,
            const Geometry::Position &start,
            const Geometry::Position &end,
            int current_layer,
            bool from_start
        );

        static std::vector<std::shared_ptr<const ObjectType::Node> > reconstruct_path(
            std::shared_ptr<PathfinderNode> current_start,
            std::shared_ptr<PathfinderNode> current_goal
        );
    };
}

#endif //LAYEREDASTARPATHFINDER_H
