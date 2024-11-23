#include "LayeredAStarPathfinder.h"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace Foliage::Pathfinder {
    // Updated get_path method using std::set for open lists
    std::vector<std::shared_ptr<const ObjectType::Node> > LayeredAStarPathfinder::get_path(
        Geometry::Position start,
        Geometry::Position end,
        std::map<std::string, std::string> preferences
    ) {
        // Priority queues for open sets
        auto compare = [](const std::shared_ptr<PathfinderNode> &a, const std::shared_ptr<PathfinderNode> &b) {
            return a->f_score > b->f_score; // Min-heap based on f_score
        };
        std::priority_queue<std::shared_ptr<PathfinderNode>, std::vector<std::shared_ptr<PathfinderNode> >,
                    std::function<bool(const std::shared_ptr<PathfinderNode> &,
                                       const std::shared_ptr<PathfinderNode> &)> >
                open_start(compare), open_goal(compare);

        // Closed sets
        std::unordered_set<int64_t> closed_start_ids, closed_goal_ids;

        // Node maps
        std::unordered_map<int64_t, std::shared_ptr<PathfinderNode> > node_map_start, node_map_goal;

        auto start_node_object = find_closest_node_on_highway(start);
        auto goal_node_object = find_closest_node_on_highway(end);
        if (!start_node_object || !goal_node_object) {
            std::cerr << "Start or goal node not found on highways.\n";
            return {};
        }

        // Initialize start node
        auto start_node = std::make_shared<PathfinderNode>();
        start_node->f_score = Geometry::compute_distance(start, end);
        start_node->g_score = 0;
        start_node->node_object = start_node_object;
        node_map_start[start_node_object->id] = start_node;
        open_start.push(start_node);

        // Initialize goal node
        auto goal_node = std::make_shared<PathfinderNode>();
        goal_node->f_score = Geometry::compute_distance(end, start);
        goal_node->g_score = 0;
        goal_node->node_object = goal_node_object;
        node_map_goal[goal_node_object->id] = goal_node;
        open_goal.push(goal_node);

        double best_cost = std::numeric_limits<double>::infinity();
        std::shared_ptr<PathfinderNode> best_meeting_node = nullptr;

        while (!open_start.empty() && !open_goal.empty()) {
            // Expand from the start side
            if (!open_start.empty()) {
                auto current_start = open_start.top();
                open_start.pop();

                if (closed_start_ids.count(current_start->node_object->id)) continue;
                closed_start_ids.insert(current_start->node_object->id);

                // Check for intersection
                if (closed_goal_ids.count(current_start->node_object->id)) {
                    auto meeting_node_goal = node_map_goal[current_start->node_object->id];
                    double total_cost = current_start->g_score + meeting_node_goal->g_score;
                    if (total_cost < best_cost) {
                        best_cost = total_cost;
                        best_meeting_node = current_start;
                    }
                }

                expand_neighbors(current_start, open_start, closed_start_ids, node_map_start, preferences, start, end,
                                 3, true);
            }

            // Expand from the goal side
            if (!open_goal.empty()) {
                auto current_goal = open_goal.top();
                open_goal.pop();

                if (closed_goal_ids.count(current_goal->node_object->id)) continue;
                closed_goal_ids.insert(current_goal->node_object->id);

                // Check for intersection
                if (closed_start_ids.count(current_goal->node_object->id)) {
                    auto meeting_node_start = node_map_start[current_goal->node_object->id];
                    double total_cost = current_goal->g_score + meeting_node_start->g_score;
                    if (total_cost < best_cost) {
                        best_cost = total_cost;
                        best_meeting_node = meeting_node_start;
                    }
                }

                expand_neighbors(current_goal, open_goal, closed_goal_ids, node_map_goal, preferences, end, start, 3,
                                 false);
            }

            // Early exit if the best meeting node is found
            if (best_meeting_node) {
                return reconstruct_path(best_meeting_node, node_map_goal[best_meeting_node->node_object->id]);
            }
        }

        std::cerr << "No solution\n";
        return {}; // Return an empty path if no solution is found
    }

    // Helper function to expand the neighbors of the current node in either direction
    void LayeredAStarPathfinder::expand_neighbors(
        const std::shared_ptr<PathfinderNode> &current_node,
        std::priority_queue<std::shared_ptr<PathfinderNode>, std::vector<std::shared_ptr<PathfinderNode> >,
            std::function<bool(const std::shared_ptr<PathfinderNode> &, const std::shared_ptr<PathfinderNode> &)> > &
        open_set,
        std::unordered_set<int64_t> &closed_set_ids,
        std::unordered_map<int64_t, std::shared_ptr<PathfinderNode> > &node_map,
        const std::map<std::string, std::string> &preferences,
        const Geometry::Position &start,
        const Geometry::Position &end,
        int current_layer,
        bool from_start
    ) {
        auto neighbors = get_neighbors(current_node->node_object, preferences, current_layer, node_map);
        for (const auto &[way_weight, way_to_neighbor, neighbor]: neighbors) {
            if (closed_set_ids.count(neighbor->node_object->id)) {
                continue; // Ignore already evaluated nodes
            }

            double tentative_g_score = current_node->g_score + way_weight; // Update with the weight of the way

            // If this path to the neighbor is better, or neighbor is not in node_map
            if (tentative_g_score < neighbor->g_score) {
                if (from_start) neighbor->came_from_start = current_node;
                else neighbor->came_from_goal = current_node;
                neighbor->g_score = tentative_g_score;
                neighbor->f_score = neighbor->g_score +
                                    Geometry::compute_distance(neighbor->node_object->position, end);
                open_set.push(neighbor);
            }
        }
    }

    // Helper function to reconstruct the path from the start to the goal
    std::vector<std::shared_ptr<const ObjectType::Node> > LayeredAStarPathfinder::reconstruct_path(
        std::shared_ptr<PathfinderNode> meeting_node_start,
        std::shared_ptr<PathfinderNode> meeting_node_goal
    ) {
        std::vector<std::shared_ptr<const ObjectType::Node> > path;

        // Build path from start to meeting node
        std::vector<std::shared_ptr<const ObjectType::Node> > forward_path;
        auto current = meeting_node_start;
        while (current != nullptr) {
            forward_path.push_back(current->node_object);
            current = current->came_from_start;
        }
        std::reverse(forward_path.begin(), forward_path.end());

        // Build path from meeting node to goal
        std::vector<std::shared_ptr<const ObjectType::Node> > backward_path;
        current = meeting_node_goal->came_from_goal; // Skip the meeting node to avoid duplication
        while (current != nullptr) {
            backward_path.push_back(current->node_object);
            current = current->came_from_goal;
        }

        // Combine paths
        path.reserve(forward_path.size() + backward_path.size());
        path.insert(path.end(), forward_path.begin(), forward_path.end());
        path.insert(path.end(), backward_path.begin(), backward_path.end());

        return path;
    }


    std::shared_ptr<ObjectType::Node> LayeredAStarPathfinder::find_closest_node_on_highway(
        const Geometry::Position position,
        const double search_radius
    ) const {
        const auto bbox = Geometry::BoundingBox(position, search_radius);
        auto nodes = qtree->find_node(bbox,
                                      [&](const std::shared_ptr<ObjectType::Node> &node) {
                                          //     std::cerr << "Evaluating predicate for node " << node->id << std::endl;
                                          for (const auto &way: node->ways) {
                                              if (way->tags.contains("highway")) {
                                                  //             std::cerr << "Accept node " << node->id;
                                                  //           std::cerr << " because of way " << way->id << std::endl;
                                                  return true;
                                              }
                                          }
                                          return false;
                                      });
        if (nodes.empty()) {
            std::cerr << "No node found within given proximity: proximity is " << search_radius << ", position is "
                    << position.latitude << " " << position.longitude << std::endl;
            return nullptr;
        }
        return *std::ranges::min_element(nodes,
                                         [=](const std::shared_ptr<ObjectType::Node> &a,
                                             const std::shared_ptr<ObjectType::Node> &b) {
                                             return Geometry::compute_distance(a->position, position) <
                                                    Geometry::compute_distance(b->position, position);
                                         });
    }

    double LayeredAStarPathfinder::get_way_weight(ObjectType::NeighborInfo way) {
        if (!way.tags.contains("highway")) throw std::invalid_argument("Way should be a highway");
        else {
            if (way.tags.contains("oneway") && way.tags.at("oneway") == "yes") {
                if (way.is_positive_direction == false) {
                    return -1; // negative weight will not be counted anyway;
                }
            }
            // Step 1. Compute the speed of the road
            double speed;
            if (way.tags.contains("maxspeed")) {
                speed = 0.9 * std::stod(way.tags.at("maxspeed"));
            } else {
                const std::unordered_map<std::string, double> assumed_speed = {
                    {"motorway", 120},
                    {"trunk", 100},
                    {"primary", 80},
                    {"secondary", 60},
                    {"tertiary", 50}
                };
                if (assumed_speed.contains(way.tags["highway"])) speed = assumed_speed.at(way.tags["highway"]);
                else speed = 30; // Default speed for unknown roads
            }

            // Step 2. Retrieve the length of the road
            double road_length = way.distance;

            // Step 3. Calculate base cost
            double cost = road_length / speed;

            // Step 4. Adjust cost based on road type
            const std::unordered_map<std::string, double> highway_bonus = {
                {"motorway", 0.5}, // Encourage motorways most
                {"motorway_link", 0.5},
                {"trunk", 0.8},
                {"trunk_link", 0.8},
                {"primary", 1.0},
                {"primary_link", 1.0},
                {"secondary", 3.0},
                {"secondary_link", 3.0},
                {"tertiary", 10.0},
                {"tertiary_link", 10.0},
                {"unclassified", 1000.0},
                {"residential", 10000.0}
           //     {"service", 10} // don't use service roads unless absolutely necessary
            };

            if (highway_bonus.contains(way.tags["highway"])) {
                cost *= highway_bonus.at(way.tags["highway"]);
            }
            return cost;
        }
    }


    std::vector<LayeredAStarPathfinder::NodeWayPair>
    LayeredAStarPathfinder::get_neighbors(
        const std::shared_ptr<const ObjectType::Node> &node,
        const std::map<std::string, std::string> &preferences,
        int layer, // TODO: refactor this! This argument is unused, pass whatever you want.
        std::unordered_map<int64_t, std::shared_ptr<PathfinderNode> > &node_map
    ) {
        const auto &neighbor_map = node->neighbors;
        std::vector<NodeWayPair> ret;

        // Get current highway type from the `PathfinderNode`
        std::string current_highway = node_map[node->id]->current_highway;

        for (const auto &[target, way_to_target]: neighbor_map) {
            if (way_to_target.tags.contains("highway")) {
                double way_weight = get_way_weight(way_to_target);
                std::string target_highway = way_to_target.tags.at("highway");
                // Discourage transitions off highways
                const std::unordered_map<std::string, int> highway_priority = {
                    {"motorway", 1},{"motorway_link", 1},
                    {"trunk", 2}, {"trunk_link", 2},
                    {"primary", 3},{"primary_link", 3},
                    {"secondary", 4},{"secondary_link", 4},
                    {"tertiary", 5},{"tertiary_link", 5},
                    {"unclassified", 6},
                    {"residential", 7}
                };

                int current_priority = highway_priority.contains(current_highway)
                                           ? highway_priority.at(current_highway)
                                           : 100;
                int target_priority = highway_priority.contains(target_highway)
                                          ? highway_priority.at(target_highway)
                                          : 100;

                if (current_priority < target_priority) {
                    way_weight *= 3; // Penalty for downgrading
                } else {
                    way_weight *= 0.5; // Bonus for upgrading
                }

                if (way_weight >= 0) {
                    // Get or create the PathfinderNode for the target node
                    std::shared_ptr<PathfinderNode> neighbor_node;
                    if (node_map.contains(target->id)) {
                        neighbor_node = node_map.at(target->id);
                    } else {
                        neighbor_node = std::make_shared<PathfinderNode>(PathfinderNode{
                            .node_object = target,
                            .f_score = std::numeric_limits<double>::infinity(),
                            .g_score = std::numeric_limits<double>::infinity(),
                            .came_from_start = nullptr,
                            .came_from_goal = nullptr,
                            .current_highway = target_highway // Update highway level
                        });
                        node_map[target->id] = neighbor_node;
                    }

                    // Create the NodeWayPair and push it to the result
                    ret.push_back(NodeWayPair{
                        way_weight, way_to_target, neighbor_node
                    });
                }
            }
        }

        return ret;
    }
}

