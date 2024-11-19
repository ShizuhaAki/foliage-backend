//
// Created by lilyw on 11/12/2024.
//

#include "QuadTree.h"

#include <iostream>

namespace Foliage::Util {
    using namespace Foliage::ObjectType;
    bool QuadTree::insert(const std::shared_ptr<Object>& item) {
        // Use polymorphism to handle different object types
        if (auto node = std::dynamic_pointer_cast<const Node>(item)) {
            // The object is a node
            if (!bounding_box.contains(node->position)) {
            //    std::cerr << "Did not insert: node is out of bounds" << std::endl;
                return false;  // Node is outside the bounding box
            }
        } else if (auto way = std::dynamic_pointer_cast<const Way>(item)) {
            // The object is a way
            if (!bounding_box.intersects(way->get_bounding_box())) {
                return false;  // Way does not intersect the bounding box
            }
        } else {
            throw std::invalid_argument("Argument is neither a Node nor a Way");
        }

        if (items.size() < capacity) {
            items.push_back(item);
            return true;
        } else {
            if (!divided) {
                subdivide();
                divided = true;
            }

            for (const auto& child : children) {
                if (child->insert(item)) {
                    return true;  // Successfully inserted into a child
                }
            }
        }
        throw std::runtime_error("Reached unexpected position: shouldn't fail insert");
    //    return false;  // Should not reach here unless there's a bug
    }

    std::vector<std::shared_ptr<ObjectType::Node>> QuadTree::find_node(const Geometry::BoundingBox &bbox, const std::function<bool(const std::shared_ptr<ObjectType::Node>&)>& include_predicate) const {
        std::vector<std::shared_ptr<ObjectType::Node>> result;

        // Check items in the current QuadTree node
        for (const auto& item : items) {
            if (auto node = std::dynamic_pointer_cast<ObjectType::Node>(item)) {
                if (bbox.contains(node->position)) {
                    if (include_predicate(node)) {  // Predicate must explicitly filter
               //         std::cerr << "Node " << node->id << " satisfies predicate" << std::endl;
                        result.push_back(node);
                    } else {
                 //       std::cerr << "Node " << node->id << " does NOT satisfy predicate" << std::endl;
                    }
                }

            }
        }

        // If subdivided, check in children
        if (divided) {
            for (const auto& child : children) {
                if (child->bounding_box.intersects(bbox)) {
                    // Only search in children whose bounding boxes intersect the search box
                    auto child_result = child->find_node(bbox, include_predicate);
                    result.insert(result.end(), child_result.begin(), child_result.end());
                }
            }
        }

        return result;
    }

    std::vector<std::shared_ptr<ObjectType::Way>> QuadTree::find_way(const Geometry::BoundingBox &bbox, const std::function<bool(const std::shared_ptr<ObjectType::Way>&)>& include_predicate) const {
        std::vector<std::shared_ptr<ObjectType::Way>> result;

        // Check items in the current QuadTree node
        for (const auto& item : items) {
            if (auto way = std::dynamic_pointer_cast<ObjectType::Way>(item)) {
                if (bbox.intersects(way->get_bounding_box()) && include_predicate(way)) {
                    result.push_back(way);
                }
            }
        }

        // If subdivided, check in children
        if (divided) {
            for (const auto& child : children) {
                if (child->bounding_box.intersects(bbox)) {
                    // Only search in children whose bounding boxes intersect the search box
                    auto child_result = child->find_way(bbox);
                    result.insert(result.end(), child_result.begin(), child_result.end());
                }
            }
        }

        return result;
    }

    void QuadTree::subdivide() {
        auto minLat = bounding_box.min_position.latitude, minLon = bounding_box.min_position.longitude;
        auto maxLat = bounding_box.max_position.latitude, maxLon = bounding_box.max_position.longitude;

        auto midLat = (minLat + maxLat) / 2;
        auto midLon = (minLon + maxLon) / 2;

        children.push_back(new QuadTree(
            Geometry::BoundingBox({minLat, minLon}, {midLat, midLon}),
            capacity));

        children.push_back(new QuadTree(
            Geometry::BoundingBox({minLat, midLon}, {midLat, maxLon}),
            capacity));

        children.push_back(new QuadTree(
            Geometry::BoundingBox({midLat, minLon}, {maxLat, midLon}),
            capacity));

        children.push_back(new QuadTree(
            Geometry::BoundingBox({midLat, midLon}, {maxLat, maxLon}),
            capacity));

        divided = true;
    }
}
