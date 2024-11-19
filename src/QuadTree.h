//
// Created by lilyw on 11/12/2024.
//

#ifndef QUADTREE_H
#define QUADTREE_H
#include <vector>

#include "Geometry.h"
#include "object.h"


namespace Foliage::Util {

    class QuadTree {
    private:
        void subdivide();
    public:
        std::vector<std::shared_ptr<ObjectType::Object>> items;
        int capacity;
        bool divided;
        Geometry::BoundingBox bounding_box;
        std::vector<QuadTree*> children;

        QuadTree(const Geometry::BoundingBox &bounding_box, int capacity):
            capacity(capacity),
            divided(false),
            bounding_box(bounding_box) {}


        ~QuadTree() {
            if (divided) {
                for (auto child :children ) {
                    delete child;
                }
            }
        }
        bool insert(const std::shared_ptr<ObjectType::Object>& item);
        std::vector<std::shared_ptr<ObjectType::Node>> find_node(
            const Geometry::BoundingBox &bbox,
            const std::function<bool(const std::shared_ptr<ObjectType::Node>&)>& include_predicate=
                [](const std::shared_ptr<ObjectType::Node>&) { return true; }) const;
        std::vector<std::shared_ptr<ObjectType::Way>> find_way(
            const Geometry::BoundingBox &bbox,
            const std::function<bool(const std::shared_ptr<ObjectType::Way>&)>& include_predicate =
                [](const std::shared_ptr<ObjectType::Way>&) { return true; }) const;
    };
}

#endif //QUADTREE_H
