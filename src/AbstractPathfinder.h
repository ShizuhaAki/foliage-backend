//
// Created by lilyw on 11/13/2024.
//

#ifndef ABSTRACTPATHFINDER_H
#define ABSTRACTPATHFINDER_H
#include "QuadTree.h"

namespace Foliage::Pathfinder {
    class AbstractPathfinder {
    public:
        virtual ~AbstractPathfinder() {}
        virtual std::vector<std::shared_ptr<const ObjectType::Node>> get_path(Geometry::Position start,
                                                                              Geometry::Position end,
                                                                              std::map<std::string, std::string>
                                                                              preferences)
        = 0;
    };
}
#endif //ABSTRACTPATHFINDER_H
