//
// Created by lilyw on 11/11/2024.
//

#ifndef OSM_H
#define OSM_H
#include <string>
#include <utility>
#include <vector>

#include "AbstractDocument.h"
#include "Geometry.h"
#include "object.h"
#include <memory>

#include "QuadTree.h"
#include "../third-party/tinyxml2.h"

namespace Foliage::DataProvider::OSM {
    using namespace Foliage::Geometry;
    class Document final : AbstractDocument {
    private:
        tinyxml2::XMLDocument doc;
        std::string xmlFile;

    public:
        std::unordered_map<int64_t, std::shared_ptr<ObjectType::Node>> nodes_by_id;
        std::unordered_map<int64_t, std::shared_ptr<ObjectType::Way>> ways_by_id;
        Util::QuadTree qtree;
        BoundingBox border;
        /**
         * Loads a file from the FS as a XMLDocument
         * @param xmlFile The file to load from
         */
        explicit Document(std::string xmlFile = ""):
            xmlFile(std::move(xmlFile)),
            qtree(Util::QuadTree(BoundingBox({-1, -1}, {-1, -1}), 10)) {}
         std::shared_ptr<ObjectType::Object> get_object_by_id(int64_t id) const override;

         std::shared_ptr<ObjectType::Node> get_node_by_id(int64_t id) const override;

         std::shared_ptr<ObjectType::Way> get_way_by_id(int64_t id) const override;

        void set_document(std::string xmlFile = "") {
            this->xmlFile = std::move(xmlFile);
        }
        void load() override;
        void reset() override;
        void parse() override;
    };
}
#endif //OSM_H
