//
// Created by lilyw on 11/12/2024.
//

#ifndef ABSTRACTDOCUMENT_H
#define ABSTRACTDOCUMENT_H
#include <memory>
#include <map>
namespace Foliage::ObjectType {
    struct Node;  // Forward declaration of Node
    struct Way;   // Forward declaration of Way
    struct Object; // Forward declaration of Object
}

namespace Foliage::DataProvider {
    class AbstractDocument {
    public:
        std::map<int64_t, std::shared_ptr<ObjectType::Node>> nodes_by_id;
        std::map<int64_t, std::shared_ptr<ObjectType::Way>> ways_by_id;
        virtual void reset() = 0;
        virtual void load() = 0;
        virtual void parse() = 0;
        [[nodiscard]] virtual std::shared_ptr<ObjectType::Object> get_object_by_id(int64_t id) const = 0;

        [[nodiscard]] virtual std::shared_ptr<ObjectType::Node> get_node_by_id(int64_t id) const = 0;

        [[nodiscard]] virtual std::shared_ptr<ObjectType::Way> get_way_by_id(int64_t id) const = 0;
        virtual ~AbstractDocument() = default;
    };
}
#endif //ABSTRACTDOCUMENT_H
