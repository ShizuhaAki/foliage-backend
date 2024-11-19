// osm.cpp
#include "OSM.h"

#include <iostream>
#include <memory>

namespace Foliage::DataProvider::OSM {

    void Document::load() {
        auto result = doc.LoadFile(xmlFile.c_str());
        if (result != tinyxml2::XML_SUCCESS) {
            std::cerr << result << std::endl;
            throw std::runtime_error("Error loading XML file");
        }
    }

    std::shared_ptr<ObjectType::Object> Document::get_object_by_id(int64_t id) const {
        auto node_it = nodes_by_id.find(id);
        if (node_it != nodes_by_id.end()) {
            return node_it->second;
        }
        auto way_it = ways_by_id.find(id);
        if (way_it != ways_by_id.end()) {
            return way_it->second;
        }
        throw std::runtime_error("Object not found");
    }

    std::shared_ptr<ObjectType::Node> Document::get_node_by_id(int64_t id) const {
        auto it = nodes_by_id.find(id);
        if (it == nodes_by_id.end()) {
            throw std::runtime_error("Node not found");
        }
        return std::dynamic_pointer_cast<ObjectType::Node>(it->second);
    }

    std::shared_ptr<ObjectType::Way> Document::get_way_by_id(int64_t id) const {
        auto it = ways_by_id.find(id);
        if (it == ways_by_id.end()) {
            throw std::runtime_error("Way not found");
        }
        return std::dynamic_pointer_cast<ObjectType::Way>(it->second);
    }

    void Document::parse() {
        auto root = doc.FirstChildElement("osm");
        if (!root) {
            throw std::runtime_error("No <osm> tag found, check file integrity");
        }

        // Set border
        const auto* boundElement = root->FirstChildElement("bounds");
        if (boundElement) {
            border = Geometry::BoundingBox(
                {
                    .latitude = boundElement->DoubleAttribute("minlat"),
                    .longitude = boundElement->DoubleAttribute("minlon"),
                },
               {
                    .latitude = boundElement->DoubleAttribute("maxlat"),
                    .longitude = boundElement->DoubleAttribute("maxlon"),
               }
            );
        }

        // Parse nodes
        for (auto* nodeElement = root->FirstChildElement("node"); nodeElement;
             nodeElement = nodeElement->NextSiblingElement("node")) {
            auto node = std::make_shared<ObjectType::Node>();
            node->position.latitude = nodeElement->DoubleAttribute("lat");
            node->position.longitude = nodeElement->DoubleAttribute("lon");

            for (auto* tagElement = nodeElement->FirstChildElement("tag"); tagElement;
                 tagElement = tagElement->NextSiblingElement("tag")) {
                node->tags[tagElement->Attribute("k")] = tagElement->Attribute("v");
            }
            node->id = nodeElement->Int64Attribute("id");
            nodes_by_id[node->id] = node;
        }

        // Parse ways
        for (auto* wayElement = root->FirstChildElement("way"); wayElement;
             wayElement = wayElement->NextSiblingElement("way")) {
            auto way = std::make_shared<ObjectType::Way>();
            way->id = wayElement->Int64Attribute("id");
            ways_by_id[way->id] = way;
            for (auto* ndElement = wayElement->FirstChildElement("nd"); ndElement;
                 ndElement = ndElement->NextSiblingElement("nd")) {
                auto ref = ndElement->Int64Attribute("ref", -1);
                if (ref != -1) {
                    way->nodes.push_back(nodes_by_id.at(ref));
                    if (auto node = nodes_by_id[ref]) {
                        node->ways.insert(ways_by_id.at(way->id));  // back reference
                    }
                }
            }

            for (auto* tagElement = wayElement->FirstChildElement("tag"); tagElement;
                 tagElement = tagElement->NextSiblingElement("tag")) {
                way->tags[tagElement->Attribute("k")] = tagElement->Attribute("v");
            }
        }

        // Precompute neighbors for all nodes
        qtree.bounding_box = this->border;
        for (const auto& [_, node]: nodes_by_id) {
            node->compute_neighbors();
            qtree.insert(node);
        }

    }
}
