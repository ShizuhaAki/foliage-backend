#include <LayeredAStarPathfinder.h>

#include "src/OSM.h"
#include "third-party/httplib.h"
#include "third-party/json.hpp"

httplib::Server server;
Foliage::DataProvider::OSM::Document doc;
Foliage::Pathfinder::LayeredAStarPathfinder pathfinder;

int main() {
    server.Get("/test", [](const httplib::Request &, httplib::Response &res) {
        nlohmann::json json;
        json["status"] = "ok";
        json["message"] = "hello";
        res.set_content(json.dump(), "application/json");
    });
    server.Get("/load", [](const httplib::Request &req, httplib::Response &res) {
        if (req.get_param_value("file").empty()) {
            auto res_json = nlohmann::json();
            res_json["status"] = "error";
            res_json["message"] = "no file given";
            res.set_content(res_json.dump(), "application/json");
        } else {
            std::cerr << "Received load request " << req.get_param_value("file") << std::endl;
            doc.set_document(req.get_param_value("file"));
            try {
                doc.load();
                doc.parse();
                auto res_json = nlohmann::json();
                res_json["status"] = "ok";
                res.set_content(res_json.dump(), "application/json");
                pathfinder.qtree = std::shared_ptr<Foliage::Util::QuadTree>(&doc.qtree);
                std::cerr << "Load finished" << std::endl;
            } catch (const std::exception &e) {
                auto res_json = nlohmann::json();
                res_json["status"] = "error";
                res_json["message"] = e.what();
                res.set_content(res_json.dump(), "application/json");
            }
        }
    });
    server.Post("/query", [](const httplib::Request &req, httplib::Response &res) {
        try {
            auto req_json = nlohmann::json::parse(req.body);
            std::cerr << pathfinder.qtree->bounding_box.min_position.latitude << ' ' << pathfinder.qtree->bounding_box.min_position.longitude << std::endl;

            Foliage::Geometry::Position st(req_json["start"]["lat"], req_json["start"]["lon"]);
            Foliage::Geometry::Position goal(req_json["goal"]["lat"], req_json["goal"]["lon"]);

            auto preference = req_json.at("preference").get<std::map<std::string, std::string> >();
            auto path = pathfinder.get_path(st, goal, preference);
            nlohmann::json res_json = nlohmann::json::array();

            for (const auto &p: path) {
                res_json.push_back({{"lat", p->position.latitude}, {"lon", p->position.longitude}});
            }

            res.set_content(res_json.dump(), "application/json");
        } catch (const std::exception &e) {
            // Handle errors and send an error response
            nlohmann::json error_json = {{"error", e.what()}};
            res.status = 400;
            res.set_content(error_json.dump(), "application/json");
        }
    });

    server.listen("0.0.0.0", 9961);
    return 0;
}
