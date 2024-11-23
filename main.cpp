#include <LayeredAStarPathfinder.h>
#include "src/OSM.h"
#include "third-party/httplib.h"
#include "third-party/json.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>
#include <future>
#include <string>

httplib::Server server;
Foliage::DataProvider::OSM::Document doc;
Foliage::Pathfinder::LayeredAStarPathfinder pathfinder;

std::mutex task_mutex;
std::condition_variable task_cv;
std::queue<std::function<void()> > task_queue;
std::atomic<bool> running{true};

enum TaskStatus {
    NotFound,
    InQueue,
    Running,
    Success,
    Failed
};

std::unordered_map<std::string, TaskStatus> task_status; // Task ID -> Status
std::unordered_map<std::string, std::string> task_result; // Task ID -> Result (JSON string)

// Generate a unique task ID
std::string generate_task_id() {
    static std::atomic<int> id_counter{0};
    return "task_" + std::to_string(id_counter++);
}

// Background worker to process tasks
void task_worker() {
    while (running) {
        std::function<void()> task; {
            std::unique_lock<std::mutex> lock(task_mutex);
            task_cv.wait(lock, [] { return !task_queue.empty() || !running; });
            if (!running && task_queue.empty()) break;
            task = std::move(task_queue.front());
            task_queue.pop();
        }
        if (task) task();
    }
}

// Convert TaskStatus to a string
std::string task_status_to_string(TaskStatus status) {
    switch (status) {
        case InQueue: return "InQueue";
        case Running: return "Running";
        case Success: return "Success";
        case Failed: return "Failed";
        default: return "NotFound";
    }
}

int main() {
    // Start the worker thread
    std::thread worker(task_worker);

    server.Get("/api/test", [](const httplib::Request &, httplib::Response &res) {
        nlohmann::json json;
        json["status"] = "ok";
        json["message"] = "hello";
        res.set_content(json.dump(), "application/json");
    });

    server.Get("/api/task/:id/status", [](const httplib::Request &req, httplib::Response &res) {
        auto task_id = req.path_params.at("id");
        auto it = task_status.find(task_id);
        nlohmann::json json;
        if (it != task_status.end()) {
            json = {{"status", task_status_to_string(it->second)}};
        } else {
            json = {{"status", "NotFound"}};
        }
        res.set_content(json.dump(), "application/json");
    });

    server.Get("/api/task/:id/result", [](const httplib::Request &req, httplib::Response &res) {
        auto task_id = req.path_params.at("id");
        auto it = task_result.find(task_id);
        if (it != task_result.end()) {
            res.set_content(it->second, "application/json");
        } else {
            nlohmann::json error_json = {{"error", "Task ID not found or not completed"}};
            res.status = 404;
            res.set_content(error_json.dump(), "application/json");
        }
    });

    server.Post("/api/load", [](const httplib::Request &req, httplib::Response &res) {
        auto file = req.get_param_value("file");
        if (file.empty()) {
            nlohmann::json res_json = {{"status", "error"}, {"message", "No file given"}};
            res.set_content(res_json.dump(), "application/json");
            return;
        }

        std::string task_id = generate_task_id();
        task_status[task_id] = InQueue; {
            std::unique_lock<std::mutex> lock(task_mutex);
            task_queue.push([file, task_id]() {
                task_status[task_id] = Running;
                try {
                    doc.set_document(file);
                    doc.reset();
                    doc.load();
                    doc.parse();
                    pathfinder.qtree = std::make_shared<Foliage::Util::QuadTree>(doc.qtree);
                    task_status[task_id] = Success;
                    nlohmann::json result_json = {
                        {
                            "min_bound",
                            {"lat", doc.border.min_position.latitude},
                            {"lon", doc.border.min_position.longitude}
                        },
                        {
                            "max_bound",
                            {"lat", doc.border.max_position.latitude},
                            {"lon", doc.border.max_position.longitude}
                        }
                    };
                    task_result[task_id] = result_json.dump();
                } catch (const std::exception &e) {
                    task_status[task_id] = Failed;
                    task_result[task_id] = nlohmann::json({{"error", e.what()}}).dump();
                }
            });
        }
        task_cv.notify_one();

        nlohmann::json res_json = {{"task_id", task_id}};
        res.set_content(res_json.dump(), "application/json");
    });

    server.Post("/api/query", [](const httplib::Request &req, httplib::Response &res) {
        try {
            auto req_json = nlohmann::json::parse(req.body);
            std::string task_id = generate_task_id();
            task_status[task_id] = InQueue;

            std::unique_lock<std::mutex> lock(task_mutex);
            task_queue.push([req_json, task_id]() {
                task_status[task_id] = Running;
                try {
                    Foliage::Geometry::Position st(req_json["start"]["lat"], req_json["start"]["lon"]);
                    Foliage::Geometry::Position goal(req_json["goal"]["lat"], req_json["goal"]["lon"]);

                    auto preference = req_json.at("preference").get<std::map<std::string, std::string> >();
                    auto path = pathfinder.get_path(st, goal, preference);

                    nlohmann::json res_json = nlohmann::json::array();
                    for (const auto &p: path) {
                        res_json.push_back({{"lat", p->position.latitude}, {"lon", p->position.longitude}});
                    }
                    res_json = {{"result", res_json}};
                    task_status[task_id] = Success;
                    task_result[task_id] = res_json.dump();
                } catch (const std::exception &e) {
                    task_status[task_id] = Failed;
                    task_result[task_id] = nlohmann::json({{"error", e.what()}}).dump();
                }
            });
            task_cv.notify_one();

            nlohmann::json res_json = {{"task_id", task_id}};
            res.set_content(res_json.dump(), "application/json");
        } catch (const std::exception &e) {
            nlohmann::json error_json = {{"error", e.what()}};
            res.status = 400;
            res.set_content(error_json.dump(), "application/json");
        }
    });

    server.listen("0.0.0.0", 9961);

    // Clean up
    running = false;
    task_cv.notify_all();
    worker.join();

    return 0;
}
