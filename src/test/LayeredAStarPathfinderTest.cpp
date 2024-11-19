#include <gtest/gtest.h>
#include "../LayeredAStarPathfinder.h"
#include "../QuadTree.h"
#include "../object.h"
#include <map>
#include <vector>
#include <memory>  // For std::shared_ptr
#include <cmath>   // For std::sqrt
#include <fstream> // For file handling
#include <sstream> // For stringstream
#include <filesystem> // For directory scanning (C++17 and above)

namespace fs = std::filesystem;

using namespace Foliage;

class LayeredAStarPathfinderTest : public ::testing::TestWithParam<std::string> {
protected:
    void SetUp() override {
        // Initialize the QuadTree with appropriate boundaries
        qtree = std::make_shared<Foliage::Util::QuadTree>(Geometry::BoundingBox({0, 0}, {100, 100}), 15);

        // Create nodes dynamically based on the graph data
        load_graph_from_file(GetParam()); // Load graph from the test file

        // Initialize the pathfinder with the QuadTree
        pathfinder = std::make_shared<Foliage::Pathfinder::LayeredAStarPathfinder>(qtree);

        // Define preferences (if applicable)
        preferences = {{"highway", "primary"}, {"avoid_obstacles", "false"}};
    }

    void TearDown() override {
        // Clear neighbors to break potential cyclic references
        for (auto& node : nodes) {
            node->neighbors.clear();
            node->ways.clear();
        }

        // Reset shared pointers
        pathfinder.reset();
        qtree.reset();
    }

    // Helper function to calculate Euclidean distance
    double calculate_distance(const Foliage::Geometry::Position& a, const Foliage::Geometry::Position& b) {
        double dx = a.latitude - b.latitude;
        double dy = a.longitude - b.longitude;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Helper function to load graph from a file
    void load_graph_from_file(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        std::map<int, std::shared_ptr<Foliage::ObjectType::Node>> node_map;

        // Read the graph data from the file
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            int id1, id2;
            double lat1, lon1, lat2, lon2;
            // Read two nodes and their positions
            if (iss >> id1 >> lat1 >> lon1 >> id2 >> lat2 >> lon2) {
                auto node1 = get_or_create_node(id1, lat1, lon1, node_map);
                auto node2 = get_or_create_node(id2, lat2, lon2, node_map);

                // Create and add edges (ways) between the nodes
                auto way = std::make_shared<Foliage::ObjectType::NeighborInfo>();
                way->distance = calculate_distance(node1->position, node2->position);
                way->tags = {{"highway", "primary"}, {"maxspeed", "10"}};

                node1->neighbors[node2] = *way;
                node2->neighbors[node1] = *way;

                // Create and add the ways to the nodes' ways list
                auto new_way = Foliage::ObjectType::Way();
                new_way.nodes.push_back(node1);
                new_way.nodes.push_back(node2);
                new_way.tags["highway"] = "true";
                node1->ways.insert(std::make_shared<const Foliage::ObjectType::Way>(new_way));
                node2->ways.insert(std::make_shared<const Foliage::ObjectType::Way>(new_way));
            }
        }
        std::cerr << "Test setup finished here" << std::endl;
    }

    // Helper function to get or create a node from a map (creates new node if not found)
    std::shared_ptr<Foliage::ObjectType::Node> get_or_create_node(int id, double lat, double lon,
                                                                  std::map<int, std::shared_ptr<Foliage::ObjectType::Node>>& node_map) {
        if (node_map.find(id) == node_map.end()) {
            auto new_node = std::make_shared<Foliage::ObjectType::Node>();
            new_node->id = id;
            new_node->position = Foliage::Geometry::Position(lat, lon);
            node_map[id] = new_node;
            nodes.push_back(new_node);
            qtree->insert(new_node);
        }
        return node_map[id];
    }

    // Test members
    std::shared_ptr<Foliage::Pathfinder::LayeredAStarPathfinder> pathfinder;
    std::shared_ptr<Foliage::Util::QuadTree> qtree;
    Foliage::Geometry::Position start, end;
    std::map<std::string, std::string> preferences;
    std::vector<std::shared_ptr<Foliage::ObjectType::Node>> nodes;
};

TEST_P(LayeredAStarPathfinderTest, GetPath_ReturnsValidPath) {
    // Set start and end positions based on the first and last nodes (or any specific nodes you prefer)
    start = nodes.front()->position;
    end = nodes.back()->position;

    // Act
    auto st = std::clock();
    auto path = pathfinder->get_path(start, end, preferences);
    auto ed = std::clock();
    std::cerr << "took " << (ed-st)/(double)CLOCKS_PER_SEC << " seconds" << std::endl;
    // Assert
    ASSERT_FALSE(path.empty()) << "Expected non-empty path.";
    std::cerr << "Path info: " << std::endl;
    for (auto p : path) {
        std::cerr << p->position.latitude << " " << p->position.longitude << std::endl;
    }
    ASSERT_EQ(path.front()->position, start) << "Path does not start at the correct position.";
    ASSERT_EQ(path.back()->position, end) << "Path does not end at the correct position.";
}

// Function to scan the test directory and extract all test files
std::vector<std::string> GetTestFiles(const std::string& directory) {
    std::vector<std::string> test_files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            test_files.push_back(entry.path().string());
        }
    }
    return test_files;
}

// Instantiating the test suite for each file found in the testset directory
INSTANTIATE_TEST_SUITE_P(
    LayeredAStarTests,
    LayeredAStarPathfinderTest,
    ::testing::ValuesIn(GetTestFiles("../src/test/testset"))  // Directory containing your test files
);
