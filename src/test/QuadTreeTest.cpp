#include <gtest/gtest.h>
#include "../QuadTree.h"
#include "../object.h"
#include "../Geometry.h"
#include <vector>
#include <memory>

using namespace Foliage;
using namespace Foliage::Util;

class QuadTreeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Define the boundaries of the QuadTree
        Geometry::Position topLeft(100, 100);
        Geometry::Position bottomRight(-100, -100);
        Geometry::BoundingBox boundingBox(bottomRight, topLeft);

        // Initialize the QuadTree with capacity (e.g., 4)
        quadTree = std::make_unique<QuadTree>(boundingBox, 4);

        // Create sample objects (Nodes and Ways)
        // Nodes
        auto node1 = std::make_shared<ObjectType::Node>();
        node1->position = Geometry::Position(10, 10);

        auto node2 = std::make_shared<ObjectType::Node>();
        node2->position = Geometry::Position(-20, 30);

        auto node3 = std::make_shared<ObjectType::Node>();
        node3->position = Geometry::Position(50, -50);

        // Ways (assuming Ways have positions or bounding boxes)
        auto way1 = std::make_shared<ObjectType::Way>();
        way1->nodes.push_back(node1);
        way1->nodes.push_back(node2);

        auto way2 = std::make_shared<ObjectType::Way>();
        way2->nodes.push_back(node2);
        way2->nodes.push_back(node3);

        // Store objects for use in tests
        objects.push_back(node1);
        objects.push_back(node2);
        objects.push_back(node3);
    //    objects.push_back(way1);
    //    objects.push_back(way2);

        // Insert objects into the QuadTree
        for (const auto& obj : objects) {
            quadTree->insert(obj);
        }
    }

    // Helper method to create a Node at a specific position
    std::shared_ptr<ObjectType::Node> createNode(double x, double y) {
        auto node = std::make_shared<ObjectType::Node>();
        node->position = Geometry::Position(x, y);
        return node;
    }

    // Test members
    std::unique_ptr<QuadTree> quadTree;
    std::vector<std::shared_ptr<ObjectType::Object>> objects;
};

TEST_F(QuadTreeTest, Initialization) {
    auto initialSize = quadTree->items.size();
    ASSERT_EQ(initialSize, 3);
}

TEST_F(QuadTreeTest, InsertObjects) {
    // Arrange
    auto initialSize = quadTree->items.size();
    ASSERT_EQ(initialSize, 3);

    // Act
    auto newNode = createNode(70, 70);
    bool inserted = quadTree->insert(newNode);

    // Assert
    ASSERT_TRUE(inserted) << "Expected the new node to be inserted.";
    ASSERT_EQ(quadTree->items.size(), initialSize + 1) << "QuadTree should contain one more item.";
}

TEST_F(QuadTreeTest, InsertOutsideBounds) {
    // Arrange
    auto outOfBoundsNode = createNode(200, 200);

    // Act
    bool inserted = quadTree->insert(outOfBoundsNode);

    // Assert
    ASSERT_FALSE(inserted) << "Expected insertion to fail for out-of-bounds node.";
}

TEST_F(QuadTreeTest, FindNodesWithinBoundingBox) {
    // Arrange
    Geometry::Position topLeft(0, 0);
    Geometry::Position bottomRight(60, 60);
    Geometry::BoundingBox searchBox(topLeft, bottomRight);

    // Act
    auto foundNodes = quadTree->find_node(searchBox);

    // Assert
    ASSERT_FALSE(foundNodes.empty()) << "Expected to find nodes within the bounding box.";
    for (const auto& node : foundNodes) {
        ASSERT_TRUE(searchBox.contains(node->position)) << "Found node is not within the search bounding box.";
    }
}
TEST_F(QuadTreeTest, SubdivideAfterCapacityExceeded) {
    // Arrange
    int capacity = quadTree->capacity;
    // Insert enough nodes to exceed capacity
    for (int i = 0; i < capacity + 1; ++i) {
        auto node = createNode(i * 10, i * 10);
        quadTree->insert(node);
    }

    // Act
    bool isDivided = quadTree->divided;

    // Assert
    ASSERT_TRUE(isDivided) << "QuadTree should be subdivided after capacity is exceeded.";
}

TEST_F(QuadTreeTest, FindNodesWithPredicate) {
    // Arrange
    Geometry::Position topLeft(-100, -100);
    Geometry::Position bottomRight(100, 100);
    Geometry::BoundingBox searchBox(topLeft, bottomRight);

    // Define a predicate to include nodes only in the positive quadrant
    auto predicate = [](const std::shared_ptr<ObjectType::Node>& node) {
        return node->position.latitude >= 0 && node->position.longitude >= 0;
    };

    // Act
    auto foundNodes = quadTree->find_node(searchBox, predicate);

    // Assert
    ASSERT_FALSE(foundNodes.empty()) << "Expected to find nodes satisfying the predicate.";
    for (const auto& node : foundNodes) {
        ASSERT_GE(node->position.latitude, 0);
        ASSERT_GE(node->position.longitude, 0);
    }
}


TEST_F(QuadTreeTest, DestructorCleansUpChildren) {
    // Arrange
    {
        auto tempQuadTree = std::make_unique<QuadTree>(quadTree->bounding_box, quadTree->capacity);
        // Insert enough nodes to cause subdivision
        for (int i = 0; i < quadTree->capacity + 1; ++i) {
            auto node = createNode(i * 5, i * 5);
            tempQuadTree->insert(node);
        }

        // At this point, tempQuadTree should be subdivided
        ASSERT_TRUE(tempQuadTree->divided);
    }
    // Act & Assert
    // Since tempQuadTree is out of scope, we expect its destructor to have been called without issues.
    SUCCEED() << "QuadTree destructor executed without issues.";
}
