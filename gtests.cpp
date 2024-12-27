 #include <gtest/gtest.h>
#include "graph_algorithms.cpp" 


Graph createLinearGraph() {
    Graph graph;
   
    Node* nodeA = graph.addNode(0, 0);
    Node* nodeB = graph.addNode(1, 0);
    Node* nodeC = graph.addNode(2, 0);
    Node* nodeD = graph.addNode(3, 0);
    Node* nodeE = graph.addNode(4, 0);

    
    graph.addEdge(0, 0, 1, 0, 1); 
    graph.addEdge(1, 0, 2, 0, 1); 
    graph.addEdge(2, 0, 3, 0, 1); 
    graph.addEdge(3, 0, 4, 0, 1); 
    return graph;
}


Graph createNetworkGraph() {
    Graph graph;
    
    Node* nodeA = graph.addNode(0, 0);
    Node* nodeB = graph.addNode(1, 0);
    Node* nodeC = graph.addNode(2, 0);
    Node* nodeD = graph.addNode(2, 1);
    Node* nodeE = graph.addNode(1, 1);
    Node* nodeF = graph.addNode(0, 1);

    
    graph.addEdge(0, 0, 1, 0, 1);
    graph.addEdge(1, 0, 2, 0, 1); 
    graph.addEdge(2, 0, 2, 1, 1); 
    graph.addEdge(2, 1, 1, 1, 1); 
    graph.addEdge(1, 1, 0, 1, 1);
    graph.addEdge(0, 0, 0, 1, 1); 
    graph.addEdge(1, 0, 1, 1, std::sqrt(2)); 
    graph.addEdge(0, 1, 2, 0, std::sqrt(2));
    return graph;
}

// Тест BFS
TEST(GraphTest, BFSTest) {
    
    Graph graph1 = createLinearGraph();
    Node* startNode = graph1.findClosestNode(0, 0);
    Node* goalNode = graph1.findClosestNode(4, 0);
    std::vector<Node*> path1 = graph1.bfs(startNode, goalNode);
    ASSERT_EQ(path1.size(), 5); // Путь должен содержать 5 узлов: A -> B -> C -> D -> E

    
    Graph graph2 = createNetworkGraph();
    startNode = graph2.findClosestNode(0, 0);
    goalNode = graph2.findClosestNode(2, 1);
    std::vector<Node*> path2 = graph2.bfs(startNode, goalNode);
    ASSERT_EQ(path2.size(), 4); // Путь от A до D: A -> B -> C -> D
}

// Тест DFS
TEST(GraphTest, DFSTest) {
    
    Graph graph1 = createLinearGraph();
    Node* startNode = graph1.findClosestNode(0, 0);
    Node* goalNode = graph1.findClosestNode(4, 0);
    std::vector<Node*> path1 = graph1.dfs(startNode, goalNode);
    ASSERT_EQ(path1.size(), 5); // Путь должен содержать 5 узлов: A -> B -> C -> D -> E

    
    Graph graph2 = createNetworkGraph();
    startNode = graph2.findClosestNode(0, 0);
    goalNode = graph2.findClosestNode(2, 1);
    std::vector<Node*> path2 = graph2.dfs(startNode, goalNode);
    ASSERT_EQ(path2.size(), 4); // Путь от A до D: A -> B -> C -> D
}

// Тест алгоритма Дейкстры
TEST(GraphTest, DijkstraTest) {
    
    Graph graph1 = createLinearGraph();
    Node* startNode = graph1.findClosestNode(0, 0);
    Node* goalNode = graph1.findClosestNode(4, 0);
    std::vector<Node*> path1 = graph1.dijkstra(startNode, goalNode);
    ASSERT_EQ(path1.size(), 5); // Путь должен содержать 5 узлов: A -> B -> C -> D -> E

    
    Graph graph2 = createNetworkGraph();
    startNode = graph2.findClosestNode(0, 0);
    goalNode = graph2.findClosestNode(2, 1);
    std::vector<Node*> path2 = graph2.dijkstra(startNode, goalNode);
    ASSERT_EQ(path2.size(), 4); // Путь от A до D: A -> B -> C -> D
}

// Тест алгоритма A*
TEST(GraphTest, AStarTest) {
    
    Graph graph1 = createLinearGraph();
    Node* startNode = graph1.findClosestNode(0, 0);
    Node* goalNode = graph1.findClosestNode(4, 0);
    std::vector<Node*> path1 = graph1.aStar(startNode, goalNode);
    ASSERT_EQ(path1.size(), 5); // Путь должен содержать 5 узлов: A -> B -> C -> D -> E

    
    Graph graph2 = createNetworkGraph();
    startNode = graph2.findClosestNode(0, 0);
    goalNode = graph2.findClosestNode(2, 1);
    std::vector<Node*> path2 = graph2.aStar(startNode, goalNode);
    ASSERT_EQ(path2.size(), 4); // Путь от A до D: A -> B -> C -> D
}

// Тест на отсутствие пути для всех алгоритмов 
TEST(GraphTest, NoPathTest) {
    
    Graph graph1 = createLinearGraph();
    Node* startNode = graph1.findClosestNode(0, 0);
    Node* goalNode = graph1.findClosestNode(5, 0); 
    std::vector<Node*> path1 = graph1.bfs(startNode, goalNode);
    ASSERT_TRUE(path1.empty());

    path1 = graph1.dfs(startNode, goalNode);
    ASSERT_TRUE(path1.empty());

    path1 = graph1.dijkstra(startNode, goalNode);
    ASSERT_TRUE(path1.empty());

    path1 = graph1.aStar(startNode, goalNode);
    ASSERT_TRUE(path1.empty());

    
    Graph graph2 = createNetworkGraph();
    startNode = graph2.findClosestNode(0, 0);
    goalNode = graph2.findClosestNode(3, 3); 
    std::vector<Node*> path2 = graph2.bfs(startNode, goalNode);
    ASSERT_TRUE(path2.empty());

    path2 = graph2.dfs(startNode, goalNode);
    ASSERT_TRUE(path2.empty());

    path2 = graph2.dijkstra(startNode, goalNode);
    ASSERT_TRUE(path2.empty());

    path2 = graph2.aStar(startNode, goalNode);
    ASSERT_TRUE(path2.empty());
}


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
