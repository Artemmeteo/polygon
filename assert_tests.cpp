#include <iostream>
#include <cassert>
#include <cmath>
#include <graph_algorithms.cpp>

// Функция для тестирования
void testGraphAlgorithms() {
    Graph graph;

    // Создание графа 1
    Node* n1 = graph.addNode(0, 0);
    Node* n2 = graph.addNode(1, 0);
    Node* n3 = graph.addNode(1, 1);
    Node* n4 = graph.addNode(0, 1);
    Node* n5 = graph.addNode(2, 0);
    Node* n6 = graph.addNode(2, 1);
    Node* n7 = graph.addNode(3, 0);

    // Добавление рёбер для первого графа
    graph.addEdge(0, 0, 1, 0, 1); // n1 - n2
    graph.addEdge(1, 0, 1, 1, 1); // n2 - n3
    graph.addEdge(1, 1, 0, 1, 1); // n3 - n4
    graph.addEdge(0, 1, 0, 0, 1); // n4 - n1
    graph.addEdge(1, 0, 2, 0, 2); // n2 - n5
    graph.addEdge(2, 0, 2, 1, 1); // n5 - n6
    graph.addEdge(2, 1, 3, 0, 2); // n6 - n7

    // Задание координат для поиска пути
    double startLon = 0, startLat = 0; // n1
    double goalLon = 3, goalLat = 0;   // n7

    // Нахождение ближайших узлов для стартовой и целевой точки
    Node* startNode = graph.findClosestNode(startLon, startLat);
    Node* goalNode = graph.findClosestNode(goalLon, goalLat);

    // Тест BFS
    std::vector<Node*> bfsPath = graph.bfs(startNode, goalNode);
    assert(bfsPath.size() == 4); // Должен быть путь через 4 узла
    assert(bfsPath[0] == startNode); // Начало пути - n1
    assert(bfsPath.back() == goalNode); // Конец пути - n7

    // Тест DFS
    std::vector<Node*> dfsPath = graph.dfs(startNode, goalNode);
    assert(dfsPath.size() == 4); // Должен быть путь через 4 узла
    assert(dfsPath[0] == startNode); // Начало пути - n1
    assert(dfsPath.back() == goalNode); // Конец пути - n7

    // Тест Дейкстры
    std::vector<Node*> dijkstraPath = graph.dijkstra(startNode, goalNode);
    assert(dijkstraPath.size() == 4); // Должен быть путь через 4 узла
    assert(dijkstraPath[0] == startNode); // Начало пути - n1
    assert(dijkstraPath.back() == goalNode); // Конец пути - n7

    // Тест A*
    std::vector<Node*> aStarPath = graph.aStar(startNode, goalNode);
    assert(aStarPath.size() == 4); // Должен быть путь через 4 узла
    assert(aStarPath[0] == startNode); // Начало пути - n1
    assert(aStarPath.back() == goalNode); // Конец пути - n7

    std::cout << "Все тесты пройдены успешно!" << std::endl;
}

int main() {
    try {
        testGraphAlgorithms();
    } catch (const std::exception& e) {
        std::cerr << "Ошибка: " << e.what() << std::endl;
    }
    return 0;
}
