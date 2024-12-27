#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <limits>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include <algorithm>
#include <chrono> 

// Структура, представляющая узел графа
struct Node {
    double lon, lat; // Долгота и широта узла
    std::vector<std::pair<Node*, double>> neighbors; // Список смежных узлов и весов рёбер
};

// Класс с методами чтения файла, добавления узлов и ребер
// Также в этом же классе реализовал и алгоритмы 
// У себя в работе буду делать неориентированный граф
class Graph {
public:
    // Метод для добавления узла в граф
    Node* addNode(double lon, double lat) {
        // Создаем ключ для узла по его координатам для хранения
        std::string key = generateKey(lon, lat);
        // Если еще не существет ключа с таким значением вершины(узла) то создадим новый
        if (nodesMap.find(key) == nodesMap.end()) {
            Node* newNode = new Node{lon, lat}; // Создание нового узла
            nodesMap[key] = newNode; // Добавляем в мапку узлов
            nodes.push_back(newNode); // Добавляем узел в список всех узлов
            return newNode;
        }
        // возвращаем узел если есть (появился)
        return nodesMap[key];
    }

    // Метод для добавления рёбер между двумя узлами
    void addEdge(double lon1, double lat1, double lon2, double lat2, double weight) {
        // создание ключей для двух узлов
        std::string key1 = generateKey(lon1, lat1);
        std::string key2 = generateKey(lon2, lat2);

        // Проверка на наличие в графе
        if (nodesMap.find(key1) != nodesMap.end() && nodesMap.find(key2) != nodesMap.end()) {
            Node* node1 = nodesMap[key1];
            Node* node2 = nodesMap[key2];
            // добавляю ребра в обе стороны, тк делаю неориентированный граф
            node1->neighbors.emplace_back(node2, weight);
            node2->neighbors.emplace_back(node1, weight); // Обратное ребро
        }
    }

    // метод для чтения файла (spb_graph.txt)
    void readFromFile(const std::string& filename) {
        std::ifstream file(filename);
        // Проверка, что файл открылся
        if (!file.is_open()) {
            std::cerr << "Не удалось открыть файл: " << filename << "\n";
            return;
        }

        std::string line;
        // читаем построчно 
        while (std::getline(file, line)) {
            std::istringstream lineStream(line);
            std::string parentData, edgesData;

            // разделяем по знаку ":" родительский узел и смежные сним
            if (std::getline(lineStream, parentData, ':')) {
                // заменяем запятые на пробелы
                std::replace(parentData.begin(), parentData.end(), ',', ' ');
                std::istringstream parentStream(parentData);

                double lon1, lat1;
                // чтение долготы и широты
                if (!(parentStream >> lon1 >> lat1)) {
                    std::cerr << "[ERROR] Ошибка парсинга узла: " << parentData << "\n";
                    continue; // Переход к следующей строке
                }
                // Добавляем родительский узел
                Node* parentNode = addNode(lon1, lat1);

                // чтение смежных ребер, которые идут через знак ";"
                while (std::getline(lineStream, edgesData, ';')) {
                    // разделяем строку по символу
                    std::replace(edgesData.begin(), edgesData.end(), ',', ' ');
                    std::istringstream edgeStream(edgesData);

                    double lon2, lat2, weight;
                    if (!(edgeStream >> lon2 >> lat2 >> weight)) {
                        std::cerr << "[ERROR] Ошибка парсинга ребра: " << edgesData << "\n";
                        continue;
                    }
                    // добавляем соседний узел и ребро
                    Node* childNode = addNode(lon2, lat2);
                    addEdge(lon1, lat1, lon2, lat2, weight);
                }
            }
        }
        file.close(); // закрываем файл 
    }

    // метод для поиска ближайшего узла к заданным координатам
    // этот метод уже был реализован
    Node* findClosestNode(double lon, double lat) const {
        double minDist = std::numeric_limits<double>::max(); 
        Node* closest = nullptr; 

        for (Node* node : nodes) {
            double dx = node->lon - lon; 
            double dy = node->lat - lat; 
            double dist = std::sqrt(dx * dx + dy * dy); 
            if (dist < minDist) {
                minDist = dist;
                closest = node; 
            }
        }
        return closest;
    }

    // Метод для вывода найденного пути
    void printPath(const std::vector<Node*>& path) {
        if (path.empty()) {
            std::cout << "Путь не найден" << std::endl; // если пустой выводим это
            return;
        }
        double totalDistance = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i]->lon << "," << path[i]->lat; // вывод координат через запятую
            if (i != path.size() - 1) {
                // Находим вес ребра между текущим и следующим узлом (для подсчета пути)
                for (const auto& neighbor : path[i]->neighbors) {
                    if (neighbor.first == path[i + 1]) {
                        totalDistance += neighbor.second;
                        break;
                    }
                }
                std::cout << " -> "; // между координатами выводим стрелочку
            }
        }
        std::cout << "\nДлина пути: " << totalDistance << std::endl; // вывод длины пути
    }

    // Алгоритм поиска в ширину (BFS)
    // реализуется через использование очереди (queue)
    std::vector<Node*> bfs(Node* start, Node* goal) {

        std::unordered_map<Node*, Node*> cameFrom; // мапка для дальнейшего восстан пути
        std::queue<Node*> toVisit; // инициализируем очередь для обхода
        toVisit.push(start); // добавляем начальный узел в очередь (с которого начинаем)
        cameFrom[start] = nullptr; // у него нет родителя

        while (!toVisit.empty()) {
            Node* current = toVisit.front(); // получаем текущий узел
            toVisit.pop();

            if (current == goal) {
                return reconstructPath(cameFrom, goal); // если дошли до нужного нам узла, то победа - восстанавливаем путь 
            }

            for (const auto& neighbor : current->neighbors) {
                if (cameFrom.find(neighbor.first) == cameFrom.end()) {
                    cameFrom[neighbor.first] = current; // родитель отправляется в путь 
                    toVisit.push(neighbor.first); // смежные вершины в очередь
                }
            }
        }

        return {}; // если нет пути 
    }

    // Алгоритм поиска в глубину (DFS)
    // реализуется через стэк (stack)
    std::vector<Node*> dfs(Node* start, Node* goal) {

        std::unordered_map<Node*, Node*> cameFrom; // мапка для дальнейшего восстан пути
        std::stack<Node*> toVisit; // инициализируем cтек для обхода
        toVisit.push(start); // данный узел в очередь
        cameFrom[start] = nullptr; // у начального нет родителя

        while (!toVisit.empty()) {
            Node* current = toVisit.top(); // текщуий узел
            toVisit.pop();

            if (current == goal) {
                return reconstructPath(cameFrom, goal); // если дошли до нужного, то победа - восстан путь
            }

            for (const auto& neighbor : current->neighbors) {
                if (cameFrom.find(neighbor.first) == cameFrom.end()) {
                    cameFrom[neighbor.first] = current; // родитель в путь
                    toVisit.push(neighbor.first); // сосед в стэк
                }
            }
        }

        return {}; // если нет пути
    }

    // Алгоритм Дейкстры для нахождения кратчайшего пути
    // реализуется через приоритетную очередь (так скажем отсортированная очередь)
    // так как на каждом шаге нужно оптимальное решение (greedy)
    std::vector<Node*> dijkstra(Node* start, Node* goal) {
        
        // мапки для расстояний и родителей
        std::unordered_map<Node*, double> distances;
        std::unordered_map<Node*, Node*> cameFrom;
        // приоритетная очередь
        auto cmp = [&distances](Node* a, Node* b) {
            return distances[a] > distances[b];
        };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> toVisit(cmp);

        // инит расстония
        for (Node* node : nodes) {
            distances[node] = std::numeric_limits<double>::max(); // огромное расстояние (бесконечность) можно просто большим числом
        }
        distances[start] = 0; // у начального расстояние 0 тк в нем и находимся
        toVisit.push(start);

        while (!toVisit.empty()) {
            Node* current = toVisit.top(); // получаем узел с минимальным расстоянием
            toVisit.pop();

            if (current == goal) {
                return reconstructPath(cameFrom, goal); // дошли до нужного - победа, нужно восстан путь
            }

            // смотрим все смежные вершины
            for (const auto& neighbor : current->neighbors) {
                double newDist = distances[current] + neighbor.second; // вычисляем новое расстояние до соседа
                if (newDist < distances[neighbor.first]) {
                    distances[neighbor.first] = newDist;
                    cameFrom[neighbor.first] = current; // предок в путь
                    toVisit.push(neighbor.first); // сосед в приор очередь
                }
            }
        }

        return {}; // не нашли путь 
    }

    // Алгоритм A* для нахождения кратчайшего пути с эвристикой
    // как дейкстра, но с эврстикой через евклидово расстояние
    std::vector<Node*> aStar(Node* start, Node* goal) {

        // мапки для хранения g-оценок, f-оценок и родителей 
        std::unordered_map<Node*, double> gScore, fScore;
        std::unordered_map<Node*, Node*> cameFrom;
        // инит приоритетная очередь
        auto cmp = [&fScore](Node* a, Node* b) {
            return fScore[a] > fScore[b];
        };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> toVisit(cmp);

        // инициализация g-оценок и f-оценок
        for (Node* node : nodes) {
            gScore[node] = std::numeric_limits<double>::max();
            fScore[node] = std::numeric_limits<double>::max();
        }
        gScore[start] = 0;
        fScore[start] = heuristic(start, goal); // f-оценка для начального узла
        toVisit.push(start);

        while (!toVisit.empty()) {
            Node* current = toVisit.top(); // узел с минимальной f-оценкой
            toVisit.pop();

            if (current == goal) {
                return reconstructPath(cameFrom, goal); // дошли до нужного - победа, восстан путь
            }

            for (const auto& neighbor : current->neighbors) {
                double tentativeGScore = gScore[current] + neighbor.second; // g-щценка для соседа
                if (tentativeGScore < gScore[neighbor.first]) {
                    cameFrom[neighbor.first] = current; // предок в путь 
                    gScore[neighbor.first] = tentativeGScore;
                    fScore[neighbor.first] = tentativeGScore + heuristic(neighbor.first, goal); // вычисляем f-оценку
                    toVisit.push(neighbor.first); // сосед в приор очередь
                }
            }
        }

        return {}; // не нашли путь
    }

    // для освобождения памяти от графа
    ~Graph() {
        for (Node* node : nodes) {
            delete node; // Удаляем все узлы
        }
    }

private:
    std::vector<Node*> nodes; // Список всех узлов в графе
    std::unordered_map<std::string, Node*> nodesMap; // мапка для поиска узлов по ключам (координатам)

    // создание ключа для узла по координатам
    std::string generateKey(double lon, double lat) {
        std::ostringstream oss;
        oss << lon << "," << lat;
        return oss.str();
    }

    // евклидова эвристика для алгоритма A*
    static double heuristic(Node* a, Node* b) {
        double dx = a->lon - b->lon;
        double dy = a->lat - b->lat;
        return std::sqrt(dx * dx + dy * dy); // вычисление расстояния между двумя узлами
    }

    // Восстановление пути по мапке предков
    std::vector<Node*> reconstructPath(const std::unordered_map<Node*, Node*>& cameFrom, Node* current) {
        std::vector<Node*> path; // вектор для хранения пути
        while (current != nullptr) {
            path.push_back(current); // добавляем текущий узел в путь
            if (cameFrom.find(current) == cameFrom.end()) {
                break; // если нет предка, останавливаем восстановление
            }
            current = cameFrom.at(current); // переход к родителю
        }
        std::reverse(path.begin(), path.end()); // реверсаем путь 
        return path;
    }
};

// Основная функция
int main() {
    Graph graph;

    try {
        graph.readFromFile("spb_graph.txt"); // Чтение графа из файла

        double startLon = 30.480101;
        double startLat = 59.951809;
        double goalLon = 30.307004;
        double goalLat = 59.956078;

        // Поиск ближайших узлов к заданным координатам
        Node* startNode = graph.findClosestNode(startLon, startLat);
        Node* goalNode = graph.findClosestNode(goalLon, goalLat);

        // Замер времени и выполнение алгоритмов поиска
        auto start_time = std::chrono::high_resolution_clock::now();
        std::cout << "BFS путь: ";
        graph.printPath(graph.bfs(startNode, goalNode)); // Алгоритм BFS
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> bfs_duration = end_time - start_time;
        std::cout << "Время BFS: " << bfs_duration.count() << " секунд\n";

        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "DFS путь: ";
        graph.printPath(graph.dfs(startNode, goalNode)); // Алгоритм DFS
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dfs_duration = end_time - start_time;
        std::cout << "Время DFS: " << dfs_duration.count() << " секунд\n";

        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "Дейкстра путь: ";
        graph.printPath(graph.dijkstra(startNode, goalNode)); // Алгоритм Дейкстры
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dijkstra_duration = end_time - start_time;
        std::cout << "Время Дейкстры: " << dijkstra_duration.count() << " секунд\n";

        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "A* путь: ";
        graph.printPath(graph.aStar(startNode, goalNode)); // Алгоритм A*
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> astar_duration = end_time - start_time;
        std::cout << "Время A*: " << astar_duration.count() << " секунд\n";

    } catch (const std::exception& e) {
        std::cerr << "Ошибка: " << e.what() << std::endl;
    }

    return 0;
}
