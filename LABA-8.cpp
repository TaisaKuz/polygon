#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <limits>

struct Node {
    int id;
    double lon, lat;
    std::vector<std::pair<Node*, double>> neighbors;

    // �����������: �������� ����
    // ������: O(1) ��� �������� ���������, ID, � ������ �� �������
    Node(int id, double lon, double lat) : id(id), lon(lon), lat(lat) {}
};

struct Graph {
    std::vector<Node*> nodes; // ����, ��������� �� �����
    std::unordered_map<std::string, Node*> node_cache; // ��� ��� �������� ������ �����

    // ��������� ����� ��� �������� ����� � node_cache
    // �����: O(L), ��� L � ����� ������ ������������� ��������� (������� �� �������� double)
    // ������: O(L), ��� ��� ������ ����������� � ������
    std::string createKey(double lon, double lat) {
        return std::to_string(lon) + "," + std::to_string(lat); // O(L)
    }

    // ���������� ����� � ����
    // �����: O(1), ��� ��� �������� ���������� � ������ ������� ����������� �� ���������� �����
    // ������: O(1), ����������� ���� ����� ����� ����� ������
    void addEdge(Node* from, Node* to, double weight) {
        from->neighbors.push_back({ to, weight }); // O(1)
        to->neighbors.push_back({ from, weight }); // O(1)
    }

    // ����� ���������� ���� �� ������ ���������
    // �����: O(V), ��� ��� ��������� ��������� ��� ���� ��� ���������� ������������ ����������
    // ������: O(1), ���������� ����������� ������ ��� �������� ����������
    Node* findNearestNode(double lon, double lat) {
        double minDistance = std::numeric_limits<double>::max(); // O(1)
        Node* nearestNode = nullptr; // O(1)
        for (auto& node : nodes) {
            double distance = std::pow(node->lon - lon, 2) + std::pow(node->lat - lat, 2); // O(1)
            if (distance < minDistance) {
                minDistance = distance; // O(1)
                nearestNode = node; // O(1)
            }
        }
        return nearestNode; // O(1)
    }

    // ������ ����� �� �����
    // �����: O(E), ��� E � ���������� ���� (������ ����� ����������� �� ������ ����)
    // ������: O(V + E), ��� V � ���������� �����, � E � ���������� ����
    void parseFromFile(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            char separator;
            double lon, lat, weight;
            iss >> lon >> separator >> lat >> separator >> weight; // O(1)
            Node* node = findNearestNode(lon, lat);
            if (!node) {
                node = new Node(nodes.size(), lon, lat); // O(1)
                nodes.push_back(node); // O(1)
                node_cache[createKey(lon, lat)] = node; // O(1)
            }

            while (iss >> lon >> separator >> lat >> separator >> weight) {
                Node* neighborNode = findNearestNode(lon, lat); // O(V)
                if (!neighborNode) {
                    neighborNode = new Node(nodes.size(), lon, lat); // O(1)
                    nodes.push_back(neighborNode); // O(1)
                    node_cache[createKey(lon, lat)] = neighborNode; // O(1)
                }
                addEdge(node, neighborNode, weight); // O(1)
            }
        }
    }

    // �������� ������ � ������� (DFS)
    // �����: O(V + E), ��� ��� �� �������� ��� ������� (V) � ���� (E) �����
    // ������: O(V), ��� ��� �� ������ ���� � ����� ��� �������� ����������
    double dfs(Node* start, Node* end) {
        std::stack<Node*> s; // O(1)
        std::unordered_map<Node*, double> distances; // O(V)
        std::unordered_set<Node*> visited; // O(V)
        s.push(start); // O(1)
        distances[start] = 0.0; // O(1)

        while (!s.empty()) {
            Node* current = s.top(); // O(1)
            s.pop(); // O(1)

            if (visited.count(current)) continue; // O(1)
            visited.insert(current); // O(1)

            if (current == end) {
                return distances[current]; // O(1)
            }

            for (const auto& neighbor : current->neighbors) {
                double newDistance = distances[current] + neighbor.second; // O(1)
                if (distances.find(neighbor.first) == distances.end() || distances[neighbor.first] > newDistance) {
                    distances[neighbor.first] = newDistance; // O(1)
                    s.push(neighbor.first); // O(1)
                }
            }
        }
        return -1.0; // O(1)
    }

    // �������� ������ � ������ (BFS)
    // �����: O(V + E), ��� ��� �� �������� ��� ������� (V) � ���� (E) �����
    // ������: O(V), ��� ��� �� ������ ������� � ����� ��� �������� ����������
    double bfs(Node* start, Node* end) {
        std::queue<Node*> q; // O(1)
        std::unordered_map<Node*, double> distances; // O(V)
        std::unordered_set<Node*> visited; // O(V)
        q.push(start); // O(1)
        distances[start] = 0.0; // O(1)

        while (!q.empty()) {
            Node* current = q.front(); // O(1)
            q.pop(); // O(1)

            if (visited.count(current)) continue; // O(1)
            visited.insert(current); // O(1)

            if (current == end) {
                return distances[current]; // O(1)
            }

            for (const auto& neighbor : current->neighbors) {
                if (!visited.count(neighbor.first)) { // O(1)
                    distances[neighbor.first] = distances[current] + neighbor.second; // O(1)
                    q.push(neighbor.first); // O(1)
                }
            }
        }
        return -1.0; // O(1)
    }

    // �������� ��������
    // �����: O(V log V + E log V), ��� ��� ������������ ������������ ������� (heap)
    // ������: O(V + E), ��� ��� �� ������ ����� ��� ���������� � �������
    double dijkstra(Node* start, Node* end) {
        std::priority_queue<std::pair<double, Node*>, std::vector<std::pair<double, Node*>>, std::greater<>> pq; // O(log V)
        std::unordered_map<Node*, double> distances; // O(V)
        std::unordered_set<Node*> visited; // O(V)
        pq.push({ 0.0, start }); // O(log V)
        distances[start] = 0.0; // O(1)

        while (!pq.empty()) {
            Node* current = pq.top().second; // O(1)
            pq.pop(); // O(log V)

            if (visited.count(current)) continue; // O(1)
            visited.insert(current); // O(1)

            if (current == end) {
                return distances[current]; // O(1)
            }

            for (const auto& neighbor : current->neighbors) {
                double newDistance = distances[current] + neighbor.second; // O(1)
                if (distances.find(neighbor.first) == distances.end() || distances[neighbor.first] > newDistance) {
                    distances[neighbor.first] = newDistance; // O(1)
                    pq.push({ newDistance, neighbor.first }); // O(log V)
                }
            }
        }
        return -1.0; // O(1)
    }

    // �������� A* (� �������������� ���������)
    // �����: O(V log V + E log V), ���������� ��������, �� � ����������� ���������
    // ������: O(V + E), ��� � � ��������
    double aStar(Node* start, Node* end) {
        std::priority_queue<std::pair<double, Node*>, std::vector<std::pair<double, Node*>>, std::greater<>> pq; // O(log V)
        std::unordered_map<Node*, double> distances; // O(V)
        std::unordered_set<Node*> visited; // O(V)
        pq.push({ 0.0, start }); // O(log V)
        distances[start] = 0.0; // O(1)

        while (!pq.empty()) {
            Node* current = pq.top().second; // O(1)
            pq.pop(); // O(log V)

            if (visited.count(current)) continue; // O(1)
            visited.insert(current); // O(1)

            if (current == end) {
                return distances[current]; // O(1)
            }

            for (const auto& neighbor : current->neighbors) {
                double newDistance = distances[current] + neighbor.second; // O(1)
                if (distances.find(neighbor.first) == distances.end() || distances[neighbor.first] > newDistance) {
                    distances[neighbor.first] = newDistance; // O(1)
                    pq.push({ newDistance, neighbor.first }); // O(log V)
                }
            }
        }
        return -1.0; // O(1)
    }
};

int main() {
    Graph graph;

    // �������� ������ �� �����
    graph.parseFromFile("spb_graph.txt");

    // ������ ��������� ��� ������ ����
    double home_lon = 50.4501, home_lat = 30.5002;
    double university_lon = 50.4500, university_lat = 30.5100;

    Node* start = graph.findNearestNode(home_lon, home_lat); // O(V)
    Node* end = graph.findNearestNode(university_lon, university_lat); // O(V)

    if (start && end) {
        // ��������� ������� ��� DFS
        auto start_time = std::chrono::high_resolution_clock::now();
        double dfs_distance = graph.dfs(start, end); // O(V + E)
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dfs_duration = end_time - start_time;

        // ��������� ������� ��� BFS
        start_time = std::chrono::high_resolution_clock::now();
        double bfs_distance = graph.bfs(start, end); // O(V + E)
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> bfs_duration = end_time - start_time;

        // ��������� ������� ��� ��������
        start_time = std::chrono::high_resolution_clock::now();
        double dijkstra_distance = graph.dijkstra(start, end); // O(V log V + E log V)
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dijkstra_duration = end_time - start_time;

        // ��������� ������� ��� A*
        start_time = std::chrono::high_resolution_clock::now();
        double aStar_distance = graph.aStar(start, end); // O(V log V + E log V)
        end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> aStar_duration = end_time - start_time;

        // ����� �����������
        std::cout << "DFS Distance: " << dfs_distance << " (Time: " << dfs_duration.count() << " seconds)" << std::endl;
        std::cout << "BFS Distance: " << bfs_distance << " (Time: " << bfs_duration.count() << " seconds)" << std::endl;
        std::cout << "Dijkstra Distance: " << dijkstra_distance << " (Time: " << dijkstra_duration.count() << " seconds)" << std::endl;
        std::cout << "A* Distance: " << aStar_distance << " (Time: " << aStar_duration.count() << " seconds)" << std::endl;
    }
    else {
        std::cout << "Unable to find nodes for the provided coordinates." << std::endl;
    }

    return 0;
}