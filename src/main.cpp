#ifdef _WIN32
#include <winsock2.h>
#endif
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>

using json = nlohmann::json;
using namespace httplib;

struct Node {
    int x, y, distance, cost;
    Node* parent;

    Node(int x, int y, int d, int c, Node* p = nullptr)
        : x(x), y(y), distance(d), cost(c), parent(p) {}

    // Comparator for priority queue (min-heap)
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

// **Dijkstra's Algorithm**
json solveMazeDijkstra(const std::vector<std::vector<int>>& maze,
                       const std::pair<int, int>& start,
                       const std::pair<int, int>& end) {
    int rows = maze.size(), cols = maze[0].size();
    std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INT_MAX));
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::pair<int, int>> visitedNodes, path;

    dist[start.first][start.second] = 0;
    pq.emplace(start.first, start.second, 0, 0);

    std::vector<std::pair<int, int>> dirs = {{-1,0}, {1,0}, {0,-1}, {0,1}};

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (visited[current.x][current.y]) continue;
        visited[current.x][current.y] = true;
        visitedNodes.emplace_back(current.x, current.y);

        if (current.x == end.first && current.y == end.second) {
            Node* node = &current;
            while (node) {
                path.emplace_back(node->x, node->y);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        for (const auto& dir : dirs) {
            int nx = current.x + dir.first, ny = current.y + dir.second;
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols &&
                !visited[nx][ny] && maze[nx][ny] == 0) {
                int newDist = current.distance + 1;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    pq.emplace(nx, ny, newDist, 0, new Node(current));
                }
            }
        }
    }

    return json{{"visitedNodes", visitedNodes}, {"path", path}};
}

// **A* (A-Star) Algorithm**
json solveMazeAStar(const std::vector<std::vector<int>>& maze,
                    const std::pair<int, int>& start,
                    const std::pair<int, int>& end) {
    int rows = maze.size(), cols = maze[0].size();
    auto heuristic = [&](int x, int y) { return std::abs(x - end.first) + std::abs(y - end.second); };

    std::vector<std::vector<int>> gScores(rows, std::vector<int>(cols, INT_MAX));
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::pair<int, int>> visitedNodes, path;

    gScores[start.first][start.second] = 0;
    pq.emplace(start.first, start.second, 0, heuristic(start.first, start.second));

    std::vector<std::pair<int, int>> dirs = {{-1,0}, {1,0}, {0,-1}, {0,1}};

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        if (visited[current.x][current.y]) continue;
        visited[current.x][current.y] = true;
        visitedNodes.emplace_back(current.x, current.y);

        if (current.x == end.first && current.y == end.second) {
            Node* node = &current;
            while (node) {
                path.emplace_back(node->x, node->y);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        for (const auto& dir : dirs) {
            int nx = current.x + dir.first, ny = current.y + dir.second;
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols &&
                !visited[nx][ny] && maze[nx][ny] == 0) {
                int tentativeGScore = gScores[current.x][current.y] + 1;
                if (tentativeGScore < gScores[nx][ny]) {
                    gScores[nx][ny] = tentativeGScore;
                    int fScore = tentativeGScore + heuristic(nx, ny);
                    pq.emplace(nx, ny, tentativeGScore, fScore, new Node(current));
                }
            }
        }
    }

    return json{{"visitedNodes", visitedNodes}, {"path", path}};
}

// **Main Server**
int main() {
    Server svr;

    // CORS Setup
    svr.set_default_headers({
        {"Access-Control-Allow-Origin", "https://dijikstra.netlify.app"},
        {"Access-Control-Allow-Methods", "POST, OPTIONS"},
        {"Access-Control-Allow-Headers", "Content-Type"}
    });

    // **Dijkstra API Endpoint**
    svr.Post("/api/maze/dijkstra", [](const Request& req, Response& res) {
        try {
            auto body = json::parse(req.body);
            auto maze = body["maze"].get<std::vector<std::vector<int>>>();
            auto start = body["start"].get<std::vector<int>>();
            auto end = body["end"].get<std::vector<int>>();

            auto result = solveMazeDijkstra(maze, {start[0], start[1]}, {end[0], end[1]});
            res.set_content(result.dump(), "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json({{"error", e.what()}}).dump(), "application/json");
        }
    });

    // **A* API Endpoint**
    svr.Post("/api/maze/astar", [](const Request& req, Response& res) {
        try {
            auto body = json::parse(req.body);
            auto maze = body["maze"].get<std::vector<std::vector<int>>>();
            auto start = body["start"].get<std::vector<int>>();
            auto end = body["end"].get<std::vector<int>>();

            auto result = solveMazeAStar(maze, {start[0], start[1]}, {end[0], end[1]});
            res.set_content(result.dump(), "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json({{"error", e.what()}}).dump(), "application/json");
        }
    });

    // OPTIONS Handler
    svr.Options("/api/maze/dijkstra", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "https://dijikstra.netlify.app");
        res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.status = 200;  // Successful preflight response
    });


    const char* port = std::getenv("PORT");
    int server_port = port ? std::stoi(port) : 8080;  // Default to 8080 if not set

    if (!svr.listen("0.0.0.0", server_port)) {
        std::cerr << "Error: Failed to start server on port " << server_port << "\n";
        return 1;
    }

    return 0;
}
