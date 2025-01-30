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
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    std::priority_queue<std::pair<int, std::pair<int, int>>, std::vector<std::pair<int, std::pair<int, int>>>, std::greater<>> pq;
    std::vector<std::pair<int, int>> visitedNodes, path;

    dist[start.first][start.second] = 0;
    pq.emplace(0, start);

    std::vector<std::pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!pq.empty()) {
        auto [d, curr] = pq.top();
        pq.pop();

        int x = curr.first, y = curr.second;
        if (x == end.first && y == end.second) break;
        if (dist[x][y] < d) continue;

        visitedNodes.emplace_back(x, y);

        for (const auto& dir : dirs) {
            int nx = x + dir.first, ny = y + dir.second;
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && maze[nx][ny] == 0) {
                int newDist = dist[x][y] + 1;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    parent[nx][ny] = {x, y};
                    pq.emplace(newDist, std::make_pair(nx, ny));
                }
            }
        }
    }

    // Reconstruct path
    for (std::pair<int, int> at = end; at != std::make_pair(-1, -1); at = parent[at.first][at.second]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    return json{{"visitedNodes", visitedNodes}, {"path", path}};
}

// **A* (A-Star) Algorithm**
json solveMazeAStar(const std::vector<std::vector<int>>& maze,
                    const std::pair<int, int>& start,
                    const std::pair<int, int>& end) {
    int rows = maze.size(), cols = maze[0].size();
    auto heuristic = [&](int x, int y) { return std::abs(x - end.first) + std::abs(y - end.second); };

    std::vector<std::vector<int>> gScores(rows, std::vector<int>(cols, INT_MAX));
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    std::priority_queue<std::tuple<int, int, int>, std::vector<std::tuple<int, int, int>>, std::greater<>> pq;
    std::vector<std::pair<int, int>> visitedNodes, path;

    gScores[start.first][start.second] = 0;
    pq.emplace(heuristic(start.first, start.second), start.first, start.second);

    std::vector<std::pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!pq.empty()) {
        auto [fScore, x, y] = pq.top();
        pq.pop();

        if (x == end.first && y == end.second) break;
        if (gScores[x][y] + heuristic(x, y) < fScore) continue;

        visitedNodes.emplace_back(x, y);

        for (const auto& dir : dirs) {
            int nx = x + dir.first, ny = y + dir.second;
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && maze[nx][ny] == 0) {
                int tentativeGScore = gScores[x][y] + 1;
                if (tentativeGScore < gScores[nx][ny]) {
                    gScores[nx][ny] = tentativeGScore;
                    parent[nx][ny] = {x, y};
                    pq.emplace(tentativeGScore + heuristic(nx, ny), nx, ny);
                }
            }
        }
    }

    // Reconstruct path
    for (std::pair<int, int> at = end; at != std::make_pair(-1, -1); at = parent[at.first][at.second]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    return json{{"visitedNodes", visitedNodes}, {"path", path}};
}

void set_cors_headers(Response& res) {
    res.set_header("Access-Control-Allow-Origin", "https://dijikstra.netlify.app");
    res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
}

int main() {
    Server svr;

    // **Dijkstra API Endpoint**
    svr.Post("/api/maze/dijkstra", [](const Request& req, Response& res) {
        try {
            auto body = json::parse(req.body);
            auto maze = body["maze"].get<std::vector<std::vector<int>>>();
            auto start = body["start"].get<std::vector<int>>();
            auto end = body["end"].get<std::vector<int>>();

            auto result = solveMazeDijkstra(maze, {start[0], start[1]}, {end[0], end[1]});
            res.set_content(result.dump(), "application/json");

            set_cors_headers(res);  // ✅ Add CORS headers manually
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json({{"error", e.what()}}).dump(), "application/json");
            set_cors_headers(res);  // ✅ Ensure error responses also have CORS headers
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

            set_cors_headers(res);  // ✅ Add CORS headers manually
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json({{"error", e.what()}}).dump(), "application/json");
            set_cors_headers(res);  // ✅ Ensure error responses also have CORS headers
        }
    });

    // **Handle OPTIONS Preflight Requests**
    svr.Options(R"(/api/maze/.*)", [](const Request&, Response& res) {
        set_cors_headers(res);
        res.status = 204;  // No content, successful preflight response
    });

    // **Start the Server**
    const char* port = std::getenv("PORT");
    int server_port = port ? std::stoi(port) : 8080;  // Default to 8080 if not set

    if (!svr.listen("0.0.0.0", server_port)) {
        std::cerr << "Error: Failed to start server on port " << server_port << "\n";
        return 1;
    }

    return 0;
}