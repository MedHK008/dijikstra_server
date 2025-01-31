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

// Helper function to check if a cell is valid
bool isValid(int x, int y, int rows, int cols, const std::vector<std::vector<int>>& maze) {
    return x >= 0 && y >= 0 && x < rows && y < cols && maze[x][y] == 0;
}

struct Node {
    int x, y, gScore, fScore;
    Node* parent;

    Node(int x, int y, int g, int f, Node* p = nullptr)
        : x(x), y(y), gScore(g), fScore(f), parent(p) {}

    // Comparator for priority queue (min-heap)
    bool operator>(const Node& other) const {
        return fScore > other.fScore;
    }
};

// **A* Algorithm**
json solveMazeAStar(const std::vector<std::vector<int>>& maze,
                    const std::pair<int, int>& start,
                    const std::pair<int, int>& end) {
    int rows = maze.size(), cols = maze[0].size();
    auto heuristic = [&](int x, int y) { return std::abs(x - end.first) + std::abs(y - end.second); };

    std::vector<std::vector<int>> gScores(rows, std::vector<int>(cols, INT_MAX));
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));

    // Priority queue using Node struct
    std::priority_queue<Node, std::vector<Node>, std::greater<>> pq;

    std::vector<std::pair<int, int>> visitedNodes, path;
    std::unordered_map<int, int> bestFScore;  // Map to track best fScore per node

    gScores[start.first][start.second] = 0;
    int startF = heuristic(start.first, start.second);
    pq.emplace(start.first, start.second, 0, startF);
    bestFScore[start.first * cols + start.second] = startF;

    std::vector<std::pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        int x = current.x, y = current.y;
        int fScore = current.fScore;

        // Ignore outdated fScores
        if (bestFScore.find(x * cols + y) != bestFScore.end() && fScore > bestFScore[x * cols + y]) 
            continue;

        visitedNodes.emplace_back(x, y);

        // Goal reached
        if (x == end.first && y == end.second) 
            break;

        for (const auto& dir : dirs) {
            int nx = x + dir.first;
            int ny = y + dir.second;

            if (!isValid(nx, ny, rows, cols, maze)) continue;

            int tentativeGScore = gScores[x][y] + 1;
            if (tentativeGScore < gScores[nx][ny]) {
                parent[nx][ny] = {x, y};
                gScores[nx][ny] = tentativeGScore;
                int newFScore = tentativeGScore + heuristic(nx, ny);

                pq.emplace(nx, ny, tentativeGScore, newFScore);
                bestFScore[nx * cols + ny] = newFScore;
            }
        }
    }

    // Reconstruct path
    if (gScores[end.first][end.second] != INT_MAX) {
        std::pair<int, int> current = end;
        while (current != start) {
            path.push_back(current);
            if (parent[current.first][current.second] == std::make_pair(-1, -1)) 
                break;  // Prevent out-of-bounds access
            current = parent[current.first][current.second];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
    }

    // Debugging Output
    std::cout << "Visited Nodes: ";
    for (const auto& node : visitedNodes) {
        std::cout << "(" << node.first << ", " << node.second << ") ";
    }
    std::cout << std::endl;

    std::cout << "Path: ";
    for (const auto& node : path) {
        std::cout << "(" << node.first << ", " << node.second << ") ";
    }
    std::cout << std::endl;

    return json{{"visitedNodes", visitedNodes}, {"path", path}};
}

// **Breadth-First Search (BFS) Algorithm**
json solveBFS(const std::vector<std::vector<int>>& maze,
              const std::pair<int, int>& start,
              const std::pair<int, int>& end) {
    int rows = maze.size(), cols = maze[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    std::queue<std::pair<int, int>> q;
    std::vector<std::pair<int, int>> visitedNodes, path;

    q.push(start);
    visited[start.first][start.second] = true;

    std::vector<std::pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        visitedNodes.emplace_back(x, y);

        if (x == end.first && y == end.second) break;

        for (const auto& dir : dirs) {
            int nx = x + dir.first, ny = y + dir.second;
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && maze[nx][ny] == 0 && !visited[nx][ny]) {
                visited[nx][ny] = true;
                parent[nx][ny] = {x, y};
                q.push({nx, ny});
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

     // **BFS API Endpoint**
    svr.Post("/api/maze/bfs", [](const Request& req, Response& res) {
        try {
            auto body = json::parse(req.body);
            auto maze = body["maze"].get<std::vector<std::vector<int>>>();
            auto start = body["start"].get<std::vector<int>>();
            auto end = body["end"].get<std::vector<int>>();

            auto result = solveBFS(maze, {start[0], start[1]}, {end[0], end[1]});
            res.set_content(result.dump(), "application/json");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json({{"error", e.what()}}).dump(), "application/json");
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