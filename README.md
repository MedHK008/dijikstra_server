### check the website via this link : 
[dijikstra](https://dijikstra.netlify.app/)


### Frontend is in this repo : 
[Vue_frontend](https://github.com/MedHK008/dijikstra_project)

# Maze Solving Algorithms: Dijkstra, A*, and BFS

This document provides an overview of three pathfinding algorithms implemented to solve mazes: **Dijkstra's Algorithm**, **A\* Search**, and **Breadth-First Search (BFS)**. Each section explains the theory behind the algorithm and details its implementation.

---

## 1. Dijkstra's Algorithm

### Theory
Dijkstra's Algorithm finds the shortest path between nodes in a graph with **non-negative edge weights**. It greedily explores the node with the smallest known distance from the start, updating distances to neighboring nodes as it progresses. For mazes (unweighted grids), it effectively becomes a search for the shortest path in terms of steps.

### Implementation Details
- **Data Structures**:
  - `priority_queue` to track nodes with the smallest current distance.
  - `dist` matrix to store the shortest distance from the start to each cell.
  - `parent` matrix to reconstruct the path.
  - `visitedNodes` and `path` vectors for visualization.

- **Steps**:
  1. Initialize the start node's distance to `0` and others to `INT_MAX`.
  2. Use a priority queue to process nodes in order of increasing distance.
  3. For each node, explore its neighbors (up, down, left, right). If moving to a neighbor reduces its distance, update the distance and parent.
  4. Terminate early if the end node is reached.
  5. Reconstruct the path by backtracking from the end node using the `parent` matrix.

- **Key Code**:
  ```cpp
  std::priority_queue<std::pair<int, std::pair<int, int>>, ...> pq;
  if (newDist < dist[nx][ny]) {
      dist[nx][ny] = newDist;
      parent[nx][ny] = {x, y};
      pq.emplace(newDist, {nx, ny});
  }
  ```

---

## 2. A* Search Algorithm

### Theory
A* enhances Dijkstra by incorporating a **heuristic function** (e.g., Manhattan distance) to guide the search toward the goal. It prioritizes nodes based on `fScore = gScore + hScore`, where:
- `gScore`: Cost from the start to the current node.
- `hScore`: Estimated cost from the current node to the goal (heuristic).

### Implementation Details
- **Data Structures**:
  - `priority_queue` of `Node` structs, sorted by `fScore`.
  - `gScores` matrix to track the actual cost from the start.
  - `bestFScore` map to ignore outdated nodes in the queue.

- **Steps**:
  1. Initialize the start node with `gScore = 0` and `fScore = heuristic(start)`.
  2. Process nodes by lowest `fScore`, updating `gScores` and parents for neighbors.
  3. Skip nodes with outdated `fScore` values using `bestFScore`.
  4. Reconstruct the path similarly to Dijkstra.

- **Heuristic**:
  ```cpp
  auto heuristic = [&](int x, int y) { 
      return std::abs(x - end.first) + std::abs(y - end.second); 
  };
  ```

- **Node Struct**:
  ```cpp
  struct Node {
      int x, y, gScore, fScore;
      Node* parent;
      // Comparator prioritizes lower fScore.
      bool operator>(const Node& other) const { return fScore > other.fScore; }
  };
  ```

---

## 3. Breadth-First Search (BFS)

### Theory
BFS explores all nodes at the current depth level before moving to the next. It guarantees the shortest path in **unweighted graphs** (like mazes) by visiting nodes in layers, using a FIFO queue.

### Implementation Details
- **Data Structures**:
  - `queue` to manage nodes in FIFO order.
  - `visited` matrix to track explored nodes.
  - `parent` matrix for path reconstruction.

- **Steps**:
  1. Mark the start node as visited and enqueue it.
  2. Dequeue nodes layer by layer, enqueuing valid unvisited neighbors (up, down, left, right).
  3. Stop when the end node is dequeued.
  4. Reconstruct the path using the `parent` matrix.

- **Key Code**:
  ```cpp
  std::queue<std::pair<int, int>> q;
  q.push(start);
  visited[start.first][start.second] = true;
  ```

---

## Common Features

### Direction Vectors
All algorithms use the same 4-directional movement:
```cpp
std::vector<std::pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
```

### Path Reconstruction
Paths are reconstructed by backtracking from the end node using a `parent` matrix:
```cpp
for (std::pair<int, int> at = end; at != {-1, -1}; at = parent[at.first][at.second]) {
    path.push_back(at);
}
std::reverse(path.begin(), path.end());
```

### Output
Each function returns a JSON object containing:
- `visitedNodes`: List of cells explored in order.
- `path`: Shortest path from start to end (if exists).
```cpp
return json{{"visitedNodes", visitedNodes}, {"path", path}};
```

---

## Performance Comparison
| Algorithm | Time Complexity       | Use Case                               |
|-----------|-----------------------|----------------------------------------|
| BFS       | O(V + E)              | Unweighted grids, shortest path in steps. |
| Dijkstra  | O((V + E) log V)      | Weighted graphs or grids.              |
| A*        | O((V + E) log V)      | Weighted/unweighted grids with heuristic guidance. |

In the context of graph algorithms like Dijkstra's, A*, and BFS, **`V`** and **`E`** refer to:  

- **`V` (Vertices)**: The total number of nodes/cells in the graph or maze.  
  - For a maze of size `rows × cols`, `V = rows * cols`.  

- **`E` (Edges)**: The total number of possible connections between nodes (i.e., valid moves between cells).  
  - In a grid-based maze, each cell has up to 4 edges (to its top, bottom, left, and right neighbors), assuming walls block some connections.  
  - If the maze is fully connected (no walls), `E ≈ 4V`.  

---



**Note**: Dijikstra and BFS wll have the same result because we are working with an unweighted grid(graph)