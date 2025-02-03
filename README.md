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

### Theoretical Explanation of the A* Algorithm

A* is a best-first search algorithm that is widely used for pathfinding and graph traversal. Its goal is to find the shortest path from a start node to a goal node efficiently by combining the benefits of Dijkstra’s algorithm and a greedy best-first search.

#### Key Concepts

1. **Cost Functions:**
   - **g(n):** The actual cost from the start node to the current node *n*.
   - **h(n):** A heuristic that estimates the cost from node *n* to the goal. In grid-based mazes, a common choice is the Manhattan distance (the sum of the absolute differences of the coordinates).
   - **f(n):** The total estimated cost of the path through *n*, defined as  
     \[
     f(n) = g(n) + h(n)
     \]
     The algorithm prioritizes nodes with the lowest *f(n)* value.

2. **Open and Closed Sets:**
   - **Open Set:** A collection (often implemented as a priority queue) of nodes that are candidates for exploration. Nodes in this set are prioritized by their *f(n)* value.
   - **Closed Set:** Nodes that have already been visited and processed, so they are not revisited.

3. **The Algorithm Steps:**
   - **Initialization:** 
     - Set the starting node’s *g* score to 0.
     - Compute its *f* score using the heuristic.
     - Insert the start node into the open set.
   - **Main Loop:**
     1. **Select the Node with Lowest f(n):** Remove the node from the open set that has the smallest *f(n)*.
     2. **Goal Test:** If this node is the goal, the algorithm terminates successfully.
     3. **Explore Neighbors:** For each neighbor of the current node:
        - Skip neighbors that are invalid (e.g., walls or outside the grid).
        - Compute a tentative *g* score by adding the cost from the current node to the neighbor (often a constant like 1 for grid movement).
        - If this tentative *g* score is lower than the neighbor’s recorded *g* score, update the neighbor’s *g* and *f* scores, record the current node as its parent, and add the neighbor to the open set.
   - **Path Reconstruction:** Once the goal is reached, reconstruct the path by backtracking from the goal node to the start node using the recorded parent pointers.

The heuristic guides the algorithm towards the goal, while the *g* cost ensures that the actual distance traveled is minimized.

---

### Code Walkthrough

Now, let’s break down your provided code step by step.

#### 1. Helper Function: `isValid`

```cpp
bool isValid(int x, int y, int rows, int cols, const std::vector<std::vector<int>>& maze) {
    return x >= 0 && y >= 0 && x < rows && y < cols && maze[x][y] == 0;
}
```

- **Purpose:**  
  Checks whether the cell `(x, y)` is within the maze bounds and is traversable (i.e., it is not a wall).  
- **Details:**  
  - `x >= 0 && y >= 0`: Ensures the cell is not outside the top or left boundaries.
  - `x < rows && y < cols`: Ensures the cell is not beyond the bottom or right boundaries.
  - `maze[x][y] == 0`: Assumes that `0` represents a free cell (where you can move) and any other value (e.g., `1`) represents an obstacle.

---

#### 2. Node Structure

```cpp
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
```

- **Purpose:**  
  Represents a cell or state in the maze.
- **Members:**
  - `x, y`: The coordinates of the cell.
  - `gScore`: The cost to reach this node from the start.
  - `fScore`: The estimated total cost (`gScore + h(n)`).
  - `parent`: Pointer to the parent node (useful for reconstructing the path).
- **Operator Overloading:**  
  The `operator>` is overloaded so that the node with the lower `fScore` has a higher priority in the priority queue (since by default, C++’s `std::priority_queue` is a max-heap; using `std::greater<>` reverses that behavior, effectively turning it into a min-heap).

---

#### 3. Main Function: `solveMazeAStar`

```cpp
json solveMazeAStar(const std::vector<std::vector<int>>& maze,
                    const std::pair<int, int>& start,
                    const std::pair<int, int>& end) {
```

- **Purpose:**  
  Uses the A* algorithm to find a path through the maze from `start` to `end`, then returns the visited nodes and the found path as JSON.

##### a. Initialization

```cpp
    int rows = maze.size(), cols = maze[0].size();
    auto heuristic = [&](int x, int y) { return std::abs(x - end.first) + std::abs(y - end.second); };

    std::vector<std::vector<int>> gScores(rows, std::vector<int>(cols, INT_MAX));
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
```

- **Maze Dimensions:**  
  `rows` and `cols` store the maze’s dimensions.
- **Heuristic Function:**  
  A lambda function calculates the Manhattan distance from any cell `(x, y)` to the goal `(end.first, end.second)`.
- **gScores:**  
  A 2D vector that holds the best-known cost from the start to each cell. It is initialized with `INT_MAX` (representing infinity) for all cells.
- **Parent Tracking:**  
  A 2D vector that records the parent (previous cell) for each cell. Initialized with `{-1, -1}` to indicate that no parent is assigned yet.

##### b. Priority Queue and Tracking Structures

```cpp
    std::priority_queue<Node, std::vector<Node>, std::greater<>> pq;

    std::vector<std::pair<int, int>> visitedNodes, path;
    std::unordered_map<int, int> bestFScore;  // Map to track best fScore per node
```

- **Priority Queue (`pq`):**  
  Stores nodes to explore, ordered by their `fScore` (lowest first).
- **Visited Nodes:**  
  A vector to keep track of the nodes that have been popped from the queue (useful for debugging or visualization).
- **Best fScore Map:**  
  An `unordered_map` used to quickly check if the node with a particular index (computed by `x * cols + y`) has been reached with a lower `fScore` already. This helps in ignoring outdated queue entries.

##### c. Starting Node Setup

```cpp
    gScores[start.first][start.second] = 0;
    int startF = heuristic(start.first, start.second);
    pq.emplace(start.first, start.second, 0, startF);
    bestFScore[start.first * cols + start.second] = startF;
```

- **Set gScore for Start:**  
  The starting cell’s `gScore` is set to 0.
- **Compute fScore for Start:**  
  The starting cell’s `fScore` is just its heuristic value (since `gScore` is 0).
- **Add to Priority Queue:**  
  The starting node is pushed into the priority queue.
- **Record in bestFScore Map:**  
  The computed `fScore` for the start is stored using its unique index (`x * cols + y`).

##### d. Define Movement Directions

```cpp
    std::vector<std::pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
```

- **Purpose:**  
  Defines the four possible movement directions (up, down, left, right) in the maze.

##### e. Main Loop of A*

```cpp
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
```

- **Extract the Best Candidate:**  
  The node with the lowest `fScore` is popped from the priority queue.
- **Outdated Entry Check:**  
  If a better `fScore` was recorded for this node (via `bestFScore`), the current node is skipped. This avoids processing nodes that have already been improved upon.
- **Mark Node as Visited:**  
  The current node’s coordinates are added to `visitedNodes` for debugging output.
- **Goal Check:**  
  If the current node matches the end coordinates, the algorithm exits the loop.
- **Neighbor Exploration:**
  - For each of the four directions, calculate the neighbor’s coordinates `(nx, ny)`.
  - Use `isValid` to ensure the neighbor is within bounds and not a wall.
  - **Tentative gScore:**  
    The cost to reach this neighbor from the start via the current node is computed (`gScores[x][y] + 1`). The cost increment is 1 because movement to an adjacent cell is assumed to have a uniform cost.
  - **Update If Better Path Found:**  
    If this new `tentativeGScore` is less than the previously recorded `gScore` for the neighbor:
    - Update the neighbor’s parent to the current node.
    - Set the neighbor’s `gScore` to `tentativeGScore`.
    - Calculate the neighbor’s new `fScore` by adding the heuristic.
    - Add the neighbor to the priority queue.
    - Update the `bestFScore` map with the new `fScore` for that neighbor.

##### f. Path Reconstruction

```cpp
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
```

- **Check for Success:**  
  If the `gScore` for the end cell is still `INT_MAX`, it means the end was not reached. Otherwise, we have a valid path.
- **Backtracking:**  
  Start from the end cell and follow the parent pointers back to the start cell. Each cell is added to the `path` vector.
- **Reversal:**  
  Since the path was built from the end to the start, it is reversed to present it from the start to the end.

##### g. Debugging Output and Return

```cpp
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
```

- **Printing Visited Nodes:**  
  The code prints out all the nodes that were explored during the search.
- **Printing the Path:**  
  The final computed path is printed.
- **Return Value:**  
  A JSON object containing two keys, `"visitedNodes"` and `"path"`, is returned. This can be useful for visualizing the search process and the final path.

---

### Summary

- **The A* Algorithm** combines the actual cost (`gScore`) from the start and a heuristic estimate (`h(n)`) to guide the search toward the goal efficiently.
- **The Code:**
  - Uses a helper function to ensure valid moves.
  - Defines a `Node` structure with necessary information (coordinates, scores, and parent pointer) and overloads the comparison operator for use in a min-heap.
  - Initializes necessary data structures, including the `gScores`, parent trackers, and a priority queue.
  - Iteratively processes nodes by selecting the one with the lowest `fScore`, updating neighboring nodes if a better path is found, and finally reconstructs the path once the goal is reached.
  - Provides debugging output by printing the nodes visited and the final path.

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