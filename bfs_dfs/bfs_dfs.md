# Graph Traversal: BFS and DFS

Graph traversal refers to the process of visiting all the vertices of a graph. Two popular traversal techniques are **Breadth-First Search (BFS)** and **Depth-First Search (DFS)**.

---

## 1. **Breadth-First Search (BFS)**

### Definition:
BFS is a graph traversal technique that explores all the vertices at the current depth level before moving on to the next depth level.

### Characteristics:
- Uses a **queue** (FIFO) data structure.
- Visits vertices in **layers** (or levels).
- Finds the shortest path (in terms of the number of edges) in an unweighted graph.

### Algorithm:
1. Start from a source vertex and mark it as visited.
2. Enqueue the source vertex.
3. While the queue is not empty:
   - Dequeue a vertex, process it, and visit all its unvisited neighbors.
   - Mark the neighbors as visited and enqueue them.

### Pseudocode:

```
while Q is not empty:
    vertex = Dequeue Q
    Process vertex
    for each neighbor of vertex:
        if neighbor is not visited:
            Mark neighbor as visited
            Enqueue neighbor into Q
```

A-B || C-D
**BFS starting from A**:
1. Start with A → `Visited: [A]`
2. Explore neighbors B and C → `Visited: [A, B, C]`
3. Explore D from B and C → `Visited: [A, B, C, D]`

**Order of traversal**: A → B → C → D

---

## 2. **Depth-First Search (DFS)**

### Definition:
DFS is a graph traversal technique that explores as far as possible along one branch before backtracking.

### Characteristics:
- Uses a **stack** (LIFO) or recursion.
- Traverses in a **depth-oriented** manner.
- Suitable for exploring all paths, detecting cycles, and topological sorting.

### Algorithm:
1. Start from a source vertex and mark it as visited.
2. Push the source vertex onto the stack (or use recursion).
3. While the stack is not empty:
   - Pop a vertex, process it, and visit its unvisited neighbors.
   - Push each unvisited neighbor onto the stack.

### Pseudocode:

```
while S is not empty:
    vertex = Pop S
    Process vertex
    for each neighbor of vertex:
        if neighbor is not visited:
            Mark neighbor as visited
            Push neighbor onto S
```

A-B || C-D
**DFS starting from A**:
1. Start with A → `Visited: [A]`
2. Explore neighbor B → `Visited: [A, B]`
3. Explore neighbor D → `Visited: [A, B, D]`
4. Backtrack and explore C → `Visited: [A, B, D, C]`

**Order of traversal**: A → B → D → C

---

## Comparison of BFS and DFS

| Feature                | BFS                          | DFS                          |
|------------------------|------------------------------|------------------------------|
| **Traversal Method**   | Level by level               | Depth-oriented               |
| **Data Structure Used**| Queue (FIFO)                | Stack (LIFO) or recursion    |
| **Shortest Path**      | Finds the shortest path      | Does not guarantee shortest path |
| **Space Complexity**   | \( O(V + E) \) for adjacency list | \( O(V) \) for recursion/stack |
| **Best For**           | Finding shortest paths, exploring layers | Exploring all paths, detecting cycles |

---

## Applications of BFS
1. Shortest path in unweighted graphs.
2. Social network analysis.
3. Finding connected components.

## Applications of DFS
1. Cycle detection.
2. Topological sorting.
3. Solving maze problems.

## C++ Code

```
#include <iostream>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

// Graph class using adjacency list representation
class Graph {
    int V;                     // Number of vertices
    vector<vector<int>> adj;   // Adjacency list

public:
    Graph(int vertices) : V(vertices) {
        adj.resize(V);
    }

    // Add an edge to the graph
    void addEdge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);  // For undirected graph
    }

    // BFS implementation
    void BFS(int start) {
        vector<bool> visited(V, false); // Visited array
        queue<int> q;                   // Queue for BFS

        visited[start] = true;          // Mark the start node as visited
        q.push(start);

        cout << "BFS Traversal: ";
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            cout << node << " ";

            // Visit all unvisited neighbors
            for (int neighbor : adj[node]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
        cout << endl;
    }

    // DFS implementation (using recursion)
    void DFS(int start) {
        vector<bool> visited(V, false); // Visited array
        cout << "DFS Traversal: ";
        DFSUtil(start, visited);
        cout << endl;
    }

private:
    // Helper function for DFS
    void DFSUtil(int node, vector<bool>& visited) {
        visited[node] = true;          // Mark the current node as visited
        cout << node << " ";

        // Visit all unvisited neighbors
        for (int neighbor : adj[node]) {
            if (!visited[neighbor]) {
                DFSUtil(neighbor, visited);
            }
        }
    }
};

// Main function
int main() {
    int vertices = 5;
    Graph g(vertices);

    // Add edges
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(1, 4);

    // Perform BFS and DFS
    g.BFS(0);
    g.DFS(0);

    return 0;
}
```

