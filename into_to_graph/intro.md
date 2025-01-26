# Graphs in Data Structures

## What is a Graph?
A **graph** is a non-linear data structure consisting of **vertices (or nodes)** and **edges (or arcs)**. It is used to represent relationships between pairs of objects. Formally, a graph is defined as \( G = (V, E) \), where:

- \( V \): A set of vertices.
- \( E \): A set of edges connecting the vertices.

Graphs can represent various real-world problems like social networks, computer networks, or city maps.

---

## Types of Graphs

### 1. **Based on the Direction of Edges**
#### a) **Undirected Graph**
- Edges have no direction.
- If there is an edge between two vertices \( u \) and \( v \), it can be traversed both ways.
- **Example**: Social networks where friendships are mutual.

#### b) **Directed Graph (Digraph)**
- Edges have a specific direction.
- If there is an edge from \( u \) to \( v \), it can only be traversed in one direction.
- **Example**: Twitter, where a user can follow another user without mutuality.

---

### 2. **Based on the Weight of Edges**
#### a) **Weighted Graph**
- Edges are assigned weights or costs.
- Used in scenarios like road maps where weights represent distances or costs.
- **Example**: A map with distances between cities.

#### b) **Unweighted Graph**
- Edges are not assigned weights.
- All edges are considered equal in cost or significance.
- **Example**: A simple network of computers.

---

### 3. **Based on the Presence of Cycles**
#### a) **Cyclic Graph**
- Contains at least one cycle, i.e., a path where the starting vertex is also the ending vertex.
- **Example**: A network with loops.

#### b) **Acyclic Graph**
- Does not contain any cycles.
- A **Directed Acyclic Graph (DAG)** is often used in scheduling and dependency graphs.

---

### 4. **Special Types of Graphs**
#### a) **Connected Graph**
- There exists a path between every pair of vertices.
- **Example**: A fully connected network.

#### b) **Disconnected Graph**
- Some vertices are not connected by any path.
- **Example**: Islands in a network.

#### c) **Complete Graph**
- Every pair of distinct vertices is connected by a unique edge.
- Denoted as \( K_n \), where \( n \) is the number of vertices.

#### d) **Bipartite Graph**
- Vertices can be divided into two disjoint sets \( U \) and \( V \) such that every edge connects a vertex in \( U \) to one in \( V \).
- **Example**: Matching problems.

#### e) **Subgraph**
- A subset of a graph's vertices and edges that forms a graph.

---

### 5. **Planar Graph**
- Can be drawn on a plane without any edges crossing.
- **Example**: Road networks.

---

### 6. **Sparse vs. Dense Graph**
#### a) **Sparse Graph**
- Number of edges is much less than the maximum possible edges.
- **Example**: Tree structures.

#### b) **Dense Graph**
- Number of edges is close to the maximum possible edges.
- **Example**: Fully connected social networks.

---

## Graph Representation
Graphs can be represented in two main ways:
1. **Adjacency Matrix**:
   - A 2D array where \( A[i][j] = 1 \) if there is an edge between vertex \( i \) and \( j \); otherwise, \( 0 \).

2. **Adjacency List**:
   - An array of lists where each list represents the neighbors of a vertex.

---

## Example of a Graph
### Undirected Graph:
Vertices: \( V = \{A, B, C, D\} \)  
Edges: \( E = \{(A, B), (A, C), (B, D), (C, D)\} \)

# Graph Representation

Graphs can be represented in two primary ways: **Adjacency Matrix** and **Adjacency List**. These representations are chosen based on the specific use case and efficiency requirements.

---

## 1. **Adjacency Matrix**
An adjacency matrix is a 2D array where the rows and columns represent the vertices of the graph, and the entries indicate the presence or absence of edges.

### Structure:
- For an **unweighted graph**, the matrix contains:
  - `1` if there is an edge between the vertices.
  - `0` otherwise.
- For a **weighted graph**, the matrix contains:
  - The weight of the edge if an edge exists.
  - `0` or `∞` (infinity) if no edge exists.

### Characteristics:
- Suitable for **dense graphs** (graphs with many edges).
- Space complexity: \( O(V^2) \), where \( V \) is the number of vertices.


---

## 2. **Adjacency List**
An adjacency list is an array of lists where each index represents a vertex, and the list at that index contains all the vertices it is connected to.

### Structure:
- Each vertex is associated with a list of its neighbors.
- For a **weighted graph**, each entry in the list also includes the weight of the edge.

### Characteristics:
- Suitable for **sparse graphs** (graphs with fewer edges).
- Space complexity: \( O(V + E) \), where \( E \) is the number of edges.


---

## Comparison of Adjacency Matrix and Adjacency List

| Feature               | Adjacency Matrix           | Adjacency List              |
|-----------------------|----------------------------|-----------------------------|
| **Space Complexity**  | \( O(V^2) \)              | \( O(V + E) \)              |
| **Edge Lookup**       | \( O(1) \)                | \( O(k) \) (where \( k \) is the degree of the vertex) |
| **Best for**          | Dense Graphs              | Sparse Graphs               |
| **Implementation**    | Simple for weighted graphs| More efficient for traversal|

---

## Conclusion
The choice between an adjacency matrix and an adjacency list depends on the graph's density and the operations required:
- Use **adjacency matrix** for dense graphs or when constant-time edge lookups are needed.
- Use **adjacency list** for sparse graphs or when minimizing space usage is critical.



