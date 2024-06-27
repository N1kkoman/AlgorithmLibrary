#include "../include/graph_algorithms.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <queue>
#include <stack>
#include <iostream>
#include <cmath> // for std::sqrt

namespace AlgorithmLibrary {
    namespace Graph {

        template<typename T>
        Graph<T>::Graph(int vertices) : vertices(vertices) {
            adjacencyMatrix.resize(vertices, std::vector<T>(vertices, 0));
        }

        template<typename T>
        Graph<T>::Graph(const std::vector<std::vector<T>>& adjacencyMatrix)
            : vertices(adjacencyMatrix.size()), adjacencyMatrix(adjacencyMatrix) {}

        template<typename T>
        void Graph<T>::addEdge(int v, int w, T weight) {
            // ��������� ����� ����� ��������� v � w � ����� weight
            adjacencyMatrix[v][w] = weight;
            adjacencyMatrix[w][v] = weight; // ��� ������������������ �����
        }

        template<typename T>
        std::vector<T> Graph<T>::bfs(int start) {
            // ���������� ��������� ������ � ������ (BFS)
            std::vector<bool> visited(vertices, false); // ������ ��� ������������ ���������� ������
            std::vector<T> result; // ������ ��� �������� ���������� ������
            std::queue<int> q; // ������� ��� ������ ������

            visited[start] = true; // �������� ��������� ������� ��� ����������
            q.push(start); // ��������� ��������� ������� � �������

            while (!q.empty()) {
                int v = q.front(); // ��������� ������� �� �������
                q.pop();
                result.push_back(v); // ��������� ������� � ��������� ������

                // �������� �� ���� ������� ��������
                for (int i = 0; i < vertices; ++i) {
                    // ���� ���� ����� ����� ������� �������� � ��������, � �������� ������� �� ��������
                    if (adjacencyMatrix[v][i] != 0 && !visited[i]) {
                        visited[i] = true; // �������� �������� ������� ��� ����������
                        q.push(i); // ��������� �������� ������� � �������
                    }
                }
            }

            return result;
        }

        template<typename T>
        std::vector<T> Graph<T>::dfs(int start) {
            // ���������� ��������� ������ � ������� (DFS)
            std::vector<bool> visited(vertices, false); // ������ ��� ������������ ���������� ������
            std::vector<T> result; // ������ ��� �������� ���������� ������
            std::stack<int> s; // ���� ��� ������ ������
            s.push(start); // ��������� ��������� ������� � ����

            while (!s.empty()) {
                int v = s.top(); // ��������� ������� �� �����
                s.pop();

                if (!visited[v]) {
                    visited[v] = true; // �������� ������� ��� ����������
                    result.push_back(v); // ��������� ������� � ��������� ������

                    // �������� �� ���� ������� �������� � �������� �������
                    for (int i = vertices - 1; i >= 0; --i) {
                        // ���� ���� ����� ����� ������� �������� � ��������, � �������� ������� �� ��������
                        if (adjacencyMatrix[v][i] != 0 && !visited[i]) {
                            s.push(i); // ��������� �������� ������� � ����
                        }
                    }
                }
            }

            return result;                      
        }

        template<typename T>
        std::vector<T> Graph<T>::dijkstra(int src) {
            // ���������� ��������� �������� ��� ������ ���������� ����� �� �������� �������
            std::vector<T> dist(vertices, std::numeric_limits<T>::max()); // ������ ��� �������� ���������� �� ������
            std::vector<bool> visited(vertices, false); // ������ ��� ������������ ���������� ������
            dist[src] = 0; // ���������� �� �������� ������� ����� 0

            for (int i = 0; i < vertices; ++i) {
                int u = -1;
                // ������� ������� � ����������� ����������� ����� ������������ ������
                for (int j = 0; j < vertices; ++j) {
                    if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                        u = j;
                    }
                }

                // ���� ���������� �� ������� u ����� �������������, �� ������� �� �����
                if (dist[u] == std::numeric_limits<T>::max()) break;

                visited[u] = true; // �������� ������� u ��� ����������

                // ��������� ���������� �� �������� ������
                for (int v = 0; v < vertices; ++v) {
                    // ���� ���� ����� ����� ��������� u � v, � v �� ��������,
                    // � ���������� �� v ����� u ������ �������� ���������� �� v
                    if (adjacencyMatrix[u][v] && !visited[v] && dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                        dist[v] = dist[u] + adjacencyMatrix[u][v]; // ��������� ���������� �� v
                    }
                }
            }

            return dist; // ���������� ������ ���������� �� ������
        }

        template<typename T>
        std::vector<T> Graph<T>::bellmanFord(int src) {
            // ������� ������ dist ��� �������� ���������� �� ������
            // �������������� ��� ���������� ������������ ��������� ���� T
            std::vector<T> dist(vertices, std::numeric_limits<T>::max());

            // ������������� ���������� �� �������� ������� ������ 0
            dist[src] = 0;

            // ��������� (vertices - 1) �������� ��������� ��������-�����
            for (int i = 1; i < vertices; ++i) {
                // ���������� �� ���� ������ �����
                for (int u = 0; u < vertices; ++u) {
                    for (int v = 0; v < vertices; ++v) {
                        // ���� ���������� ����� ����� ��������� u � v
                        // � ���������� �� ������� u �� ����������
                        // � ����� ���������� �� ������� v ������ ��������
                        if (adjacencyMatrix[u][v] && dist[u] != std::numeric_limits<T>::max() && dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                            // ��������� ���������� �� ������� v
                            dist[v] = dist[u] + adjacencyMatrix[u][v];
                        }
                    }
                }
            }

            // ��������� ������� ������������� ������
            for (int u = 0; u < vertices; ++u) {
                for (int v = 0; v < vertices; ++v) {
                    // ���� ���������� ����� ����� ��������� u � v
                    // � ���������� �� ������� u �� ����������
                    // � ����� ���������� �� ������� v ������ ��������
                    if (adjacencyMatrix[u][v] && dist[u] != std::numeric_limits<T>::max() && dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                        // ���� �������� ������������� ����
                        std::cerr << "Graph contains a negative-weight cycle" << std::endl;
                        // ���������� ������ ������, ����������� �� ������
                        return {};
                    }
                }
            }

            // ���������� ������ ���������� ���������� �� �������� ������� �� ���� ��������� ������
            return dist;
        }

        template<typename T>
        struct Edge {
            int src, dest;
            T weight;
            bool operator<(const Edge<T>& other) const {
                return weight < other.weight;
            }
        };

        template<typename T>
        int Graph<T>::find(std::vector<int>& parent, int i) {
            // ���� �������� ������� i �� ����� ����� ������� i
            if (parent[i] != i) {
                // ���������� ������� �������� ������� ��� ������� i � ��������� �������� ������� i
                parent[i] = find(parent, parent[i]);
            }
            // ���������� �������� (�������� �������) ��� ������� i
            return parent[i];
        }

        template<typename T>
        void Graph<T>::unionSets(std::vector<int>& parent, std::vector<int>& rank, int x, int y) {
            // ������� �������� ������� ��� ������ x � y
            int rootX = find(parent, x);
            int rootY = find(parent, y);

            // ���� ���� �������� ������� rootX ������ ����� �������� ������� rootY
            if (rank[rootX] < rank[rootY]) {
                // ������������� rootY � �������� �������� ��� rootX
                parent[rootX] = rootY;
            }
            // ���� ���� �������� ������� rootX ������ ����� �������� ������� rootY
            else if (rank[rootX] > rank[rootY]) {
                // ������������� rootX � �������� �������� ��� rootY
                parent[rootY] = rootX;
            }
            // ���� ����� �������� ������ �����
            else {
                // ������������� rootX � �������� �������� ��� rootY
                parent[rootY] = rootX;
                // ����������� ���� �������� ������� rootX �� 1
                rank[rootX]++;
            }
        }

        template<typename T>
        std::pair<T, std::vector<std::pair<int, int>>> Graph<T>::kruskal() {
            // ������� ������ ����� �����
            std::vector<Edge<T>> edges;
            // �������� �� ���� ����� ������ � ������� ���������
            for (int u = 0; u < vertices; ++u) {
                for (int v = u + 1; v < vertices; ++v) {
                    // ���� ����� ��������� u � v ���� �����
                    if (adjacencyMatrix[u][v]) {
                        // ��������� ����� � ������ �����
                        edges.push_back({ u, v, adjacencyMatrix[u][v] });
                    }
                }
            }

            // ��������� ������ ����� �� ����������� ����
            std::sort(edges.begin(), edges.end());

            // ������� ������ ��� �������� ��������� ������
            std::vector<int> parent(vertices);
            // ������� ������ ��� �������� ������ ������, �������������� ��� ����� ������
            std::vector<int> rank(vertices, 0);
            // �������������� ������ ������� ��� ���� ����������� ������������ �������
            for (int i = 0; i < vertices; ++i) {
                parent[i] = i;
            }

            // ���������� ��� �������� ���� ������������ ��������� ������
            T mstWeight = 0;
            // ������ ��� �������� ����� ������������ ��������� ������
            std::vector<std::pair<int, int>> mstEdges;
            // �������� �� ���� ������ � ��������������� ������� �����
            for (const auto& edge : edges) {
                // ������� �������� ������� ��� ������ �������� �����
                int rootX = find(parent, edge.src);
                int rootY = find(parent, edge.dest);

                // ���� �������� ������� �������� (����� �� �������� ����)
                if (rootX != rootY) {
                    // ��������� ��� �������� ����� � ������ ���� ������������ ��������� ������
                    mstWeight += edge.weight;
                    // ���������� ���������, ���������� ����� �������� �����
                    unionSets(parent, rank, rootX, rootY);
                    // ��������� ������� ����� � ������ ����� ������������ ��������� ������
                    mstEdges.push_back({ edge.src, edge.dest });
                }
            }

            // ���������� ����: ��� ������������ ��������� ������ � ������ ����� ������������ ��������� ������
            return { mstWeight, mstEdges };
        }

        template<typename T>
        std::vector<std::vector<T>> Graph<T>::toIncidenceMatrix() {
            // ������� ������� ������������� �������� vertices x edgeCount
            std::vector<std::vector<T>> incidenceMatrix(vertices);

            // ������������ ���������� ����� � �����
            int edgeCount = 0;
            for (int i = 0; i < vertices; ++i) {
                for (int j = i + 1; j < vertices; ++j) {
                    if (adjacencyMatrix[i][j] != 0) {
                        edgeCount++;
                    }
                }
            }

            // �������� ������ ������ ������ ������� ������������� � ������������ � ����������� �����
            for (int i = 0; i < vertices; ++i) {
                incidenceMatrix[i].resize(edgeCount, 0);
            }

            // ��������� ������� �������������
            int edgeIndex = 0;
            for (int i = 0; i < vertices; ++i) {
                for (int j = i + 1; j < vertices; ++j) {
                    if (adjacencyMatrix[i][j] != 0) {
                        // ���� ����� ��������� i � j ���� �����, ��������� ��������������� �������� ������� �������������
                        incidenceMatrix[i][edgeIndex] = adjacencyMatrix[i][j];
                        incidenceMatrix[j][edgeIndex] = adjacencyMatrix[i][j];
                        edgeIndex++;
                    }
                }
            }

            // ���������� ������� �������������
            return incidenceMatrix;
        }

        template<typename T>
        std::vector<std::vector<T>> Graph<T>::floydWarshall() {
            // ������� ������� ����������, �������������� �� ���������� �� ������� ���������
            std::vector<std::vector<T>> dist = adjacencyMatrix;

            // ��������� ������� ���������� �������������� ��� �������������� �����
            for (int i = 0; i < vertices; ++i) {
                for (int j = 0; j < vertices; ++j) {
                    if (dist[i][j] == 0 && i != j) {
                        dist[i][j] = std::numeric_limits<T>::max();
                    }
                }
            }

            // ��������� �������� ������-��������
            for (int k = 0; k < vertices; ++k) {
                for (int i = 0; i < vertices; ++i) {
                    for (int j = 0; j < vertices; ++j) {
                        // ���� ���������� ���� �� ������� i � ������� j ����� ������� k
                        // � ���� ���� ������ �������� ����������� ���� �� i � j
                        if (dist[i][k] != std::numeric_limits<T>::max() && dist[k][j] != std::numeric_limits<T>::max() &&
                            dist[i][k] + dist[k][j] < dist[i][j]) {
                            // ��������� �������� ����������� ���� �� i � j
                            dist[i][j] = dist[i][k] + dist[k][j];
                        }
                    }
                }
            }

            // ���������� ������� ���������� ���������� ����� ����� ������ ������
            return dist;
        }

        template<typename T>
        bool Graph<T>::isSafe(int v, std::vector<int>& path, int pos) {
            // ���������, ���� �� ����� ����� ������� �������� � ���������� �������� � ����
            if (adjacencyMatrix[path[pos - 1]][v] == 0) {
                return false;
            }

            // ���������, �� �������� �� �� ��� ��� ������� ����� � ����
            for (int i = 0; i < pos; ++i) {
                if (path[i] == v) {
                    return false;
                }
            }

            // ���� ������� ��������� ��� ���������� � ����, ���������� true
            return true;
        }

        template<typename T>
        bool Graph<T>::hamCycleUtil(std::vector<int>& path, int pos) {
            // ���� �� �������� ��� �������, ��������� ������� ����� ����� ��������� � ������ ���������
            if (pos == vertices) {
                if (adjacencyMatrix[path[pos - 1]][path[0]] != 0) {
                    return true;
                }
                else {
                    return false;
                }
            }

            // ���������� ��� �������, ����� ���������
            for (int v = 1; v < vertices; ++v) {
                // ���� ������� ��������� ��� ���������� � ����
                if (isSafe(v, path, pos)) {
                    // ��������� ������� � ����
                    path[pos] = v;
                    // ���������� �������� ������� ��� ��������� ������� � ����
                    if (hamCycleUtil(path, pos + 1) == true) {
                        return true;
                    }
                    // ���� �� ������� ����� ����������� ����, ������� ������� �� ����
                    path[pos] = -1;
                }
            }

            // ���� �� ������� ����� ����������� ����, ���������� false
            return false;
        }

        template<typename T>
        bool Graph<T>::findHamiltonianPath(std::vector<int>& path) {
            // �������� ������ ������� path � �������������� ��� ���������� -1
            path.resize(vertices, -1);
            // �������� ���� � ������� 0
            path[0] = 0;

            // �������� ��������������� ������� hamCycleUtil ��� ������ ������������ �����
            if (hamCycleUtil(path, 1) == false) {
                return false;
            }

            // ���� ����������� ���� ������, ���������� true
            return true;
        }

        template<typename T>
        bool Graph<T>::isEulerianCycle() {
            // ���������, �������� �� ���� �������
            if (!isConnected()) {
                return false;
            }

            // ������������ ���������� ������ � �������� ��������
            int oddDegreeCount = 0;
            for (int i = 0; i < vertices; ++i) {
                int degree = 0;
                for (int j = 0; j < vertices; ++j) {
                    if (adjacencyMatrix[i][j] != 0) {
                        degree++;
                    }
                }

                if (degree % 2 != 0) {
                    oddDegreeCount++;
                }
            }

            // ���� ���������� ������ � �������� �������� ����� 0, ���� �������� ���������
            return oddDegreeCount == 0;
        }

        template<typename T>
        bool Graph<T>::isConnected() {
            // ������� ������ ��� ������������ ���������� ������
            std::vector<bool> visited(vertices, false);
            // ������� ������� ��� ������ � ������
            std::queue<int> q;

            // ���� ������ ������� � ��������� ��������
            int i;
            for (i = 0; i < vertices; ++i) {
                int degree = 0;
                for (int j = 0; j < vertices; ++j) {
                    if (adjacencyMatrix[i][j] != 0) {
                        degree++;
                    }
                }
                if (degree != 0) {
                    break;
                }
            }

            // ���� ��� ������� ����� ������� �������, ���� ��������� �������
            if (i == vertices) {
                return true;
            }

            // �������� ����� � ������ � ��������� �������
            q.push(i);
            visited[i] = true;

            // ��������� ����� � ������
            while (!q.empty()) {
                int v = q.front();
                q.pop();

                // ������������ �������� �������
                for (int j = 0; j < vertices; ++j) {
                    if (adjacencyMatrix[v][j] && !visited[j]) {
                        visited[j] = true;
                        q.push(j);
                    }
                }
            }

            // ���������, ��� �� ������� � ��������� �������� ���� ��������
            for (int i = 0; i < vertices; ++i) {
                int degree = 0;
                for (int j = 0; j < vertices; ++j) {
                    if (adjacencyMatrix[i][j] != 0) {
                        degree++;
                    }
                }

                if (degree != 0 && !visited[i]) {
                    return false;
                }
            }

            // ���� ��� ������� � ��������� �������� ���� ��������, ���� �������
            return true;
        }

        template<typename T>
        bool Graph<T>::hasEulerianCycle() {
            // �������� ������� isEulerianCycle ��� �������� ������� �������� �����
            return isEulerianCycle();
        }

        template<typename T>
        void Graph<T>::eulerianUtil(int u, std::vector<std::vector<T>>& adj, std::vector<int>& path) {
            // ������������ ��� �������� �������
            for (int v = 0; v < vertices; ++v) {
                if (adj[u][v]) {
                    // ������� ����� ����� ��������� u � v
                    adj[u][v]--;
                    adj[v][u]--;
                    // ���������� �������� ������� ��� ��������� �������
                    eulerianUtil(v, adj, path);
                }
            }
            // ��������� ������� ������� � ����
            path.push_back(u);
        }

        template<typename T>
        void Graph<T>::printEulerianCycle() {
            // ��������� ������� �������� �����
            if (!isEulerianCycle()) {
                std::cout << "Graph doesn't contain Eulerian Cycle\n";
                return;
            }

            // ������� ������ ��� �������� �������� �����
            std::vector<int> path;
            // ������� ����� ������� ���������
            std::vector<std::vector<T>> adj = adjacencyMatrix;

            // �������� ��������������� ������� ��� ���������� �������� �����
            eulerianUtil(0, adj, path);

            // �������������� ����, ����� �������� ���������� ������� ������
            std::reverse(path.begin(), path.end());

            // ������� ������� ����
            for (int v : path) {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }

        template class Graph<int>;
        template class Graph<double>;
        template class Graph<float>;

    } // namespace Graph
} // namespace AlgorithmLibrary
