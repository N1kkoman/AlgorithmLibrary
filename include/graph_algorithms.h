#pragma once

#include <vector>
#include <utility> // for std::pair
#include <iostream>

namespace AlgorithmLibrary {
    namespace Graph {

        template<typename T>
        class Graph {
        public:
            Graph(int vertices);
            Graph(const std::vector<std::vector<T>>& adjacencyMatrix);
            void addEdge(int v, int w, T weight);
            std::vector<T> bfs(int start);
            std::vector<T> dfs(int start);
            std::vector<T> dijkstra(int src);
            std::vector<T> bellmanFord(int src);
            std::pair<T, std::vector<std::pair<int, int>>> kruskal();
            std::vector<std::vector<T>> toIncidenceMatrix();
            std::vector<std::vector<T>> floydWarshall();
            bool hasEulerianCycle();
            bool findHamiltonianPath(std::vector<int>& path);
            void printEulerianCycle();

        private:
            int vertices;
            std::vector<std::vector<T>> adjacencyMatrix;
            int find(std::vector<int>& parent, int i);
            void unionSets(std::vector<int>& parent, std::vector<int>& rank, int x, int y);
            bool isSafe(int v, std::vector<int>& path, int pos);
            bool hamCycleUtil(std::vector<int>& path, int pos);
            bool isEulerianCycle();
            bool isConnected();
            void eulerianUtil(int u, std::vector<std::vector<T>>& adj, std::vector<int>& path);
        };

    } // namespace Graph
} // namespace AlgorithmLibrary