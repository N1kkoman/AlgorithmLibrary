#include <iostream>
#include <vector>
#include <string>
#include "include/sorting_algorithms.h"
#include "include/string_algorithms.h"
#include "include/search_algorithms.h"
#include "include/graph_algorithms.h"

using namespace AlgorithmLibrary;

void testSortingAlgorithms() {
    std::vector<int> arr = { 64, 34, 25, 12, 22, 11, 90, 0, -34, -23424234 };
    Sorting::Sort<int>::bubbleSort(arr);
    std::cout << "Bubble Sorted array: ";
    for (const auto& elem : arr) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::vector<int> arr2 = { 64, 34, 25, 12, 22, 11, 90, 0, -34, -23424234 };
    Sorting::Sort<int>::quickSort(arr2, 0, arr2.size() - 1);
    std::cout << "Quick Sorted array: ";
    for (const auto& elem : arr2) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::vector<int> arr3 = { 64, 34, 25, 12, 22, 11, 90, 0, -34, -23424234 };
    Sorting::Sort<int>::mergeSort(arr3, 0, arr3.size() - 1);
    std::cout << "Merge Sorted array: ";
    for (const auto& elem : arr3) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;
}

void testSearchAlgorithms() {
    std::vector<int> arr = { 11, 22, 25, 34, 64, 90 };
    int index = Search::Search<int>::linearSearch(arr, 22);
    std::cout << "Linear Search: Element 22 is at index: " << index << std::endl;

    index = Search::Search<int>::binarySearch(arr, 22, 0, arr.size() - 1);
    std::cout << "Binary Search: Element 22 is at index: " << index << std::endl;

    index = Search::Search<int>::jumpSearch(arr, 22);
    std::cout << "Jump Search: Element 22 is at index: " << index << std::endl;

    std::vector<std::string> strArr = { "apple", "banana", "cherry", "date", "fig", "grape" };
    std::cout << "Linear Search: Element 'cherry' is at index: " << Search::Search<std::string>::linearSearch(strArr, "cherry") << std::endl;
}

void testStringAlgorithms() {
    std::string text = "pabcabyabxabcabcaby";
    std::string pattern = "abcaby";
    
    std::vector<int> kmpResults = String::StringAlgorithms::KMP(text, pattern);
    std::cout << "KMP: Pattern found at indices: ";
    for (int pos : kmpResults) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    std::vector<int> rkResults = String::StringAlgorithms::rabinKarp(text, pattern);
    std::cout << "Rabin-Karp: Pattern found at indices: ";
    for (int pos : rkResults) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    std::string ahoText = "abracadabra";
    std::vector<std::string> patterns = { "abra", "cad" };
    std::vector<std::pair<int, std::string>> ahoResults = String::StringAlgorithms::ahoCorasick(ahoText, patterns);
    std::cout << "Aho-Corasick results: " << std::endl;
    for (const auto& result : ahoResults) {
        std::cout << "Pattern \"" << result.second << "\" found at index " << result.first << std::endl;
    }
}
//{0, 1, 1, 0, 0},
//{ 1, 0, 0, 0, 1 },
//{ 1, 0, 0, 1, 0 },
//{ 0, 0, 1, 0, 1 },
//{ 0, 1, 0, 1, 0 }

void testGraphAlgorithms() {
    std::vector<std::vector<int>> adjacencyMatrix = {
        { 0, 10, 0, 11, 0, 0 },
        { 10, 0, 11, 2, 6, 0 },
        { 0, 11, 0, 0, 23, 4 },
        { 11, 2, 0, 0, 8, 0 },
        { 0, 6, 23, 8, 0, 3 },
        { 0, 0, 4, 0, 3, 0 }
    };
    Graph::Graph<int> graph(adjacencyMatrix);

    std::vector<int> bfsResult = graph.bfs(0);
    std::cout << "BFS from vertex 0: ";
    for (const auto& elem : bfsResult) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::vector<int> dfsResult = graph.dfs(0);
    std::cout << "DFS from vertex 0: ";
    for (const auto& elem : dfsResult) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::vector<int> dijkstraResult = graph.dijkstra(0);
    std::cout << "Dijkstra's algorithm from vertex 0: ";
    for (const auto& elem : dijkstraResult) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    std::vector<int> bellmanFordResult = graph.bellmanFord(0);
    std::cout << "Bellman-Ford algorithm from vertex 0: ";
    for (const auto& elem : bellmanFordResult) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    auto kruskalResult = graph.kruskal();
    int mstWeight = kruskalResult.first;
    std::vector<std::pair<int, int>> mstEdges = kruskalResult.second;
    std::cout << "Kruskal's algorithm MST weight: " << mstWeight << std::endl;
    std::cout << "Edges in the Minimum Spanning Tree:" << std::endl;
    for (const auto& edge : mstEdges) {
        std::cout << edge.first << " - " << edge.second << std::endl;
    }

    std::vector<std::vector<int>> incidenceMatrix = graph.toIncidenceMatrix();
    std::cout << "Incidence Matrix:" << std::endl;
    for (const auto& row : incidenceMatrix) {
        for (const auto& elem : row) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
    }

    std::vector<std::vector<int>> fwResult = graph.floydWarshall();
    std::cout << "Floyd-Warshall distance matrix:" << std::endl;
    for (const auto& row : fwResult) {
        for (const auto& elem : row) {
            if (elem == std::numeric_limits<int>::max()) {
                std::cout << "INF ";
            }
            else {
                std::cout << elem << " ";
            }
        }
        std::cout << std::endl;
    }

    bool hasEulerian = graph.hasEulerianCycle();
    std::cout << "Has Eulerian Cycle: " << (hasEulerian ? "Yes" : "No") << std::endl;
    if (hasEulerian) {
        std::cout << "Eulerian Cycle: ";
        graph.printEulerianCycle();
    }

    std::vector<int> hamiltonianPath;
    bool hasHamiltonian = graph.findHamiltonianPath(hamiltonianPath);
    std::cout << "Hamiltonian Path: " << (hasHamiltonian ? "Yes" : "No") << std::endl;
    if (hasHamiltonian) {
        for (const auto& vertex : hamiltonianPath) {
            std::cout << vertex << " ";
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "Testing Sorting Algorithms:" << std::endl;
    testSortingAlgorithms();
    std::cout << std::endl;

    std::cout << "Testing Search Algorithms:" << std::endl;
    testSearchAlgorithms();
    std::cout << std::endl;

    std::cout << "Testing String Algorithms:" << std::endl;
    testStringAlgorithms();
    std::cout << std::endl;

    std::cout << "Testing Graph Algorithms:" << std::endl;
    testGraphAlgorithms();
    std::cout << std::endl;

    return 0;
}
