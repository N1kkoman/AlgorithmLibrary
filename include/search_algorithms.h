#pragma once
#include <vector>

namespace AlgorithmLibrary {
    namespace Search {

        template<typename T>
        class Search {
        public:
            static int linearSearch(const std::vector<T>& arr, T key);
            static int binarySearch(const std::vector<T>& arr, T key, int left, int right);
            static int jumpSearch(const std::vector<T>& arr, T key);
        };

    } // namespace Search
} // namespace AlgorithmLibrary

