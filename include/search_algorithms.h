#pragma once
#ifndef SEARCH_ALGORITHMS_H
#define SEARCH_ALGORITHMS_H

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

#endif // SEARCH_ALGORITHMS_H
