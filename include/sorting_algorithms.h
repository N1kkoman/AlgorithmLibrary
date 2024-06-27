#pragma once

#include <vector>

namespace AlgorithmLibrary {
    namespace Sorting {

        template<typename T>
        class Sort {
        public:
            static void bubbleSort(std::vector<T>& arr);
            static void quickSort(std::vector<T>& arr, int left, int right);
            static void mergeSort(std::vector<T>& arr, int left, int right);
        private:
            static void merge(std::vector<T>& arr, int left, int mid, int right);
        };

    } // namespace Sorting
} // namespace AlgorithmLibrary

