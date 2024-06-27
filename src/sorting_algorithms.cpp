#include "../include/sorting_algorithms.h"
#include <vector>

namespace AlgorithmLibrary {
    namespace Sorting {

        template<typename T>
        void Sort<T>::bubbleSort(std::vector<T>& arr) {
            bool swapped;
            for (size_t i = 0; i < arr.size() - 1; i++) {
                swapped = false;
                // Проход по элементам массива от начала до предпоследнего элемента
                for (size_t j = 0; j < arr.size() - i - 1; j++) {
                    // Сравнение соседних элементов и их обмен, если они расположены в неправильном порядке
                    if (arr[j] > arr[j + 1]) {
                        std::swap(arr[j], arr[j + 1]);
                        swapped = true;
                    }
                }
                // Если обменов не было, массив уже отсортирован, и можно завершить сортировку
                if (!swapped)
                    break;
            }
        }

        template<typename T>
        void Sort<T>::quickSort(std::vector<T>& arr, int left, int right) {
            if (left < right) {
                // Выбор опорного элемента (pivot) - последний элемент в текущем диапазоне
                int pivot = arr[right];
                int i = left - 1;

                // Разделение элементов на две части относительно опорного элемента
                for (int j = left; j < right; j++) {
                    if (arr[j] <= pivot) {
                        i++;
                        std::swap(arr[i], arr[j]);
                    }
                }
                std::swap(arr[i + 1], arr[right]);
                int pi = i + 1;

                // Рекурсивный вызов функции для левой и правой частей массива
                quickSort(arr, left, pi - 1);
                quickSort(arr, pi + 1, right);
            }
        }

        template<typename T>
        void Sort<T>::mergeSort(std::vector<T>& arr, int left, int right) {
            if (left < right) {
                // Нахождение среднего элемента для разделения массива на две части
                int mid = left + (right - left) / 2;
                // Рекурсивный вызов функции для левой и правой частей массива
                mergeSort(arr, left, mid);
                mergeSort(arr, mid + 1, right);
                // Слияние отсортированных частей массива
                merge(arr, left, mid, right);
            }
        }

        template<typename T>
        void Sort<T>::merge(std::vector<T>& arr, int left, int mid, int right) {
            int n1 = mid - left + 1;
            int n2 = right - mid;
            std::vector<T> L(n1);
            std::vector<T> R(n2);

            // Копирование элементов из исходного массива в временные массивы L и R
            for (int i = 0; i < n1; i++)
                L[i] = arr[left + i];
            for (int j = 0; j < n2; j++)
                R[j] = arr[mid + 1 + j];

            int i = 0, j = 0, k = left;
            // Слияние элементов из временных массивов L и R обратно в исходный массив
            while (i < n1 && j < n2) {
                if (L[i] <= R[j]) {
                    arr[k] = L[i];
                    i++;
                }
                else {
                    arr[k] = R[j];
                    j++;
                }
                k++;
            }

            // Копирование оставшихся элементов из массива L, если они есть
            while (i < n1) {
                arr[k] = L[i];
                i++;
                k++;
            }

            // Копирование оставшихся элементов из массива R, если они есть
            while (j < n2) {
                arr[k] = R[j];
                j++;
                k++;
            }
        }

        template class Sort<int>; // Эксплицитная инстанциация шаблона
        template class Sort<float>;
        template class Sort<double>;

    } // namespace Sorting
} // namespace AlgorithmLibrary
