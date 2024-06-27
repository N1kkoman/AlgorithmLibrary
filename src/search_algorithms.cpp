#include "../include/search_algorithms.h"
#include <cmath>
#include <string>

namespace AlgorithmLibrary {
    namespace Search {

        template<typename T>
        int Search<T>::linearSearch(const std::vector<T>& arr, T key) {
            for (size_t i = 0; i < arr.size(); i++) {
                if (arr[i] == key) { // Если текущий элемент равен искомому ключу
                    return i; // Возвращаем индекс найденного элемента
                }
            }
            return -1; // Если элемент не найден, возвращаем -1
        }

        template<typename T>
        int Search<T>::binarySearch(const std::vector<T>& arr, T key, int left, int right) {
            while (left <= right) { // Пока левая граница не превышает правую
                int mid = left + (right - left) / 2; // Вычисляем средний индекс
                if (arr[mid] == key) { // Если средний элемент равен искомому ключу
                    return mid; // Возвращаем индекс найденного элемента
                }
                if (arr[mid] < key) { // Если средний элемент меньше искомого ключа
                    left = mid + 1; // Сдвигаем левую границу вправо
                }
                else { // Если средний элемент больше искомого ключа
                    right = mid - 1; // Сдвигаем правую границу влево
                }
            }
            return -1; // Если элемент не найден, возвращаем -1
        }

        template<typename T>
        int Search<T>::jumpSearch(const std::vector<T>& arr, T key) {
            int n = arr.size(); // Получаем размер массива
            int step = std::sqrt(n); // Вычисляем размер шага для прыжков
            int prev = 0; // Инициализируем предыдущую позицию

            while (arr[std::min(step, n) - 1] < key) { // Пока элемент на текущем шаге меньше искомого ключа
                prev = step; // Сохраняем предыдущую позицию
                step += std::sqrt(n); // Увеличиваем шаг
                if (prev >= n) { // Если предыдущая позиция превышает размер массива
                    return -1; // Элемент не найден, возвращаем -1
                }
            }

            for (int i = prev; i < std::min(step, n); i++) { // Линейный поиск в диапазоне [prev, step)
                if (arr[i] == key) { // Если текущий элемент равен искомому ключу
                    return i; // Возвращаем индекс найденного элемента
                }
            }

            return -1; // Если элемент не найден, возвращаем -1
        }

        template class Search<int>; // Эксплицитная инстанциация шаблона
        template class Search<float>;
        template class Search<double>;
        template class Search<std::string>;

    } // namespace Search
} // namespace AlgorithmLibrary
