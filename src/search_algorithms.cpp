#include "../include/search_algorithms.h"
#include <cmath>
#include <string>

namespace AlgorithmLibrary {
    namespace Search {

        template<typename T>
        int Search<T>::linearSearch(const std::vector<T>& arr, T key) {
            for (size_t i = 0; i < arr.size(); i++) {
                if (arr[i] == key) { // ���� ������� ������� ����� �������� �����
                    return i; // ���������� ������ ���������� ��������
                }
            }
            return -1; // ���� ������� �� ������, ���������� -1
        }

        template<typename T>
        int Search<T>::binarySearch(const std::vector<T>& arr, T key, int left, int right) {
            while (left <= right) { // ���� ����� ������� �� ��������� ������
                int mid = left + (right - left) / 2; // ��������� ������� ������
                if (arr[mid] == key) { // ���� ������� ������� ����� �������� �����
                    return mid; // ���������� ������ ���������� ��������
                }
                if (arr[mid] < key) { // ���� ������� ������� ������ �������� �����
                    left = mid + 1; // �������� ����� ������� ������
                }
                else { // ���� ������� ������� ������ �������� �����
                    right = mid - 1; // �������� ������ ������� �����
                }
            }
            return -1; // ���� ������� �� ������, ���������� -1
        }

        template<typename T>
        int Search<T>::jumpSearch(const std::vector<T>& arr, T key) {
            int n = arr.size(); // �������� ������ �������
            int step = std::sqrt(n); // ��������� ������ ���� ��� �������
            int prev = 0; // �������������� ���������� �������

            while (arr[std::min(step, n) - 1] < key) { // ���� ������� �� ������� ���� ������ �������� �����
                prev = step; // ��������� ���������� �������
                step += std::sqrt(n); // ����������� ���
                if (prev >= n) { // ���� ���������� ������� ��������� ������ �������
                    return -1; // ������� �� ������, ���������� -1
                }
            }

            for (int i = prev; i < std::min(step, n); i++) { // �������� ����� � ��������� [prev, step)
                if (arr[i] == key) { // ���� ������� ������� ����� �������� �����
                    return i; // ���������� ������ ���������� ��������
                }
            }

            return -1; // ���� ������� �� ������, ���������� -1
        }

        template class Search<int>; // ������������ ������������ �������
        template class Search<float>;
        template class Search<double>;
        template class Search<std::string>;

    } // namespace Search
} // namespace AlgorithmLibrary
