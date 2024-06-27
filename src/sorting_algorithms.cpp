#include "../include/sorting_algorithms.h"
#include <vector>

namespace AlgorithmLibrary {
    namespace Sorting {

        template<typename T>
        void Sort<T>::bubbleSort(std::vector<T>& arr) {
            bool swapped;
            for (size_t i = 0; i < arr.size() - 1; i++) {
                swapped = false;
                // ������ �� ��������� ������� �� ������ �� �������������� ��������
                for (size_t j = 0; j < arr.size() - i - 1; j++) {
                    // ��������� �������� ��������� � �� �����, ���� ��� ����������� � ������������ �������
                    if (arr[j] > arr[j + 1]) {
                        std::swap(arr[j], arr[j + 1]);
                        swapped = true;
                    }
                }
                // ���� ������� �� ����, ������ ��� ������������, � ����� ��������� ����������
                if (!swapped)
                    break;
            }
        }

        template<typename T>
        void Sort<T>::quickSort(std::vector<T>& arr, int left, int right) {
            if (left < right) {
                // ����� �������� �������� (pivot) - ��������� ������� � ������� ���������
                int pivot = arr[right];
                int i = left - 1;

                // ���������� ��������� �� ��� ����� ������������ �������� ��������
                for (int j = left; j < right; j++) {
                    if (arr[j] <= pivot) {
                        i++;
                        std::swap(arr[i], arr[j]);
                    }
                }
                std::swap(arr[i + 1], arr[right]);
                int pi = i + 1;

                // ����������� ����� ������� ��� ����� � ������ ������ �������
                quickSort(arr, left, pi - 1);
                quickSort(arr, pi + 1, right);
            }
        }

        template<typename T>
        void Sort<T>::mergeSort(std::vector<T>& arr, int left, int right) {
            if (left < right) {
                // ���������� �������� �������� ��� ���������� ������� �� ��� �����
                int mid = left + (right - left) / 2;
                // ����������� ����� ������� ��� ����� � ������ ������ �������
                mergeSort(arr, left, mid);
                mergeSort(arr, mid + 1, right);
                // ������� ��������������� ������ �������
                merge(arr, left, mid, right);
            }
        }

        template<typename T>
        void Sort<T>::merge(std::vector<T>& arr, int left, int mid, int right) {
            int n1 = mid - left + 1;
            int n2 = right - mid;
            std::vector<T> L(n1);
            std::vector<T> R(n2);

            // ����������� ��������� �� ��������� ������� � ��������� ������� L � R
            for (int i = 0; i < n1; i++)
                L[i] = arr[left + i];
            for (int j = 0; j < n2; j++)
                R[j] = arr[mid + 1 + j];

            int i = 0, j = 0, k = left;
            // ������� ��������� �� ��������� �������� L � R ������� � �������� ������
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

            // ����������� ���������� ��������� �� ������� L, ���� ��� ����
            while (i < n1) {
                arr[k] = L[i];
                i++;
                k++;
            }

            // ����������� ���������� ��������� �� ������� R, ���� ��� ����
            while (j < n2) {
                arr[k] = R[j];
                j++;
                k++;
            }
        }

        template class Sort<int>; // ������������ ������������ �������
        template class Sort<float>;
        template class Sort<double>;

    } // namespace Sorting
} // namespace AlgorithmLibrary
