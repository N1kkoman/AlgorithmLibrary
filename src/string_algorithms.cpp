#include "../include/string_algorithms.h"
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>

namespace AlgorithmLibrary {
    namespace String {
        std::vector<int> StringAlgorithms::KMP(const std::string& text, const std::string& pattern) {
            std::vector<int> result;
            int m = pattern.size();
            int n = text.size();

            // Построение таблицы LPS (Longest Proper Prefix which is also Suffix)
            std::vector<int> lps(m);
            int len = 0; // Инициализация длины текущего совпадения
            lps[0] = 0;
            int i = 1; // Инициализация индекса текущего символа в шаблоне
            while (i < m) { // Цикл для вычисления значений LPS для каждого символа шаблона
                if (pattern[i] == pattern[len]) { // Если текущий символ шаблона совпадает с символом префикса
                    len++; // Увеличение длины текущего совпадения
                    lps[i] = len; // Сохранение значения LPS для текущего символа
                    i++; // Переход к следующему символу шаблона
                }
                else {
                    if (len != 0) { // Если длина текущего совпадения не равна 0
                        len = lps[len - 1]; // Уменьшение длины текущего совпадения до значения LPS предыдущего символа
                    }
                    else {
                        lps[i] = 0; // Установка значения LPS для текущего символа в 0
                        i++; // Переход к следующему символу шаблона
                    }
                }
            }

            // Поиск всех вхождений шаблона в тексте
            i = 0; // Инициализация индекса текущего символа в тексте
            int j = 0; // Инициализация индекса текущего символа в шаблоне
            while (i < n) { // Цикл для сравнения символов текста и шаблона
                if (pattern[j] == text[i]) {
                    j++;
                    i++;
                }
                if (j == m) { // Если достигнуто полное совпадение шаблона
                    result.push_back(i - j); // Сохранение позиции начала совпадения в result
                    j = lps[j - 1]; // Сдвиг индекса текущего символа в шаблоне на значение LPS предыдущего символа
                }
                else if (i < n && pattern[j] != text[i]) { // Если символы не совпадают
                    if (j != 0) { // Если индекс текущего символа в шаблоне не равен 0
                        j = lps[j - 1]; // Сдвиг индекса текущего символа в шаблоне на значение LPS предыдущего символа
                    }
                    else {
                        i++; // Увеличение индекса текущего символа в тексте
                    }
                }
            }

            return result;
        }

        std::vector<int> StringAlgorithms::rabinKarp(const std::string& text, const std::string& pattern, int prime) {
            std::vector<int> result;
            int m = pattern.size();
            int n = text.size();
            int i, j; // Объявление переменных для циклов
            int p = 0; // Инициализация хеша для шаблона
            int t = 0; // Инициализация хеша для текущего окна текста
            int h = 1; // Инициализация значения для вычисления хеша
            int d = 256; // Количество символов в алфавите

            // Вычисление h = pow(d, m-1) % prime
            for (i = 0; i < m - 1; i++)
                h = (h * d) % prime;

            // Вычисление начальных хешей
            for (i = 0; i < m; i++) {
                p = (d * p + pattern[i]) % prime; // Вычисление хеша для шаблона
                t = (d * t + text[i]) % prime; // Вычисление хеша для текущего окна текста
            }

            // Поиск всех вхождений
            for (i = 0; i <= n - m; i++) {
                if (p == t) { // Если хеши совпадают
                    // Проверка посимвольно при совпадении хешей
                    for (j = 0; j < m; j++) {
                        if (text[i + j] != pattern[j]) // Если символы не совпадают
                            break; // Выход из цикла
                    }
                    if (j == m) // Если все символы совпали
                        result.push_back(i); // Сохранение позиции начала совпадения в result
                }

                // Вычисление хеша для следующего окна
                if (i < n - m) {
                    t = (d * (t - text[i] * h) + text[i + m]) % prime; // Вычисление хеша для следующего окна текста
                    if (t < 0) // Если хеш отрицательный
                        t = (t + prime); // Добавление значения prime для получения положительного хеша
                }
            }

            return result;
        }

        struct TrieNode {
            std::unordered_map<char, TrieNode*> children;  // Хэш-таблица для хранения дочерних узлов
            TrieNode* failureLink = nullptr;  // Ссылка на узел, по которому происходит переход при неудачном совпадении
            std::vector<std::string> output;  // Вектор для хранения паттернов, заканчивающихся в данном узле
        };

        class AhoCorasick {
        public:
            AhoCorasick(const std::vector<std::string>& patterns) {
                root = new TrieNode();  // Создание корневого узла
                buildTrie(patterns);  // Построение бора на основе паттернов
                buildFailureLinks();  // Построение ссылок на узлы при неудачном совпадении
            }

            ~AhoCorasick() {
                // Удаление дерева
                std::queue<TrieNode*> nodeQueue;
                nodeQueue.push(root);
                while (!nodeQueue.empty()) {
                    TrieNode* current = nodeQueue.front();
                    nodeQueue.pop();
                    for (auto& pair : current->children) {
                        nodeQueue.push(pair.second);
                    }
                    delete current;
                }
            }

            std::vector<std::pair<int, std::string>> search(const std::string& text) {
                std::vector<std::pair<int, std::string>> result;  // Вектор для хранения результатов поиска
                TrieNode* currentNode = root;  // Начинаем поиск с корневого узла
                for (int i = 0; i < text.size(); ++i) {
                    char c = text[i];
                    // Переходим по ссылкам на узлы при неудачном совпадении, пока не найдем совпадение или не дойдем до корня
                    while (currentNode != root && currentNode->children.find(c) == currentNode->children.end()) {
                        currentNode = currentNode->failureLink;
                    }
                    // Если найдено совпадение, переходим в соответствующий дочерний узел
                    if (currentNode->children.find(c) != currentNode->children.end()) {
                        currentNode = currentNode->children[c];
                    }
                    // Если в текущем узле есть заканчивающиеся паттерны, добавляем их в результаты поиска
                    if (!currentNode->output.empty()) {
                        for (const std::string& pattern : currentNode->output) {
                            result.emplace_back(i - pattern.size() + 1, pattern);
                        }
                    }
                }
                return result;
            }

        private:
            TrieNode* root;  // Корневой узел бора

            void buildTrie(const std::vector<std::string>& patterns) {
                for (const std::string& pattern : patterns) {
                    TrieNode* currentNode = root;
                    // Добавляем паттерн в бор
                    for (char c : pattern) {
                        if (currentNode->children.find(c) == currentNode->children.end()) {
                            currentNode->children[c] = new TrieNode();
                        }
                        currentNode = currentNode->children[c];
                    }
                    currentNode->output.push_back(pattern);
                }
            }

            void buildFailureLinks() {
                std::queue<TrieNode*> nodeQueue;
                root->failureLink = root;
                // Инициализируем ссылки на узлы при неудачном совпадении для дочерних узлов корня
                for (auto& pair : root->children) {
                    pair.second->failureLink = root;
                    nodeQueue.push(pair.second);
                }
                while (!nodeQueue.empty()) {
                    TrieNode* currentNode = nodeQueue.front();
                    nodeQueue.pop();
                    for (auto& pair : currentNode->children) {
                        char c = pair.first;
                        TrieNode* childNode = pair.second;
                        TrieNode* failureNode = currentNode->failureLink;
                        // Ищем узел, на который будет ссылаться текущий узел при неудачном совпадении
                        while (failureNode != root && failureNode->children.find(c) == failureNode->children.end()) {
                            failureNode = failureNode->failureLink;
                        }
                        if (failureNode->children.find(c) != failureNode->children.end()) {
                            childNode->failureLink = failureNode->children[c];
                        }
                        else {
                            childNode->failureLink = root;
                        }
                        // Добавляем паттерны из узла, на который ссылается текущий узел при неудачном совпадении
                        childNode->output.insert(childNode->output.end(),
                            childNode->failureLink->output.begin(),
                            childNode->failureLink->output.end());
                        nodeQueue.push(childNode);
                    }
                }
            }
        };

        std::vector<std::pair<int, std::string>> StringAlgorithms::ahoCorasick(const std::string& text, const std::vector<std::string>& patterns) {
            AhoCorasick automaton(patterns);  // Создание автомата Ахо-Корасик на основе паттернов
            return automaton.search(text);  // Поиск паттернов в тексте с помощью автомата
        }

    } // namespace String
} // namespace AlgorithmLibrary
