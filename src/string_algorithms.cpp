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

            // ���������� ������� LPS (Longest Proper Prefix which is also Suffix)
            std::vector<int> lps(m);
            int len = 0; // ������������� ����� �������� ����������
            lps[0] = 0;
            int i = 1; // ������������� ������� �������� ������� � �������
            while (i < m) { // ���� ��� ���������� �������� LPS ��� ������� ������� �������
                if (pattern[i] == pattern[len]) { // ���� ������� ������ ������� ��������� � �������� ��������
                    len++; // ���������� ����� �������� ����������
                    lps[i] = len; // ���������� �������� LPS ��� �������� �������
                    i++; // ������� � ���������� ������� �������
                }
                else {
                    if (len != 0) { // ���� ����� �������� ���������� �� ����� 0
                        len = lps[len - 1]; // ���������� ����� �������� ���������� �� �������� LPS ����������� �������
                    }
                    else {
                        lps[i] = 0; // ��������� �������� LPS ��� �������� ������� � 0
                        i++; // ������� � ���������� ������� �������
                    }
                }
            }

            // ����� ���� ��������� ������� � ������
            i = 0; // ������������� ������� �������� ������� � ������
            int j = 0; // ������������� ������� �������� ������� � �������
            while (i < n) { // ���� ��� ��������� �������� ������ � �������
                if (pattern[j] == text[i]) {
                    j++;
                    i++;
                }
                if (j == m) { // ���� ���������� ������ ���������� �������
                    result.push_back(i - j); // ���������� ������� ������ ���������� � result
                    j = lps[j - 1]; // ����� ������� �������� ������� � ������� �� �������� LPS ����������� �������
                }
                else if (i < n && pattern[j] != text[i]) { // ���� ������� �� ���������
                    if (j != 0) { // ���� ������ �������� ������� � ������� �� ����� 0
                        j = lps[j - 1]; // ����� ������� �������� ������� � ������� �� �������� LPS ����������� �������
                    }
                    else {
                        i++; // ���������� ������� �������� ������� � ������
                    }
                }
            }

            return result;
        }

        std::vector<int> StringAlgorithms::rabinKarp(const std::string& text, const std::string& pattern, int prime) {
            std::vector<int> result;
            int m = pattern.size();
            int n = text.size();
            int i, j; // ���������� ���������� ��� ������
            int p = 0; // ������������� ���� ��� �������
            int t = 0; // ������������� ���� ��� �������� ���� ������
            int h = 1; // ������������� �������� ��� ���������� ����
            int d = 256; // ���������� �������� � ��������

            // ���������� h = pow(d, m-1) % prime
            for (i = 0; i < m - 1; i++)
                h = (h * d) % prime;

            // ���������� ��������� �����
            for (i = 0; i < m; i++) {
                p = (d * p + pattern[i]) % prime; // ���������� ���� ��� �������
                t = (d * t + text[i]) % prime; // ���������� ���� ��� �������� ���� ������
            }

            // ����� ���� ���������
            for (i = 0; i <= n - m; i++) {
                if (p == t) { // ���� ���� ���������
                    // �������� ����������� ��� ���������� �����
                    for (j = 0; j < m; j++) {
                        if (text[i + j] != pattern[j]) // ���� ������� �� ���������
                            break; // ����� �� �����
                    }
                    if (j == m) // ���� ��� ������� �������
                        result.push_back(i); // ���������� ������� ������ ���������� � result
                }

                // ���������� ���� ��� ���������� ����
                if (i < n - m) {
                    t = (d * (t - text[i] * h) + text[i + m]) % prime; // ���������� ���� ��� ���������� ���� ������
                    if (t < 0) // ���� ��� �������������
                        t = (t + prime); // ���������� �������� prime ��� ��������� �������������� ����
                }
            }

            return result;
        }

        struct TrieNode {
            std::unordered_map<char, TrieNode*> children;  // ���-������� ��� �������� �������� �����
            TrieNode* failureLink = nullptr;  // ������ �� ����, �� �������� ���������� ������� ��� ��������� ����������
            std::vector<std::string> output;  // ������ ��� �������� ���������, ��������������� � ������ ����
        };

        class AhoCorasick {
        public:
            AhoCorasick(const std::vector<std::string>& patterns) {
                root = new TrieNode();  // �������� ��������� ����
                buildTrie(patterns);  // ���������� ���� �� ������ ���������
                buildFailureLinks();  // ���������� ������ �� ���� ��� ��������� ����������
            }

            ~AhoCorasick() {
                // �������� ������
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
                std::vector<std::pair<int, std::string>> result;  // ������ ��� �������� ����������� ������
                TrieNode* currentNode = root;  // �������� ����� � ��������� ����
                for (int i = 0; i < text.size(); ++i) {
                    char c = text[i];
                    // ��������� �� ������� �� ���� ��� ��������� ����������, ���� �� ������ ���������� ��� �� ������ �� �����
                    while (currentNode != root && currentNode->children.find(c) == currentNode->children.end()) {
                        currentNode = currentNode->failureLink;
                    }
                    // ���� ������� ����������, ��������� � ��������������� �������� ����
                    if (currentNode->children.find(c) != currentNode->children.end()) {
                        currentNode = currentNode->children[c];
                    }
                    // ���� � ������� ���� ���� ��������������� ��������, ��������� �� � ���������� ������
                    if (!currentNode->output.empty()) {
                        for (const std::string& pattern : currentNode->output) {
                            result.emplace_back(i - pattern.size() + 1, pattern);
                        }
                    }
                }
                return result;
            }

        private:
            TrieNode* root;  // �������� ���� ����

            void buildTrie(const std::vector<std::string>& patterns) {
                for (const std::string& pattern : patterns) {
                    TrieNode* currentNode = root;
                    // ��������� ������� � ���
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
                // �������������� ������ �� ���� ��� ��������� ���������� ��� �������� ����� �����
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
                        // ���� ����, �� ������� ����� ��������� ������� ���� ��� ��������� ����������
                        while (failureNode != root && failureNode->children.find(c) == failureNode->children.end()) {
                            failureNode = failureNode->failureLink;
                        }
                        if (failureNode->children.find(c) != failureNode->children.end()) {
                            childNode->failureLink = failureNode->children[c];
                        }
                        else {
                            childNode->failureLink = root;
                        }
                        // ��������� �������� �� ����, �� ������� ��������� ������� ���� ��� ��������� ����������
                        childNode->output.insert(childNode->output.end(),
                            childNode->failureLink->output.begin(),
                            childNode->failureLink->output.end());
                        nodeQueue.push(childNode);
                    }
                }
            }
        };

        std::vector<std::pair<int, std::string>> StringAlgorithms::ahoCorasick(const std::string& text, const std::vector<std::string>& patterns) {
            AhoCorasick automaton(patterns);  // �������� �������� ���-������� �� ������ ���������
            return automaton.search(text);  // ����� ��������� � ������ � ������� ��������
        }

    } // namespace String
} // namespace AlgorithmLibrary
