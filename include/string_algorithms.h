#pragma once
#ifndef STRING_ALGORITHMS_H
#define STRING_ALGORITHMS_H

#include <string>
#include <vector>
#include <unordered_map>

namespace AlgorithmLibrary {
    namespace String {

        class StringAlgorithms {
        public:
            static std::vector<int> KMP(const std::string& text, const std::string& pattern);
            static std::vector<int> rabinKarp(const std::string& text, const std::string& pattern, int prime = 101);
            static std::vector<std::pair<int, std::string>> ahoCorasick(const std::string& text, const std::vector<std::string>& patterns);
        };

    } // namespace String
} // namespace AlgorithmLibrary

#endif // STRING_ALGORITHMS_H
