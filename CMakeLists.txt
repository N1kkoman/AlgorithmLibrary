cmake_minimum_required(VERSION 3.10)

project(AlgorithmLibrary)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Добавляем директорию include к списку путей поиска заголовочных файлов
include_directories(include)

set(SOURCE_FILES
    src/sorting_algorithms.cpp
    src/search_algorithms.cpp
    src/graph_algorithms.cpp
    src/string_algorithms.cpp
)

add_library(AlgorithmLibrary STATIC ${SOURCE_FILES})

add_executable(main main.cpp)

# Линкуем статическую библиотеку с исполняемым файлом
target_link_libraries(main AlgorithmLibrary)
