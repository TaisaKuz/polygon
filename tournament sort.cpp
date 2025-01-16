#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <fstream> 

const int MAX_SIZE = 16; // Максимальный размер массива 

std::vector<int> tournament_sort(std::vector<int>& array); // Функция для сортировки с использованием турнира

std::vector<int> bracketize(std::vector<int>& array); 

int main() { 
    std::ifstream input_file("dataset.txt");
    if (!input_file) { 
        std::cerr << "EROR reading fail" << std::endl;
        return 1;
    }

    std::vector<int> array;
    int num;

    // Считываем числа из файла и добавляем их в вектор (O(n), где n — количество чисел в файле)
    while (input_file >> num) {
        array.push_back(num); // Добавление в вектор амортизировано O(1)
    }
    input_file.close(); 

    std::vector<int> sorted_array = tournament_sort(array); // O(k log k) для небольших массивов или O(n ^ 2) для больших

    // Вывод отсортированного массива (O(n), где n — размер массива)
    std::cout << "Отсортированный массив: ";
    for (int num : sorted_array) { // Проход по всем элементам массива
        std::cout << num << " ";
    }
    std::cout << std::endl;

    return 0; 
}

std::vector<int> tournament_sort(std::vector<int>& array) {
    // Если размер массива <= MAX_SIZE, используется std::sort (O(k log k), где k — размер массива)
    if (array.size() <= MAX_SIZE) {
        std::sort(array.begin(), array.end()); // сортировка
        return array; // Возвращаем отсортированный массив
    }

    return bracketize(array);  // Если размер массива больше, вызывается функция bracketize Сложность bracketize: O(n^2)
}

std::vector<int> bracketize(std::vector<int>& array) {
    std::priority_queue<int, std::vector<int>, std::greater<int>> pq; // приоритетная очередь (минимальная куча) (O(1))

    std::vector<int> winners; // массив для хранения "победителей" (O(1))

    for (int i = 0; i < std::min(MAX_SIZE, (int)array.size()); ++i) {   // Заполняем приоритетную очередь  O(log k) для каждого элемента, итого O(k log k)
        pq.push(array[i]); 
    }

    array.erase(array.begin(), array.begin() + std::min(MAX_SIZE, (int)array.size())); // Удаление первых k элементов: O(n) из-за смещения оставшихся элементов


    while (!array.empty() || !pq.empty()) { // Цикл выполняется n раз, где n — количество элементов
        // Если очередь не пуста, добавляем минимальный элемент в массив "победителей"
        if (!pq.empty()) {
            winners.push_back(pq.top()); // Добавление в вектор O(1) амортизировано
            pq.pop(); // Удаление из кучи O(log k)
        }

        // Если в массиве еще есть элементы, добавляем их в очередь
        if (!array.empty()) {
            pq.push(array[0]); // Добавление в кучу O(log k)
            array.erase(array.begin()); // Удаление первого элемента массива O(n)
        }
    }

    return winners; // O(1)
}