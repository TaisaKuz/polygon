#include <iostream>
#include <vector>
#include <fstream>
#include <limits>

using namespace std;

// Структура для представления предмета с весом и стоимостью
struct Item {
    int weight;
    int value;
};

// Функция для чтения предметов из файла
vector<Item> readItemsFromFile(const string& filename, int& weight_limit) {
    vector<Item> items;
    ifstream infile(filename);

    if (!infile) {
        cerr << "Не удалось открыть файл: " << filename << endl;
        return items;
    }

    int w, v;
    infile >> weight_limit;  // Читаем ограничение по весу
    while (infile >> w >> v) {
        items.push_back({ w, v });
    }

    return items;
}

// Рекурсивная функция для поиска комбинаций с максимальной стоимостью
void findMaxValueCombination(const vector<Item>& items, int index, int current_weight,
    int current_value, int weight_limit, vector<Item>& current_combination,
    vector<Item>& best_combination, int& max_value) {
    // Если текущий вес превышает ограничение, вернуться
    if (current_weight >= weight_limit) {
        return;
    }

    // Если текущая стоимость больше максимальной, обновляем результат
    if (current_value > max_value) {
        max_value = current_value;
        best_combination = current_combination;
    }

    // Если все предметы рассмотрены, завершить рекурсию
    if (index == items.size()) {
        return;
    }

    // Вариант 1: не включаем текущий предмет
    findMaxValueCombination(items, index + 1, current_weight, current_value, weight_limit,
        current_combination, best_combination, max_value);

    // Вариант 2: включаем текущий предмет
    current_combination.push_back(items[index]);
    findMaxValueCombination(items, index + 1, current_weight + items[index].weight,
        current_value + items[index].value, weight_limit,
        current_combination, best_combination, max_value);
    current_combination.pop_back(); // Убираем текущий предмет для следующей комбинации
}

int main() {
    string filename = "";
    int weight_limit;
    vector<Item> items = readItemsFromFile(filename, weight_limit);

    if (items.empty()) {
        cout << "Файл пуст или не содержит данных." << endl;
        return 1;
    }

    vector<Item> best_combination;
    vector<Item> current_combination;
    int max_value = 0;

    findMaxValueCombination(items, 0, 0, 0, weight_limit, current_combination, best_combination, max_value);

    // Выводим результат
    cout << "Оптимальный набор предметов: ";
    for (const auto& item : best_combination) {
        cout << item.weight << " ";
    }
    cout << endl;
    cout << "Максимальная стоимость: " << max_value << endl;

    return 0;
}