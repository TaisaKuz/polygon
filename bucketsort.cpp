#include <iostream>  
#include <vector>    
#include <fstream>   
using namespace std;

void insertionSort(vector<float>& bucket) { // Функция для сортировки отдельных "карманов" 
    for (int i = 1; i < bucket.size(); ++i) {
        float key = bucket[i];    
        int j = i - 1;            

        while (j >= 0 && bucket[j] > key) {  // Перемещение элементов, которые больше key
            bucket[j + 1] = bucket[j];
            j--;
        }
        bucket[j + 1] = key;  
    }
    // Сложность: O(k^2), где k — количество элементов в "кармане"
}

// Функция для сортировки массива arr[] размером n методом bucket sort
void bucketSort(float arr[], int n) {
    vector<float> b[n]; // Сложность: O(n) на инициализацию вектора "карманов"

  
    for (int i = 0; i < n; i++) {
        int bi = n * arr[i];    
        b[bi].push_back(arr[i]); // Добавляем элемент в соответствующий "карман"
    }
    // Сложность: O(n), так как каждый элемент добавляется в "карман" за O(1)
    for (int i = 0; i < n; i++) {
        insertionSort(b[i]);     // Сортировка каждого "кармана"
    }
    // Сложность: O(k^2 * n), где k — среднее количество элементов в "кармане"
    // Если элементы равномерно распределены: O(n)

    int index = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < b[i].size(); j++) {
            arr[index++] = b[i][j]; // Перенос элементов из "карманов" в массив
        }
    }
    // Сложность: O(n), так как каждый элемент обрабатывается один раз
}


int main() {
    ifstream input_file("dataset.txt");
    if (!input_file) { 
        cerr << " Eror readin fail" << endl;
        return 1;
    }

    vector<float> data; // Вектор для хранения данны
    float num;
    while (input_file >> num) {  
        data.push_back(num);  
    }
    input_file.close();         
    // Сложность: O(n), где n — количество чисел в файле

    int n = data.size();
    float* arr = new float[n];  // Динамический массив для сортировки
    for (int i = 0; i < n; i++) {
        arr[i] = data[i];
    }
    // Сложность: O(n)

    // Вызываем функцию bucketSort для сортировки
    bucketSort(arr, n);  // Сложность зависит от распределения: O(n) в лучшем случае, O(n^2) в худшем

    cout << "Отсортированный массив:\n";
    for (int i = 0; i < n; i++) {
        cout << arr[i] << " ";
    }
    cout << endl;

    // Освобождаем память
    delete[] arr; // O(1)
    return 0;
}