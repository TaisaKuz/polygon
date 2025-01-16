#include <iostream>
#include <vector>
#include <fstream>
#include <queue>
#include <algorithm>
#include <chrono> // Для измерения времени
#include <random> // Для генерации тестовых данных
using namespace std;

// --- Tournament Sort ---
vector<int> tournament_sort(vector<int>& array, int max_size) {
	if (array.size() <= max_size) {
		sort(array.begin(), array.end());
		return array;
	}

	priority_queue<int, vector<int>, greater<int>> pq;
	for (int i = 0; i < min(max_size, (int)array.size()); ++i) {
		pq.push(array[i]);
	}
	array.erase(array.begin(), array.begin() + min(max_size, (int)array.size()));

	vector<int> sorted;
	while (!pq.empty() || !array.empty()) {
		if (!pq.empty()) {
			sorted.push_back(pq.top());
			pq.pop();
		}
		if (!array.empty()) {
			pq.push(array.front());
			array.erase(array.begin());
		}
	}
	return sorted;
}

// --- Bucket Sort ---
void insertionSort(vector<float>& bucket) {
	for (int i = 1; i < bucket.size(); ++i) {
		float key = bucket[i];
		int j = i - 1;
		while (j >= 0 && bucket[j] > key) {
			bucket[j + 1] = bucket[j];
			j--;
		}
		bucket[j + 1] = key;
	}
}

void bucketSort(vector<float>& arr) {
	int n = arr.size();
	vector<float> b[n];
	for (int i = 0; i < n; i++) {
		int bi = n * arr[i];
		b[bi].push_back(arr[i]);
	}
	for (int i = 0; i < n; i++) {
		insertionSort(b[i]);
	}
	arr.clear();
	for (int i = 0; i < n; i++) {
		for (float x : b[i]) {
			arr.push_back(x);
		}
	}
}

// --- Cycle Sort ---
int cycleSort(vector<int>& arr) {
	int writes = 0;
	for (int cycle_start = 0; cycle_start < arr.size() - 1; cycle_start++) {
		int item = arr[cycle_start];
		int pos = cycle_start;
		for (int i = cycle_start + 1; i < arr.size(); i++) {
			if (arr[i] < item) {
				pos++;
			}
		}
		if (pos == cycle_start) continue;
		while (item == arr[pos]) pos++;
		if (pos != cycle_start) {
			swap(item, arr[pos]);
			writes++;
		}
		while (pos != cycle_start) {
			pos = cycle_start;
			for (int i = cycle_start + 1; i < arr.size(); i++) {
				if (arr[i] < item) {
					pos++;
				}
			}
			while (item == arr[pos]) pos++;
			if (item != arr[pos]) {
				swap(item, arr[pos]);
				writes++;
			}
		}
	}
	return writes;
}

// --- Вспомогательные функции ---
vector<int> generateRandomArray(int size) {
	random_device rd;
	mt19937 gen(rd());
	uniform_int_distribution<> dis(1, 1000000); // Числа от 1 до 1 миллиона

	vector<int> arr(size);
	for (int i = 0; i < size; ++i) {
		arr[i] = dis(gen);
	}
	return arr;
}

// --- Главная функция ---
int main() {
	// Массивы разных размеров
	vector<int> sizes = { 1000, 10000, 100000, 1000000 };

	// Вектор для хранения времен сортировки для каждого размера
	vector<long long> times_tournament;
	vector<long long> times_bucket;
	vector<long long> times_cycle;

	for (int size : sizes) {
		// Генерация случайных данных
		vector<int> arr_int = generateRandomArray(size);
		vector<float> arr_float(arr_int.begin(), arr_int.end());

		// Tournament Sort
		auto start = chrono::high_resolution_clock::now();
		tournament_sort(arr_int, 16);
		auto end = chrono::high_resolution_clock::now();
		long long duration_tournament = chrono::duration_cast<chrono::microseconds>(end - start).count();
		times_tournament.push_back(duration_tournament);

		// Bucket Sort
		start = chrono::high_resolution_clock::now();
		bucketSort(arr_float);
		end = chrono::high_resolution_clock::now();
		long long duration_bucket = chrono::duration_cast<chrono::microseconds>(end - start).count();
		times_bucket.push_back(duration_bucket);

		// Cycle Sort
		start = chrono::high_resolution_clock::now();
		cycleSort(arr_int);
		end = chrono::high_resolution_clock::now();
		long long duration_cycle = chrono::duration_cast<chrono::microseconds>(end - start).count();
		times_cycle.push_back(duration_cycle);
	}

	// Вывод результатов
	cout << "Size\tTournament Sort (μs)\tBucket Sort (μs)\tCycle Sort (μs)" << endl;
	for (size_t i = 0; i < sizes.size(); ++i) {
		cout << sizes[i] << "\t" << times_tournament[i] << "\t" << times_bucket[i] << "\t" << times_cycle[i] << endl;
	}

	return 0;
}
