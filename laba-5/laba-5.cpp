#include <stdio.h>
#include <stdlib.h>

int cycleSort(int* list, size_t l_len);
void show_array(int* array, size_t a_len);

int main() {
    int arr[] = { 30, 20, 50, 40, 10 }; 
    size_t arr_k = sizeof(arr) / sizeof(arr[0]); // Получаем количество элементов в массиве
    // Это O(1) по времени, так как количество операций фиксировано 
    // (арифметические операции и вызов sizeof).

    int writes;

    show_array(arr, arr_k);
    // Время выполнения O(n), так как выводим n элементов, 
    // память O(1) для переменной ix и временных ресурсов printf.

    writes = cycleSort(arr, arr_k);
    // Время выполнения O(n^2) в худшем и среднем случаях 
    // из-за вложенных циклов. Память O(1), так как не используем дополнительных структур данных, кроме нескольких переменных.

    show_array(arr, arr_k);
    // Время выполнения O(n), память O(1).

    printf("writes: %d\n", writes);
    // Время O(1), память O(1).

    return 0; 
}

void show_array(int* array, size_t a_len) {
    for (int ix = 0; ix < a_len; ++ix) {
        printf("%d ", array[ix]); 
        // Время O(n), так как n элементов. Память O(1) для временной переменной ix.
    }
    putchar('\n');

    return;
}

int cycleSort(int* list, size_t l_len) {
    int writes = 0; 
    // Время O(1), память O(1).

    for (int cycle_start = 0; cycle_start < l_len - 1; cycle_start++) { // O(n)
        // Время O(n) в худшем случае (n-1 итерация на внешнем цикле).
        int item = list[cycle_start];
        // Время O(1), память O(1) для переменной item.

        int pos = cycle_start; // Инициализация переменной pos
        // Время O(1), память O(1).

        for (int i = cycle_start + 1; i < l_len; i++) { // O(n) - внутренний цикл для поиска позиции
            // В худшем случае, для каждого элемента, выполняется O(n), 
            // что приводит к O(n^2) как общий производительность метода.
            if (list[i] < item) {
                pos++; // Увеличиваем позицию, если элемент меньше
                // Время O(1), память O(1).
            }
        }

        while (item == list[pos]) {
            pos++; // Если элемент равен текущему, увеличиваем позицию
            // Время O(n) в худшем случае, если все элементы равны.
        }

        if (pos != cycle_start) {
            while (item == list[pos]) {
                pos++; 
                // Время O(n) в худшем случае.
            }

            item = list[pos]; 
            list[pos] = list[cycle_start];
            writes++; 
            // Время O(1), память O(1).
        }
    }
    return writes;
    // Время O(1), память O(1).
}