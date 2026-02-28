/*
Быстрая сортировка
*/
#pragma once

#include <vector>
#include <cmath>

using namespace std;

namespace quicksort {
    //поменять элементы местами
    void swap(double &a, double &b) {
        double c;
        c = a;
        a = b;
        b = c;
    }
    //разделить на меньшие и большие
    int partition(double *arr, int low, int high) {
        double pivot;
        int i, j;
        pivot = arr[high - 1];
        i = low - 1;
        for (j = low; j < high - 1; j++) {
            if (arr[j] < pivot) {
                i++;
                if (i != j) {
                    swap(arr[i], arr[j]);
                }
            }
        }
        i++;
        if (i != j) {
            swap(arr[i], arr[j]);
        }
        return i;
    }
    //быстрая сортировка
    void quicksort(double *arr, int low, int high) {
        static int pivot;
        if (low < high - 1) {
            pivot = partition(arr, low, high);
            quicksort(arr, low, pivot);
            quicksort(arr, pivot + 1, high);
        }
    }
    // int abs_partition(vector<double> &arr, int low, int high) {
    //     static double pivot;
    //     static int i, j;
    //     pivot = abs(arr[high]);
    //     i = low - 1;
    //     for (j = low; j < high; j++) {
    //         if (abs(arr[j]) < pivot) {
    //             i++;
    //             if (i != j) {
    //                 swap(arr[i], arr[j]);
    //             }
    //         }
    //     }
    //     i++;
    //     if (i != j) {
    //         swap(arr[i], arr[j]);
    //     }
    //     return i;
    // }
    // void abs_quicksort(vector<double> &arr, int low, int high) {
    //     static int pivot;
    //     if (low < high) {
    //         pivot = abs_partition(arr, low, high);
    //         abs_quicksort(arr, low, pivot - 1);
    //         abs_quicksort(arr, pivot + 1, high);
    //     }
    // }
}