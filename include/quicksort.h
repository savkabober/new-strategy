#pragma once

#include <vector>
#include <cmath>

using namespace std;

namespace quicksort {
    void swap(double &a, double &b) {
        double c;
        c = a;
        a = b;
        b = c;
    }
    int partition(double *arr, int low, int high) {
        double pivot;
        int i, j;
        pivot = arr[high];
        i = low - 1;
        for (j = low; j < high; j++) {
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
    void quicksort(double *arr, int low, int high) {
        static int pivot;
        if (low < high) {
            pivot = partition(arr, low, high);
            quicksort(arr, low, pivot - 1);
            quicksort(arr, pivot + 1, high);
        }
    }
    int abs_partition(vector<double> &arr, int low, int high) {
        static double pivot;
        static int i, j;
        pivot = abs(arr[high]);
        i = low - 1;
        for (j = low; j < high; j++) {
            if (abs(arr[j]) < pivot) {
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
    void abs_quicksort(vector<double> &arr, int low, int high) {
        static int pivot;
        if (low < high) {
            pivot = abs_partition(arr, low, high);
            abs_quicksort(arr, low, pivot - 1);
            abs_quicksort(arr, pivot + 1, high);
        }
    }
}