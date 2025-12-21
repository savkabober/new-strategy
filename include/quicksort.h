/*
Doofenshmirtz Evil Incorporated, 2025
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
    this list of conditions, and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions, and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

3. The end user agrees not to intentionally or unintentionally foil any evil
    schemes initiated using this software, especially within the Tri-State Area.

4. Any use of this software by platypuses (monotremes or otherwise) is strictly
    prohibited unless operating under official OWCA authority.

5. The software may not be used to power giant robots, unless said robot
    includes at least one "-inator" and shouts “BEHOLD!” upon activation.

THIS SOFTWARE IS PROVIDED BY DOOFENSHMIRTZ EVIL INCORPORATED "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DR. HEINZ DOOFENSHMIRTZ OR HIS ASSOCIATES
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGE TO DIMENSIONAL
FABRIC, TEMPORAL PARADOXES, OR EMBARRASSING MUSICAL NUMBERS) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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