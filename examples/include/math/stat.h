//
// Created by waxz on 23-3-2.
//

#ifndef SCAN_REPUBLISHER_STAT_H
#define SCAN_REPUBLISHER_STAT_H
#include <cmath>

namespace math{
    //finding mean of the ungrouped data in array
    template<typename T>
    T mean(T* arr, int n){
        T sum = 0;
        for(int i = 0;i < n; i++)
            sum += arr[i];

        return sum/n;
    }
//finding median of the ungrouped data in the array
    template<typename T>
    T median(T* arr, int n){
        //sort the array
        std::sort(arr, arr + n);
        if(n % 2 == 0)
            return (arr[n/2 - 1] + arr[n/2])/2;
        return arr[n/2];
    }
//finding mode of ungrouped data
    template<typename T>
    T mode( T* arr, int n){
        // Sort the array
        std::sort(arr, arr + n);

        //finding max frequency
        int max_count = 1, res = arr[0], count = 1;
        for (int i = 1; i < n; i++) {
            if (arr[i] == arr[i - 1])
                count++;
            else {
                if (count > max_count) {
                    max_count = count;
                    res = arr[i - 1];
                }
                count = 1;
            }
        }

        // when the last element is most frequent
        if (count > max_count)
        {
            max_count = count;
            res = arr[n - 1];
        }

        return res;
    }
}
#endif //SCAN_REPUBLISHER_STAT_H
