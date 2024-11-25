

// new ice thickness algo:
//last algorithm did not compute average correclty:
//Date: 11/20/2024
#include<cmath>
#include<iostream>


double ice_thick(double daycount, double temp_sum, double new_temp){


    if (new_temp <= 0){
        return 0;
    }

    temp_sum = temp_sum + new_temp;

    daycount = daycount + 1;

    int ice_thickness;

    int av_temp;

    av_temp = temp_sum/daycount;
    int ice_thickness = sqrt(daycount*av_temp)/0.08;

    return ice_thickness;
    
}
