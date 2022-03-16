/**
 * @file test2.c
 * @author Billy Sheng (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <stdio.h>
#include <math.h>


int main()
{
    /**
     * @brief 
     * calcluate the sine of a value between 0 and 1 (non inclusive). 
     * 
     */
    double sine_value, x;

    printf("Please Enter a x between [0,1) to calulate its value of sine: \n");
    scanf("%lf", &x);

    sine_value = sin(x);

    printf("the sine value of %lf is %lf ", x, sine_value);
    
    return 0;
}

