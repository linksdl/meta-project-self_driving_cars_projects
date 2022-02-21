/*

 Billy Sheng
 03/11/2021
*/
#include<stdio.h>
#define PI 3.14159

int main(void)
{ 
    float radius;
    printf("Enter radius:");
    scanf("%lf", &radius);
    printf("volume is : %lf \n\n", 4 * radius * radius * radius / 3.0);
    return 0;
}