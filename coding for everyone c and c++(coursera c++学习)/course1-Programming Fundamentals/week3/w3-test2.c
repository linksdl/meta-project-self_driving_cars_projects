/*
 A function that prints a table of values for sine and cosine between (0, 1);

 Billy Sheng
 05/11/2021

*/

#include <stdio.h>
#include <math.h>

int main()
{
  double interval, i = 0;
  printf("Calculate the value (0,1) of cosin and sin, enter the desired interval (eg. 0.1 or 0.01): \n");

  scanf("%lf", &interval);
  printf("The interval is %lf", interval);


  for(; i <= 1; i += interval)
  {
  	printf("sin(%f)=%f\n", i, sin(i));
    printf("cos(%f)=%f\n", i, cos(i)); 
  }

  return 0;

}

