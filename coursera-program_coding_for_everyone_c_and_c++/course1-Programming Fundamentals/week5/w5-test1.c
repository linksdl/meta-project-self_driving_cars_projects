/*
 read them into an array and 
 compute the average weight for the set of the elephant seals.

 Billy Sheng
 06/11/2021

*/

#include <stdio.h>


int main()
{
    int number_of_elephant_seals, elephant_seals_weights=[100],
    average_weight, sum = 0, i;
  int sum;
  double avg;
  for(i = 0; i < n; i++)
  {
  	sum = sum + weights[i];
  }

  avg = sum / n;
  printf("The average weight for the set of the elephant selas is: %lf", avg);
  return 0;
}

//#include <stdio.h>
//
//void main()
//
//{
//
//    int number_of_elephant_seals,elephant_seals[100],
//
//            average_weight,sum=0,i;
//    printf("Enter the number of elephants seals: ");
//    scanf("%d",&number_of_elephant_seals);
//    printf("\nEnter the weights of the %d elephant seals:\n",number_of_elephant_seals);
//
//    for(i=1;i<=number_of_elephant_seals;i++)
//
//    {
//        scanf("%d",&elephant_seals[i]);
//    }
//
//    for(i=1;i<=number_of_elephant_seals;i++)
//
//    {
//        sum=sum+elephant_seals[i];
//    }
//    average_weight=sum/number_of_elephant_seals;
//
//    printf("\nThe average weight for a population of %d elephants seals is equal to =  %d kg" ,number_of_elephant_seals,average_weight);
//
//}