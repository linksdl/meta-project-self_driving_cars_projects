/*
* Write a program that asks a user for five nunbers,
* Print out the sum and average of the five numbers
*
*
*/

#include <iostream>

int main()
{
    float input;
    float sum = 0;

    for(int i=0; i <5; i++)
    {
        std::cout << "What is the next numbers?";
        std::cin >> input;

        sum += input;

    } // end of the loop

    std::cout << "Sum = " << sum <<"\n";
    std::cout << "Average = " << sum/5 << "\n";

    return 0;
}