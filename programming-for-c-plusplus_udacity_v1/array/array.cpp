/*Find the min and max and average of 15 integers that a user will input.
**We will do it now for practice and again when we learn arrays. So you do not have to **keep all fifteen numbers stored in memory.
*/

#include <iostream>


int main()
{
    int userInput = 0;
    int maxNumber = 0;
    int minNumber = 100;
    int sumTotal = 0;
    float average = 0;

    // get the numbers from the user
    for (int i = 0; i < 15; i++)
    {
        std::cout << "Enter a number: ";
        std::cin>>userInput;
        std::cout << userInput << "\n";
        if(userInput > maxNumber)
        {
            maxNumber = userInput;
        }
        if(userInput < minNumber)
        {
            minNumber = userInput;
        }
        sumTotal = sumTotal + userInput;
    }

    std::cout << "Maximum number = " << maxNumber << "\n";
    std::cout << "Minimum number = " << minNumber << "\n";
    average = sumTotal / 15.0;
    std::cout << "Average = " << average << "\n";
    return 0;

}