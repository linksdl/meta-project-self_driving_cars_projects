#include <iostream>
/*
    this is a block comments
*/
// using namespace std;

int main()
{
    std::cout << "Hello World !\n";
    // print a message to the uers.
    std::cout << "Hello world, i am ready for c++ \n";
    // return the result.

    int integer = 33;
    std::cout<<"The value of integer is "<<integer;

    std::cout << "int size " <<sizeof(int);
    std::cout << "short size " <<sizeof(short);
    std::cout << "long size " <<sizeof(long);
    std::cout << "char size " <<sizeof(char);
    std::cout << "float size " <<sizeof(float);
    std::cout << "double size " <<sizeof(double);
    std::cout << "bool size " <<sizeof(bool);

    const int weightGoal = 100;
    std::cout << "The weightGoal is:"<< weightGoal;


    enum MONTH {Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};

    MONTH bestMonth;

    bestMonth = Jan;

    if(bestMonth == Jan)
    {
        std::cout << "I'm not so sure Janauary is the best month. \n";
    }

    return 0;
}

