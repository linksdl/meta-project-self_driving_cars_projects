
#include <iostream>
#include <istream>
#include <string>

int main()
{
    int age = 0;
    std::cout << "What's your age?";      

    std::cin >> age;

    std::cout << "Your age is: " << age;
    return 0;
}