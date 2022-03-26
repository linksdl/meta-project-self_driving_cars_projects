/*Goal: Find out why you care about pointers*/

#include<iostream>
#include<string>

int main ()
{
    int * pointerI;
    int number;
    char character;
    char * pointerC;
    std::string sentence;
    std::string *pointerS;

    pointerI = &number;
    *pointerI = 45;

    pointerC = &character;
    *pointerC = 'f';

    pointerS = &sentence;
    *pointerS = "Hey look at me, I know pointers!";

    std::cout << "number = "<<number<<"\n";
    std::cout<<"character = "<<character<<"\n";
    std::cout<<"sentence = "<<sentence<<"\n";

    return 0;
}