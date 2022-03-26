
#include <iostream>
#include <iomanip>

int main()
{
    std::cout<<"\n\nThe text without any formating\n";
    std::cout<<"Ints"<<"Floats"<<"Doubles"<< "\n";
    std::cout<<"\nThe text with setw(15)\n";
    std::cout<<"Ints"<<std::setw(15)<<"Floats"<<std::setw(15)<<"Doubles"<< "\n";
    std::cout<<"\n\nThe text with tabs\n";
    std::cout<<"Ints\t"<<"Floats\t"<<"Doubles"<< "\n";

    return 0;
}