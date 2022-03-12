// constructing vectors
#include <iostream>
#include <vector> // Need to include the vector library!

int main()
{
    // crearting a vector od integers
    std::vector<int> vectorInts;
    std::cout<<"vectorInts has "<<vectorInts.size()<<" elements\n";

    // Changing the size of vectorInts to 6
    vectorInts.resize(6);
    std::cout<<"\n\nvectorInts now has "<<vectorInts.size()<<" elements\n";

    return 0;
}