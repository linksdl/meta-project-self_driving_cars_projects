//templates what gets printed

#include <iostream>

using namespace std;

template <class T1, class T2>

int mystery(T1& a, T2 b, int c)

{

    T1 t = a;

    a = a + b;

    return ( a - t + c);

}

int main(void)

{

    int a = 3;

    double b = 2.5;

    int c = 1;

    cout << " answer 1 is  " << mystery(a, 2, c) << endl;

    cout << " answer 2 is  " << mystery(a, 1, c) << endl;

    a = 5;

    cout << " answer 3 is  " << mystery(a, b, c) << endl;

    a = 2;

    b = 2.5;

    cout << " answer 4 is  " << mystery(a, b, b) << endl;

    cout << " answer 5 is  " << mystery(a,mystery(a, b, b),c)<< endl;

    return 0;

}