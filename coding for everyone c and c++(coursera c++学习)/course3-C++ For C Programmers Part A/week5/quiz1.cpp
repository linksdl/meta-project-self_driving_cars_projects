//What does the following print

#include <iostream>

#include <vector>

#include <numeric>

using namespace std;

int main()

{

    vector<int> data(5,1);

    int sum {0};

    cout << sum << endl;

    for(auto element : data)

        sum += element;

    cout << sum << endl;

    for(auto p = ++data.begin(); p != --data.end(); ++p)

        sum += *p;

    cout << sum << endl;

    sum = 0;

    data.push_back(2);

    data.push_back(3);

    for(auto element : data)

        sum += element;

    cout << sum << endl;

    cout << accumulate(data.begin(), data.end(), data[0]) << endl;

}