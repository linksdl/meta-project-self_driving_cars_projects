
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main()
{
    string line;

    ofstream myFile ("intput.txt", ios::app);
    if (myFile.is_open())
    {
        myFile << "\n I am adding a line. \n";
        myFile << "I am adding another line. \n";
        myFile.close();
    }else
    {
        cout << "Uable to open this file for writing.";
    }

    //create an input stream to read the file
    ifstream myfileO("input.txt");

    if (myfileO.is_open())
    {
        while ( getline (myfileO, line))
        {
            cout << line << "\n";
        }
        myfileO.close();
    }
    else cout << "Unable to open file for reading";

    return 0;
}