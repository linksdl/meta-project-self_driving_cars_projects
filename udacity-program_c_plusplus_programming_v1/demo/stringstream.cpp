/*Goal: practice getting string inputs and
 **converting them to numeric variables using
 **stringstream.
 **
 **Prompt the user for the length of a room.
 **Then prompt for the width of the room.
 **Print out the area of the room.
 */

#include <iostream>
#include <string>
#include <sstream>

int main ()
{
    std::string stringLength, stringWidth;
    float length = 0;
    float width = 0;
    float area = 0;

    std::cout << "Enter the length of the room: ";
    //get the length as a string
    std::getline (std::cin,stringLength);
    //convert to a float
    std::stringstream(stringLength) >> length;
    //get the width as a string
    std::cout << "Enter width: ";
    std::getline (std::cin,stringWidth);
    //convert to a float
    std::stringstream(stringWidth) >> width;
    area = length * width;
    std::cout << "\nThe area of the room is: " << area << std::endl;
    return 0;
}
