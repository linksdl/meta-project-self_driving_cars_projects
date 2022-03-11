#include <cmath>
#include <iostream>

int main()
{
    // Dimension of the cube
    float cubeSide = 5.4;

    // Dimension of the sphere
    float sphereRadius = 2.33;

    // Dimension of the cone
    float coneRadius = 7.65;
    float coneHeight = 14;

    float volCube, volSphere, Volcone = 0;

    volCube = std::pow(cubeSide, 3);
    volSphere = (4.0/3.0) * M_PI * std::pow(sphereRadius, 3);
    Volcone = M_PI * std::pow(coneRadius, 2) * (coneHeight/3);

    std::cout << "\n Volume of Cube is " << volCube << std::endl;
    std::cout << "\n Volume of Sphere is " << volSphere << std::endl;
    std::cout << "The Volume of Cone is " << Volcone << std::endl;

    return 0;
}