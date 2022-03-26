#include  <iostream>
#include <vector>
#include "Eigen/Dense"


using namespace std;
using namespace Eigen;


int main() 
{
    /**
	 * Code used as example to work with Eigen matrices
	 */

    // you can create a  vertical vector of two elements with a command like this
    VectorXd my_vect (2);

    // you can create a  vertical vector of two elements with a command like this
    my_vect << 10, 20;

    // and you can use the cout command to print out the vector
    cout << my_vect << endl;


    // the matrices can be created in the same way.
    // For example, This is an initialization of a 2 by 2 matrix
    // with the values 1, 2, 3, and 4
    MatrixXd my_mat(2, 2);
    my_mat << 10, 20,  30, 40;
    cout << my_mat << endl;


    // you can use the same comma initializer or you can set each matrix value explicitly
    // For example that's how we can change the matrix elements in the second row
    my_mat(1, 0) = 11;  // second row, first column
    my_mat(1, 1) = 22; // second row, second column
    cout << my_mat << endl;


    // Also, you can compute the transpose of a matrix with the following command
    // 转置
    MatrixXd my_mat_T = my_mat.transpose();
    cout << my_mat_T<< endl;

    // And here is how you can get the matrix inverse
    // 逆矩阵
    MatrixXd my_mat_I = my_mat.inverse();
    cout << my_mat_I << endl;


    // For multiplying the matrix m with the vector b you can write this in one line as let’s say matrix c equals m times v.
    MatrixXd another_matrix;
    another_matrix = my_mat*my_vect;
    cout << another_matrix << endl;




    return 0;
}

