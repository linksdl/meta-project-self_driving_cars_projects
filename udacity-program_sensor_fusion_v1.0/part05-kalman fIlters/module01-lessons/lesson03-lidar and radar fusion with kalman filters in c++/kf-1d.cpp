// Write a function 'filter()' that implements a multi-
// dimensional Kalman Filter for the example given
//============================================================================
#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using namespace Eigen;

//Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);



int main()
{
    // design the KF with 1D motion

    x = VectorXd(2);  // object state
    x << 0, 0;

    P = MatrixXd(2, 2); // object covariance matrix
    P << 100, 0, 0, 100;

    u = VectorXd(2);  // external motion
    u << 0, 0;

    F = MatrixXd(2, 2);  // state transition matrix
	F << 1, 1, 0, 1;

    Q = MatrixXd(2, 2);  // process covariance matrix
	Q << 0, 0, 0, 0;

    // -------------------------------------------------------------

    H = MatrixXd(1, 2);  // measurement matrix
	H << 1, 0;

    R = MatrixXd(1, 1);  // measurement covariance matrix
	R << 1;

    I = MatrixXd::Identity(2, 2); // Identity matrix


    // create a list of measurements
    VectorXd single_measure (1);
    single_measure << 1;
    measurements.push_back(single_measure);

    single_measure << 2;
    measurements.push_back(single_measure);

    single_measure << 3;
    measurements.push_back(single_measure);


    // call Kalman Filter algorothm
    filter(x, P);

    return 0;
}

void filter(VectorXd &x, MatrixXd &P) 
{
     for (unsigned int n = 0; n < measurements.size(); ++n)
     {
         VectorXd z = measurements[n];
         // Your CODE HERE

         // KF Measurement update step
         VectorXd  y = z -  H * x;
         MatrixXd  H_T =  H.transpose();
         MatrixXd  S = H * P * H_T  + R;
         MatrixXd  S_I = S.inverse();
         MatrixXd  K = P * H_T * S_I;

        
         // new state
         x = x + (K * y);
         P = (I - K * H) * P;

         // KF Prediction step
        x = F * x + u;
        MatrixXd F_T = F.transpose();
        P = F * P * F_T + Q;

         std:cout << "x=" << std::endl <<  x << std::endl;
         std::cout << "P=" << std::endl << P <<  std::endl;


     }
}