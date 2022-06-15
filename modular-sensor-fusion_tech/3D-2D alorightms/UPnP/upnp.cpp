//M*//////////////////////////////////////////////////////////////////////////////////////
//
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
// By downloading, copying, installing or using the software you agree to this license.
// If you do not agree to this license, do not download, install,
// copy or use the software.
//
//
// License Agreement
// For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistribution's of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// * Redistribution's in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// * The name of the copyright holders may not be used to endorse or promote products
// derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/****************************************************************************************\
* Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation.
* Contributed by Edgar Riba
\****************************************************************************************/

#include "precomp.hpp"
#include "upnp.h"
#include <limits>

using namespace std;
using namespace cv;

upnp::upnp(const Mat& cameraMatrix, const Mat& opoints, const Mat& ipoints)
{
  if (cameraMatrix.depth() == CV_32F)
    init_camera_parameters<float>(cameraMatrix);
  else
    init_camera_parameters<double>(cameraMatrix);

  number_of_correspondences = std::max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));    // 多少组2d-3d对应点

  pws.resize(3 * number_of_correspondences);                   // 世界坐标系中的3D点
  us.resize(2 * number_of_correspondences);                    // 图像坐标系中的2D点

  if (opoints.depth() == ipoints.depth())
  {
    if (opoints.depth() == CV_32F)                             // 初始化，把opoints和ipoints中的点存到pws和us中
      init_points<Point3f,Point2f>(opoints, ipoints);
    else
      init_points<Point3d,Point2d>(opoints, ipoints);
  }
  else if (opoints.depth() == CV_32F)
    init_points<Point3f,Point2d>(opoints, ipoints);
  else
    init_points<Point3d,Point2f>(opoints, ipoints);

  alphas.resize(4 * number_of_correspondences);
  pcs.resize(3 * number_of_correspondences);

  max_nr = 0;
  A1 = NULL;
  A2 = NULL;
}

upnp::~upnp()
{
  if (A1)
    delete[] A1;
  if (A2)
    delete[] A2;
}

double upnp::compute_pose(Mat& R, Mat& t)
{
  choose_control_points();                                                    // 选择4个控制点（质点+3个主轴方向的单位向量）
  compute_alphas();                                                           // 根据4个控制点计算所有3d空间点的阿尔法系数
                                                                              // pi = ai_0*cws0 + ai_1*cws1 + ai_2*cws2 + ai_3*cws3
  Mat * M = new Mat(2 * number_of_correspondences, 12, CV_64F);               // Mx=0 M为2n*12的矩阵，x为4个控制点在相机坐标系下的x,y,z/f值，为12*1矩阵

  for(int i = 0; i < number_of_correspondences; i++)
  {
    fill_M(M, 2 * i, &alphas[0] + 4 * i, us[2 * i], us[2 * i + 1]);           // 填充M矩阵（由阿尔法，u,v和f,cx,cy组成）(cx和cy默认为图像中心, fx=fy=f)
  }

  double mtm[12 * 12], d[12], ut[12 * 12], vt[12 * 12];
  Mat MtM = Mat(12, 12, CV_64F, mtm);
  Mat D   = Mat(12,  1, CV_64F, d);
  Mat Ut  = Mat(12, 12, CV_64F, ut);
  Mat Vt  = Mat(12, 12, CV_64F, vt);

  MtM = M->t() * (*M);
  SVD::compute(MtM, D, Ut, Vt, SVD::MODIFY_A | SVD::FULL_UV);                 // 对MtM进行svd分解，得到U和Vt
  Mat(Ut.t()).copyTo(Ut);                                                     // 得到U的转置（特征向量变为 行向量）
  M->release(); 
  delete M;

  double l_6x12[6 * 12], rho[6];                                              //  L * betas = rho
  Mat L_6x12 = Mat(6, 12, CV_64F, l_6x12);                                    // 6*12  12*1   6*1
  Mat Rho    = Mat(6,  1, CV_64F, rho);

  compute_L_6x12(ut, l_6x12);                                                 // N=3时，betas为|b11, b12, b13, b22, b23, b33, ffb11, ffb12, ffb13, ffb22, ffb23, ffb33|
  compute_rho(rho);                                                           // rho 一直为 6*1 的矩阵，记录着4个控制点之间各自的距离

  double Betas[3][4], Efs[3][1], rep_errors[3];
  double Rs[3][3][3], ts[3][3];

  find_betas_and_focal_approx_1(&Ut, &Rho, Betas[1], Efs[1]);                 // N=1, betas:|b11, ffb11|; 计算betas和f
  gauss_newton(&L_6x12, &Rho, Betas[1], Efs[1]);                              // 根据rho的误差，迭代精确（提纯）betas
  rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);                // 利用v，betas和f，先求出4个ccs，再求出pcs，然后利用pcs和pws计算R和t，最后计算重投影误差

  find_betas_and_focal_approx_2(&Ut, &Rho, Betas[2], Efs[2]);                 // N=2, betas:|b11, b12, b22, ffb11, ffb12, ffb22|; 计算betas和f
  gauss_newton(&L_6x12, &Rho, Betas[2], Efs[2]);
  rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;                                   // 选出重投影误差最小的R，t和f

  Mat(3, 1, CV_64F, ts[N]).copyTo(t);
  Mat(3, 3, CV_64F, Rs[N]).copyTo(R);
  fu = fv = Efs[N][0];

  return fu;
}

void upnp::copy_R_and_t(const double R_src[3][3], const double t_src[3],
     double R_dst[3][3], double t_dst[3])
{
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++)
      R_dst[i][j] = R_src[i][j];
    t_dst[i] = t_src[i];
  }
}

void upnp::estimate_R_and_t(double R[3][3], double t[3])                      // 计算R和t  （Horn "Closed-form solution of absolute orientation using orthonormal matrices"）
{
  double pc0[3], pw0[3];

  pc0[0] = pc0[1] = pc0[2] = 0.0;
  pw0[0] = pw0[1] = pw0[2] = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    const double * pc = &pcs[3 * i];
    const double * pw = &pws[3 * i];

    for(int j = 0; j < 3; j++) {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++) {                                                // 求出pcs和pws的几何中心
    pc0[j] /= number_of_correspondences;
    pw0[j] /= number_of_correspondences;
  }

  double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
  Mat ABt   = Mat(3, 3, CV_64F, abt);
  Mat ABt_D = Mat(3, 1, CV_64F, abt_d);
  Mat ABt_U = Mat(3, 3, CV_64F, abt_u);
  Mat ABt_V = Mat(3, 3, CV_64F, abt_v);

  ABt.setTo(0.0);
  for(int i = 0; i < number_of_correspondences; i++) {
    double * pc = &pcs[3 * i];
    double * pw = &pws[3 * i];

    for(int j = 0; j < 3; j++) {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);                  // 对 各个pcs和pws点减去各自的几何中心所形成的矩阵 进行svd分解
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  SVD::compute(ABt, ABt_D, ABt_U, ABt_V, SVD::MODIFY_A);
  Mat(ABt_V.t()).copyTo(ABt_V);

  for(int i = 0; i < 3; i++)                                                  // 计算旋转矩阵
    for(int j = 0; j < 3; j++)
      R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

  const double det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  t[0] = pc0[0] - dot(R[0], pw0);                             // 计算平移矩阵
  t[1] = pc0[1] - dot(R[1], pw0);
  t[2] = pc0[2] - dot(R[2], pw0);
}

void upnp::solve_for_sign(void)
{
  if (pcs[2] < 0.0) {                                         // 检查第一个相机坐标系下的3d点，若发现深度为负，则调整所有ccs和所有pcs使其坐标非负
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
        ccs[i][j] = -ccs[i][j];

    for(int i = 0; i < number_of_correspondences; i++) {
      pcs[3 * i    ] = -pcs[3 * i];
      pcs[3 * i + 1] = -pcs[3 * i + 1];
      pcs[3 * i + 2] = -pcs[3 * i + 2];
    }
  }
}

double upnp::compute_R_and_t(const double * ut, const double * betas,
         double R[3][3], double t[3])
{
  compute_ccs(betas, ut);                                    // 计算控制点在相机坐标系下的坐标 x=betas*v， x=[xc,yc,zc/f,...]
  compute_pcs();                                             // 计算3d点在相机坐标系下的坐标 pcsi = ai_0*cws0 + ai_1*cws1 + ai_2*cws2 + ai_3*cws3

  solve_for_sign();                                          // 保证pcs和ccs坐标非负
 
  estimate_R_and_t(R, t);                                    // 计算R和t

  return reprojection_error(R, t);                           // 计算重投影误差并返回其值
}

double upnp::reprojection_error(const double R[3][3], const double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    double * pw = &pws[3 * i];
    double Xc = dot(R[0], pw) + t[0];                                     // pws经外参（R和t）变为pcs
    double Yc = dot(R[1], pw) + t[1];
    double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
    double ue = uc + fu * Xc * inv_Zc;                                    // pcs经内参变为uv
    double ve = vc + fv * Yc * inv_Zc;
    double u = us[2 * i], v = us[2 * i + 1];

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );            // 计算与真实2d点之间的误差
  }

  return sum2 / number_of_correspondences;
}

void upnp::choose_control_points()
{
    for (int i = 0; i < 4; ++i)                                           // 4个控制点可随意选择，UPnP中简单的将cws[3]设为原点，cws[1]，[2]，[3]为x，y，z轴的单位向量
      cws[i][0] = cws[i][1] = cws[i][2] = 0.0;
    cws[0][0] = cws[1][1] = cws[2][2] = 1.0;                              // EPnP中选择pws的质点作为cws[0]，PCA后的主轴单位向量作为cws[1]，[2]，[3]，提高了算法的稳定性
}

void upnp::compute_alphas()
{
    Mat CC = Mat(4, 3, CV_64F, &cws);
    Mat PC = Mat(number_of_correspondences, 3, CV_64F, &pws[0]);
    Mat ALPHAS = Mat(number_of_correspondences, 4, CV_64F, &alphas[0]);

    Mat CC_ = CC.clone().t();
    Mat PC_ = PC.clone().t();

    Mat row14 = Mat::ones(1, 4, CV_64F);
    Mat row1n = Mat::ones(1, number_of_correspondences, CV_64F);          //  n*4           4*4                  4*n
                                                                          //            | 0 1 0 0 |^-1   | pws1_x pws2_x ... |
    CC_.push_back(row14);                                                 // alphas = ( | 0 0 1 0 |    * | pws1_y pws2_y ... | ) ^t
    PC_.push_back(row1n);                                                 //            | 0 0 0 1 |      | pws1_z pws2_z ... |
                                                                          //            | 1 1 1 1 |      |   1      1    ... |
    ALPHAS = Mat( CC_.inv() * PC_ ).t();
}

void upnp::fill_M(Mat * M, const int row, const double * as, const double u, const double v)       // M 的推导见论文 很简单 (这里的代码有问题，f是未知的) ==============================================================================
{
  double * M1 = M->ptr<double>(row);       //     | ai_0  0  ai_0*(cx-ui)  ai_1  0  ai_1*(cx-ui)  ai_2  0  ai_2*(cx-ui)  ai_3  0  ai_3*(cx-ui) | -> M1
  double * M2 = M1 + 12;                   //     | 0  ai_0  ai_0*(cy-vi)  0  ai_1  ai_1*(cy-vi)  0  ai_2  ai_2*(cy-vi)  0  ai_3  ai_3*(cy-vi) | -> M2
                                           //     |    .  .             .     .  .             .     .  .             .     .  .             . |
  for(int i = 0; i < 4; i++) {             //     |    .  .             .     .  .             .     .  .             .     .  .             . |
    M1[3 * i    ] = as[i] * fu;            //     |    .  .             .     .  .             .     .  .             .     .  .             . |
    M1[3 * i + 1] = 0.0;                   // M = |    .  .             一组点提供两个式子，n组点 调用n次 fill_M 函数 来填充M矩阵  .             . |
    M1[3 * i + 2] = as[i] * (uc - u);      //     |    .  .             .     .  .             .     .  .             .     .  .             . |
                                           //     |    .  .             .     .  .             .     .  .             .     .  .             . |
    M2[3 * i    ] = 0.0;                   //     |    .  .             .     .  .             .     .  .             .     .  .             . |
    M2[3 * i + 1] = as[i] * fv;            //     | an_0  0  an_0*(cx-ui)  an_1  0  an_1*(cx-ui)  an_2  0  an_2*(cx-ui)  an_3  0  an_3*(cx-ui) |
    M2[3 * i + 2] = as[i] * (vc - v);      //     | 0  an_0  an_0*(cy-vi)  0  an_1  an_1*(cy-vi)  0  an_2  an_2*(cy-vi)  0  an_3  an_3*(cy-vi) |
  }
}

void upnp::compute_ccs(const double * betas, const double * ut)
{
    for(int i = 0; i < 4; ++i)                                    // 计算控制点在相机坐标系下的坐标
      ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0;

    int N = 4;
    for(int i = 0; i < N; ++i) {
      const double * v = ut + 12 * (9 + i);                       // 这里代码有问题，如果按照find_betas_and_focal_approx函数中求betas的方法，=========================================================================
      for(int j = 0; j < 4; ++j)                                  // 此处特征向量应该倒着来，v = ut + 12 * (11 - i);  而且N=1,2,3，所以这里v最多为3个
        for(int k = 0; k < 3; ++k)
          ccs[j][k] += betas[i] * v[3 * j + k];
    }

    for (int i = 0; i < 4; ++i) ccs[i][2] *= fu;                  // 因为Mx=0求出的x的第三项为zc/f，所以此处应乘以f得到zc的值
}

void upnp::compute_pcs(void)                                      // 计算3d点在相机坐标系下的坐标
{
  for(int i = 0; i < number_of_correspondences; i++) {
    double * a = &alphas[0] + 4 * i;
    double * pc = &pcs[0] + 3 * i;
                                                                  // pcsi = ai_0*cws0 + ai_1*cws1 + ai_2*cws2 + ai_3*cws3
    for(int j = 0; j < 3; j++)                                   
      pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
  }
}

void upnp::find_betas_and_focal_approx_1(Mat * Ut, Mat * Rho, double * betas, double * efs)
{
  Mat Kmf1 = Mat(12, 1, CV_64F, Ut->ptr<double>(11));                                 // 最后一行特征向量
  Mat dsq = Mat(6, 1, CV_64F, Rho->ptr<double>(0));                                   // rho

  Mat D = compute_constraint_distance_2param_6eq_2unk_f_unk( Kmf1 );                  // 这个D就是L:6*2（N=1）
  Mat Dt = D.t();

  Mat A = Dt * D;                                                                     // L^t * L * betas = L^t * rho
  Mat b = Dt * dsq;                                                                   //     A   *   x   =     b

  Mat x = Mat(2, 1, CV_64F);                                                          //   x = |    beta11   | 
  solve(A, b, x);                                                                     //       | ff * beta11 |

  betas[0] = sqrt( abs( x.at<double>(0) ) );                                          // beta1 = sqrt(beta11)
  betas[1] = betas[2] = betas[3] = 0.0;

  efs[0] = sqrt( abs( x.at<double>(1) ) ) / betas[0];                                 // f = sqrt(ff * beta11) / beta1
}

void upnp::find_betas_and_focal_approx_2(Mat * Ut, Mat * Rho, double * betas, double * efs)
{
  double u[12*12];
  Mat U = Mat(12, 12, CV_64F, u);
  Ut->copyTo(U);

  Mat Kmf1 = Mat(12, 1, CV_64F, Ut->ptr<double>(10));                                 // 倒数第二行 ==========================================================================================================
  Mat Kmf2 = Mat(12, 1, CV_64F, Ut->ptr<double>(11));                                 // 倒数第一行
  Mat dsq = Mat(6, 1, CV_64F, Rho->ptr<double>(0));                                   // rho

  Mat D = compute_constraint_distance_3param_6eq_6unk_f_unk( Kmf1, Kmf2 );            // 这个D就是L：6*6（N=2）

  Mat A = D;                                                                          // L * betas = rho
  Mat b = dsq;                                                                        // A *   x   =  b

  double x[6];                                                                        // x = [beta11 beta12 beta22 ffbeta11 ffbeta12 ffbeta22]^t
  Mat X = Mat(6, 1, CV_64F, x);

  solve(A, b, X, DECOMP_QR);                                                          // QR分解A，求x                                    

  double solutions[18][3];                                                            // 针对beta的6个值 C63=20 - b11 b12 b22 - ffb11 ffb12 ffb22 = 18种组合
  generate_all_possible_solutions_for_f_unk(x, solutions);                            // 对每一种组合进行计算，求出beta1,beta2和f

  // find solution with minimum reprojection error
  double min_error = std::numeric_limits<double>::max();
  int min_sol = 0;
  for (int i = 0; i < 18; ++i) {

    betas[3] = solutions[i][0];
    betas[2] = solutions[i][1];
    betas[1] = betas[0] = 0.0;
    fu = fv = solutions[i][2];

    double Rs[3][3], ts[3];
    double error_i = compute_R_and_t( u, betas, Rs, ts);                              // 计算每一组beta1，beta2和f对应的重投影误差

    if( error_i < min_error)
    {
      min_error = error_i;
      min_sol = i;
    }
}

  betas[0] = solutions[min_sol][0];                                                   // 返回最小重投影误差对应的beta1，beta2和f
  betas[1] = solutions[min_sol][1];
  betas[2] = betas[3] = 0.0;

  efs[0] = solutions[min_sol][2];
}

Mat upnp::compute_constraint_distance_2param_6eq_2unk_f_unk(const Mat& M1)
{
  Mat P = Mat(6, 2, CV_64F);

  double m[13];                                                                          // m[1]到m[12] 为 最后一行特征向量v的12个元素（xc,yc,zc/f,...）
  for (int i = 1; i < 13; ++i) m[i] = *M1.ptr<double>(i-1);

  double t1 = pow( m[4], 2 );                                                            // t1: xc_2 * xc_2
  double t4 = pow( m[1], 2 );                                                            // t4: xc_1 * xc_1
  double t5 = pow( m[5], 2 );                                                            // t5: yc_2 * yc_2
  double t8 = pow( m[2], 2 );                                                            // t8: yc_1 * yc_1
  double t10 = pow( m[6], 2 );                                                           // t10: zc_2/f * zc_2/f
  double t13 = pow( m[3], 2 );                                                           // t13: zc_1/f * zc_1/f
  double t15 = pow( m[7], 2 );                                                           // t15: xc_3 * xc_3
  double t18 = pow( m[8], 2 );                                                           // t18: yc_3 * yc_3
  double t22 = pow( m[9], 2 );                                                           // t22: zc_3/f * zc_3/f
  double t26 = pow( m[10], 2 );                                                          // t26: xc_4 * xc_4
  double t29 = pow( m[11], 2 );                                                          // t29: yc_4 * yc_4
  double t33 = pow( m[12], 2 );                                                          // t33: zc_4/f * zc_4/f

  *P.ptr<double>(0,0) = t1 - 2 * m[4] * m[1] + t4 + t5 - 2 * m[5] * m[2] + t8;           // | (xc_2 - xc_1)^2 + (yc_2 - yc_1)^2  (zc_2/f - zc_1/f)^2 |
  *P.ptr<double>(0,1) = t10 - 2 * m[6] * m[3] + t13;                                     // |                                                        |
  *P.ptr<double>(1,0) = t15 - 2 * m[7] * m[1] + t4 + t18 - 2 * m[8] * m[2] + t8;         // | (xc_3 - xc_1)^2 + (yc_3 - yc_1)^2  (zc_3/f - zc_1/f)^2 |
  *P.ptr<double>(1,1) = t22 - 2 * m[9] * m[3] + t13;                                     // |                                                        |
  *P.ptr<double>(2,0) = t26 - 2 * m[10] * m[1] + t4 + t29 - 2 * m[11] * m[2] + t8;       // | (xc_4 - xc_1)^2 + (yc_4 - yc_1)^2  (zc_4/f - zc_1/f)^2 |
  *P.ptr<double>(2,1) = t33 - 2 * m[12] * m[3] + t13;                                    // |                                                        |
  *P.ptr<double>(3,0) = t15 - 2 * m[7] * m[4] + t1 + t18 - 2 * m[8] * m[5] + t5;         // | (xc_3 - xc_2)^2 + (yc_3 - yc_2)^2  (zc_3/f - zc_2/f)^2 |
  *P.ptr<double>(3,1) = t22 - 2 * m[9] * m[6] + t10;                                     // |                                                        |
  *P.ptr<double>(4,0) = t26 - 2 * m[10] * m[4] + t1 + t29 - 2 * m[11] * m[5] + t5;       // | (xc_4 - xc_2)^2 + (yc_4 - yc_2)^2  (zc_4/f - zc_2/f)^2 |
  *P.ptr<double>(4,1) = t33 - 2 * m[12] * m[6] + t10;                                    // |                                                        | 
  *P.ptr<double>(5,0) = t26 - 2 * m[10] * m[7] + t15 + t29 - 2 * m[11] * m[8] + t18;     // | (xc_4 - xc_3)^2 + (yc_4 - yc_3)^2  (zc_4/f - zc_3/f)^2 |
  *P.ptr<double>(5,1) = t33 - 2 * m[12] * m[9] + t22;
                                                                                         //                         L 6*2
  return P;
}

Mat upnp::compute_constraint_distance_3param_6eq_6unk_f_unk(const Mat& M1, const Mat& M2)
{
  Mat P = Mat(6, 6, CV_64F);

  double m[3][13];
  for (int i = 1; i < 13; ++i)
  {
    m[1][i] = *M1.ptr<double>(i-1);                   // M1 为倒数第2行
    m[2][i] = *M2.ptr<double>(i-1);                   // M2 为倒数第1行
  }

  double t1 = pow( m[1][4], 2 );                      // 为计算6*6的L做准备（各种缩写）
  double t2 = pow( m[1][1], 2 );                      
  double t7 = pow( m[1][5], 2 );                      //             i 和  j （4个控制点，c42=6种组合，6行）
  double t8 = pow( m[1][2], 2 );                      //             ^     ^
  double t11 = m[1][1] * m[2][1];                     //   l1: (v1x_i-v1x_j)^2+(v1y_i-v1y_j)^2                                 第一列，对应beta11
  double t12 = m[1][5] * m[2][5];                     //
  double t15 = m[1][2] * m[2][2];                     //   l2: 2*(v1x_i*v2x_i + v1x_j*v2x_j - v1x_i*v2x_j - v1x_j*v2x_i
  double t16 = m[1][4] * m[2][4];                     //          + v1y_i*v2y_i + v1y_j*v2y_j - v1y_i*v2y_j - v1y_j*v2y_i)     第二列，对应beta12
  double t19 = pow( m[2][4], 2 );                     //   
  double t22 = pow( m[2][2], 2 );                     //   l3: (v2x_i-v2x_j)^2+(v2y_i-v2y_j)^2                                 第三列，对应beta22
  double t23 = pow( m[2][1], 2 );                     //
  double t24 = pow( m[2][5], 2 );                     //   l4: (v1z_i-v1z_j)^2                                                 第四列，对应ffbeta11
  double t28 = pow( m[1][6], 2 );                     //
  double t29 = pow( m[1][3], 2 );                     //   l5: 2*(v1z_i*v2z_i + v1z_j*v2z_j - v1z_i*v2z_j - v1z_j*v2z_i)       第五列，对应ffbeta12
  double t34 = pow( m[1][3], 2 );                     //
  double t36 = m[1][6] * m[2][6];                     //   l6: (v2z_i-v2z_j)^2                                                 第六列，对应ffbeta22
  double t40 = pow( m[2][6], 2 );                    
  double t41 = pow( m[2][3], 2 );                     
  double t47 = pow( m[1][7], 2 );                     
  double t48 = pow( m[1][8], 2 );                     
  double t52 = m[1][7] * m[2][7];
  double t55 = m[1][8] * m[2][8];
  double t59 = pow( m[2][8], 2 );
  double t62 = pow( m[2][7], 2 );
  double t64 = pow( m[1][9], 2 );
  double t68 = m[1][9] * m[2][9];
  double t74 = pow( m[2][9], 2 );
  double t78 = pow( m[1][10], 2 );
  double t79 = pow( m[1][11], 2 );
  double t84 = m[1][10] * m[2][10];
  double t87 = m[1][11] * m[2][11];
  double t90 = pow( m[2][10], 2 );
  double t95 = pow( m[2][11], 2 );
  double t99 = pow( m[1][12], 2 );
  double t101 = m[1][12] * m[2][12];
  double t105 = pow( m[2][12], 2 );

  *P.ptr<double>(0,0) = t1 + t2 - 2 * m[1][4] * m[1][1] - 2 * m[1][5] * m[1][2] + t7 + t8;
  *P.ptr<double>(0,1) = -2 * m[2][4] * m[1][1] + 2 * t11 + 2 * t12 - 2 * m[1][4] * m[2][1] - 2 * m[2][5] * m[1][2] + 2 * t15 + 2 * t16 - 2 * m[1][5] * m[2][2];
  *P.ptr<double>(0,2) = t19 - 2 * m[2][4] * m[2][1] + t22 + t23 + t24 - 2 * m[2][5] * m[2][2];
  *P.ptr<double>(0,3) = t28 + t29 - 2 * m[1][6] * m[1][3];
  *P.ptr<double>(0,4) = -2 * m[2][6] * m[1][3] + 2 * t34 - 2 * m[1][6] * m[2][3] + 2 * t36;
  *P.ptr<double>(0,5) = -2 * m[2][6] * m[2][3] + t40 + t41;

  *P.ptr<double>(1,0) = t8 - 2 * m[1][8] * m[1][2] - 2 * m[1][7] * m[1][1] + t47 + t48 + t2;
  *P.ptr<double>(1,1) = 2 * t15 - 2 * m[1][8] * m[2][2] - 2 * m[2][8] * m[1][2] + 2 * t52 - 2 * m[1][7] * m[2][1] - 2 * m[2][7] * m[1][1] + 2 * t55 + 2 * t11;
  *P.ptr<double>(1,2) = -2 * m[2][8] * m[2][2] + t22 + t23 + t59 - 2 * m[2][7] * m[2][1] + t62;
  *P.ptr<double>(1,3) = t29 + t64 - 2 * m[1][9] * m[1][3];
  *P.ptr<double>(1,4) = 2 * t34 + 2 * t68 - 2 * m[2][9] * m[1][3] - 2 * m[1][9] * m[2][3];
  *P.ptr<double>(1,5) = -2 * m[2][9] * m[2][3] + t74 + t41;

  *P.ptr<double>(2,0) = -2 * m[1][11] * m[1][2] + t2 + t8 + t78 + t79 - 2 * m[1][10] * m[1][1];
  *P.ptr<double>(2,1) = 2 * t15 - 2 * m[1][11] * m[2][2] + 2 * t84 - 2 * m[1][10] * m[2][1] - 2 * m[2][10] * m[1][1] + 2 * t87 - 2 * m[2][11] * m[1][2]+ 2 * t11;
  *P.ptr<double>(2,2) = t90 + t22 - 2 * m[2][10] * m[2][1] + t23 - 2 * m[2][11] * m[2][2] + t95;
  *P.ptr<double>(2,3) = -2 * m[1][12] * m[1][3] + t99 + t29;
  *P.ptr<double>(2,4) = 2 * t34 + 2 * t101 - 2 * m[2][12] * m[1][3] - 2 * m[1][12] * m[2][3];
  *P.ptr<double>(2,5) = t41 + t105 - 2 * m[2][12] * m[2][3];

  *P.ptr<double>(3,0) = t48 + t1 - 2 * m[1][8] * m[1][5] + t7 - 2 * m[1][7] * m[1][4] + t47;
  *P.ptr<double>(3,1) = 2 * t16 - 2 * m[1][7] * m[2][4] + 2 * t55 + 2 * t52 - 2 * m[1][8] * m[2][5] - 2 * m[2][8] * m[1][5] - 2 * m[2][7] * m[1][4] + 2 * t12;
  *P.ptr<double>(3,2) = t24 - 2 * m[2][8] * m[2][5] + t19 - 2 * m[2][7] * m[2][4] + t62 + t59;
  *P.ptr<double>(3,3) = -2 * m[1][9] * m[1][6] + t64 + t28;
  *P.ptr<double>(3,4) = 2 * t68 + 2 * t36 - 2 * m[2][9] * m[1][6] - 2 * m[1][9] * m[2][6];
  *P.ptr<double>(3,5) = t40 + t74 - 2 * m[2][9] * m[2][6];

  *P.ptr<double>(4,0) = t1 - 2 * m[1][10] * m[1][4] + t7 + t78 + t79 - 2 * m[1][11] * m[1][5];
  *P.ptr<double>(4,1) = 2 * t84 - 2 * m[1][11] * m[2][5] - 2 * m[1][10] * m[2][4] + 2 * t16 - 2 * m[2][11] * m[1][5] + 2 * t87 - 2 * m[2][10] * m[1][4] + 2 * t12;
  *P.ptr<double>(4,2) = t19 + t24 - 2 * m[2][10] * m[2][4] - 2 * m[2][11] * m[2][5] + t95 + t90;
  *P.ptr<double>(4,3) = t28 - 2 * m[1][12] * m[1][6] + t99;
  *P.ptr<double>(4,4) = 2 * t101 + 2 * t36 - 2 * m[2][12] * m[1][6] - 2 * m[1][12] * m[2][6];
  *P.ptr<double>(4,5) = t105 - 2 * m[2][12] * m[2][6] + t40;

  *P.ptr<double>(5,0) = -2 * m[1][10] * m[1][7] + t47 + t48 + t78 + t79 - 2 * m[1][11] * m[1][8];
  *P.ptr<double>(5,1) = 2 * t84 + 2 * t87 - 2 * m[2][11] * m[1][8] - 2 * m[1][10] * m[2][7] - 2 * m[2][10] * m[1][7] + 2 * t55 + 2 * t52 - 2 * m[1][11] * m[2][8];
  *P.ptr<double>(5,2) = -2 * m[2][10] * m[2][7] - 2 * m[2][11] * m[2][8] + t62 + t59 + t90 + t95;
  *P.ptr<double>(5,3) = t64 - 2 * m[1][12] * m[1][9] + t99;
  *P.ptr<double>(5,4) = 2 * t68 - 2 * m[2][12] * m[1][9] - 2 * m[1][12] * m[2][9] + 2 * t101;
  *P.ptr<double>(5,5) = t105 - 2 * m[2][12] * m[2][9] + t74;

  return P;
}

void upnp::generate_all_possible_solutions_for_f_unk(const double betas[5], double solutions[18][3])             // 巧解 beta1，beta2 和 f
{
  int matrix_to_resolve[18][9] = {
    { 2, 0, 0, 1, 1, 0, 2, 0, 2 }, { 2, 0, 0, 1, 1, 0, 1, 1, 2 },         // 这里的矩阵是和下面的18种组合相对应
    { 2, 0, 0, 1, 1, 0, 0, 2, 2 }, { 2, 0, 0, 0, 2, 0, 2, 0, 2 },         // 例1 ｛2，0，0，     b11      两个b1相乘，所以第一个元素为2；没有b2和f，所以第二三个元素为0
    { 2, 0, 0, 0, 2, 0, 1, 1, 2 }, { 2, 0, 0, 0, 2, 0, 0, 2, 2 },         //       1，1，0，==>  b12      一个b1，一个b2
    { 2, 0, 0, 2, 0, 2, 1, 1, 2 }, { 2, 0, 0, 2, 0, 2, 0, 2, 2 },         //       2，0，2}      ffb11    两个b1，两个f
    { 2, 0, 0, 1, 1, 2, 0, 2, 2 }, { 1, 1, 0, 0, 2, 0, 2, 0, 2 },
    { 1, 1, 0, 0, 2, 0, 1, 1, 2 }, { 1, 1, 0, 2, 0, 2, 0, 2, 2 },         // 例2 ｛0，2，0，     b22      两个b2相乘，所以第二个元素为2；没有b1和f，所以第一三个元素为0
    { 1, 1, 0, 2, 0, 2, 1, 1, 2 }, { 1, 1, 0, 2, 0, 2, 0, 2, 2 },         //       1，1，2，==>  ffb12    一个b1，一个b2，两个f
    { 1, 1, 0, 1, 1, 2, 0, 2, 2 }, { 0, 2, 0, 2, 0, 2, 1, 1, 2 },         //       0，2，2}      ffb22    两个b2，两个f
    { 0, 2, 0, 2, 0, 2, 0, 2, 2 }, { 0, 2, 0, 1, 1, 2, 0, 2, 2 }
  };

  int combination[18][3] = {                                              // C63 - b11 b12 b22 - ffb11 ffb12 ffb22 = 18种组合
    { 1, 2, 4 }, { 1, 2, 5 }, { 1, 2, 6 }, { 1, 3, 4 },
    { 1, 3, 5 }, { 1, 3, 6 }, { 1, 4, 5 }, { 1, 4, 6 },
    { 1, 5, 6 }, { 2, 3, 4 }, { 2, 3, 5 }, { 2, 3, 6 },
    { 2, 4, 5 }, { 2, 4, 6 }, { 2, 5, 6 }, { 3, 4, 5 },
    { 3, 4, 6 }, { 3, 5, 6 }
  };

  for (int i = 0; i < 18; ++i) {                                                //例1     | 2 0 0 |         | 0.5  0   0 |      
    double matrix[9], independent_term[3];                                      //    M = | 1 1 0 |, M^-1 = | -0.5 1   0 |   I = [ logb11 logb12 logffb11 ]^t
    Mat M = Mat(3, 3, CV_64F, matrix);                                          //        | 2 0 2 |         | -0.5 0 0.5 |
    Mat I = Mat(3, 1, CV_64F, independent_term);                                //       
    Mat S = Mat(1, 3, CV_64F);                                                  //        | exp( 0.5*logb11 )                 |   | exp(logb1) |   | b1 |
                                                                                //    S = | exp( -0.5*logb11 + logb12 )       | = | exp(logb2) | = | b2 |
    for (int j = 0; j < 9; ++j) matrix[j] = (double)matrix_to_resolve[i][j];    //        | exp( -0.5*logb11 + 0.5*logffb11 ) |   | exp(logf)  |   | f  |

    independent_term[0] = log( abs( betas[ combination[i][0]-1 ] ) );           //例2     | 0 2 0 |         | 0.5  1  -1 |      
    independent_term[1] = log( abs( betas[ combination[i][1]-1 ] ) );           //    M = | 1 1 2 |, M^-1 = | 0.5  0   0 |   I = [ logb22 logffb12 logffb22 ]^t
    independent_term[2] = log( abs( betas[ combination[i][2]-1 ] ) );           //        | 0 2 2 |         | -0.5 0 0.5 |
                                                                                  //
    exp( Mat(M.inv() * I), S);                                                  //        | exp( 0.5*logb22 + logffb12 - logffb22 )  |   | exp(logb1) |   | b1 |
                                                                                //    S = | exp( 0.5*logb22 )                        | = | exp(logb2) | = | b2 |
    solutions[i][0] = S.at<double>(0);                                          //        | exp( -0.5*logb22 + 0.5*logffb22 )        |   | exp(logf)  |   | f  |           
    solutions[i][1] = S.at<double>(1) * sign( betas[1] );   
    solutions[i][2] = abs( S.at<double>(2) );
  }
}

void upnp::gauss_newton(const Mat * L_6x12, const Mat * Rho, double betas[4], double * f)
{
  const int iterations_number = 50;

  double a[6*4], b[6], x[4];
  Mat * A = new Mat(6, 4, CV_64F, a);
  Mat * B = new Mat(6, 1, CV_64F, b);
  Mat * X = new Mat(4, 1, CV_64F, x);

  for(int k = 0; k < iterations_number; k++)                       // 高斯牛顿迭代，提纯beta1,beta2,beta3和f
  {
    compute_A_and_b_gauss_newton(L_6x12->ptr<double>(0), Rho->ptr<double>(0), betas, A, B, f[0]);     // 最小化||L_i * betas - rho_i||，i=0,...5.
    qr_solve(A, B, X);
    for(int i = 0; i < 3; i++)
      betas[i] += x[i];                                                                               // 调整 betas（根据计算出的 使得误差最小 的delta）
      f[0] += x[3];                                                                                   // 调整   f  （根据计算出的 使得误差最小 的delta）
  }

  if (f[0] < 0) f[0] = -f[0];
  fu = fv = f[0];

  A->release();
  delete A;

  B->release();
  delete B;

  X->release();
  delete X;

}

void upnp::compute_A_and_b_gauss_newton(const double * l_6x12, const double * rho,          // A * delta_x = b
        const double betas[4], Mat * A, Mat * b, double const f)
{

  for(int i = 0; i < 6; i++) {                         // x = [beta11 beta12 beta13 beta22 beta23 beta33 ffbeta11 ffbeta12 ffbeta13 ffbeta22 ffbeta23 ffbeta33]^t
    const double * rowL = l_6x12 + i * 12;
    double * rowA = A->ptr<double>(i);

    rowA[0] = 2 * rowL[0] * betas[0] +    rowL[1] * betas[1] +    rowL[2] * betas[2] + f*f * ( 2 * rowL[6]*betas[0] +    rowL[7]*betas[1]  +    rowL[8]*betas[2] );         // 6个含B1的
    rowA[1] =    rowL[1] * betas[0] + 2 * rowL[3] * betas[1] +    rowL[4] * betas[2] + f*f * (    rowL[7]*betas[0] + 2 * rowL[9]*betas[1]  +    rowL[10]*betas[2] );        // 6个含B2的
    rowA[2] =    rowL[2] * betas[0] +    rowL[4] * betas[1] + 2 * rowL[5] * betas[2] + f*f * (    rowL[8]*betas[0] +    rowL[10]*betas[1] + 2 * rowL[11]*betas[2] );        // 6个含B3的
    rowA[3] = 2*f * ( rowL[6]*betas[0]*betas[0] + rowL[7]*betas[0]*betas[1] + rowL[8]*betas[0]*betas[2] + rowL[9]*betas[1]*betas[1] + rowL[10]*betas[1]*betas[2] + rowL[11]*betas[2]*betas[2] ) ;    // 6个含f的

    *b->ptr<double>(i) = rho[i] -                                                          // b 为 L*betas-rho
    (
      rowL[0] * betas[0] * betas[0] +
      rowL[1] * betas[0] * betas[1] +
      rowL[2] * betas[0] * betas[2] +
      rowL[3] * betas[1] * betas[1] +
      rowL[4] * betas[1] * betas[2] +
      rowL[5] * betas[2] * betas[2] +
      f*f * rowL[6] * betas[0] * betas[0] +
      f*f * rowL[7] * betas[0] * betas[1] +
      f*f * rowL[8] * betas[0] * betas[2] +
      f*f * rowL[9] * betas[1] * betas[1] +
      f*f * rowL[10] * betas[1] * betas[2] +
      f*f * rowL[11] * betas[2] * betas[2]
     );
  }
}

void upnp::compute_L_6x12(const double * ut, double * l_6x12)             // N = 3时，betas1，2，3对应三个特征向量
{
  const double * v[3];

  v[0] = ut + 12 * 9;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 * 11;

  double dv[3][6][3];

  for(int i = 0; i < 3; i++) {
    int a = 0, b = 1;
    for(int j = 0; j < 6; j++) {
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];                        // 表示不同控制点x坐标之间的差值
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];                    // 表示不同控制点y坐标之间的差值
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];                    // 表示不同控制点z坐标之间的差值

      b++;
      if (b > 3) {                                                        // 控制着不同控制点，同EPnP代码风格相同
        a++;
        b = a + 1;
      }
    }
  }

  for(int i = 0; i < 6; i++) {                                            // L (6*12)                        *    bates (12*1)   = rho (6*1)
    double * row = l_6x12 + 12 * i;                                       //   
                                                                          // |     对beta1v1 + beta2v2    |       |  beta11   |    | dcw0_1 |
    row[0] =         dotXY(dv[0][i], dv[0][i]);                           // |     + beta3v3展开推导即可   |       |  beta12   |    | dcw0_2 |
    row[1] =  2.0f * dotXY(dv[0][i], dv[1][i]);                           // |     同EPnP相似, 不同之处:   |  *    |  beta13   |  = | dcw0_3 |
    row[2] =         dotXY(dv[1][i], dv[1][i]);                           // |     这里的前6列是对应x,y的   |       |  beta22   |    | dcw1_2 |
    row[3] =  2.0f * dotXY(dv[0][i], dv[2][i]);                           // |     后6列对应z（其实是z/f）  |       |  beta23   |    | dcw1_3 |
    row[4] =  2.0f * dotXY(dv[1][i], dv[2][i]);                           // |                            |       |  beta33   |    | dcw2_3 |
    row[5] =         dotXY(dv[2][i], dv[2][i]);                           //                                      |  ffbeta11 |
                                                                          //                                      |  ffbeta12 |                                      |  beta33  |
    row[6] =         dotZ(dv[0][i], dv[0][i]);                            //                                      |  ffbeta13 |
    row[7] =  2.0f * dotZ(dv[0][i], dv[1][i]);                            //                                      |  ffbeta22 |
    row[8] =  2.0f * dotZ(dv[0][i], dv[2][i]);                            //                                      |  ffbeta23 |
    row[9] =         dotZ(dv[1][i], dv[1][i]);                            //                                      |  ffbeta33 |                                      |  beta33  |
    row[10] = 2.0f * dotZ(dv[1][i], dv[2][i]);                            //                                     
    row[11] =        dotZ(dv[2][i], dv[2][i]);                            //                                     
  }
}

void upnp::compute_rho(double * rho)
{
  rho[0] = dist2(cws[0], cws[1]);
  rho[1] = dist2(cws[0], cws[2]);
  rho[2] = dist2(cws[0], cws[3]);
  rho[3] = dist2(cws[1], cws[2]);
  rho[4] = dist2(cws[1], cws[3]);
  rho[5] = dist2(cws[2], cws[3]);
}

double upnp::dist2(const double * p1, const double * p2)
{
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double upnp::dot(const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double upnp::dotXY(const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1];
}

double upnp::dotZ(const double * v1, const double * v2)
{
  return v1[2] * v2[2];
}

double upnp::sign(const double v)
{
  return ( v < 0.0 ) ? -1.0 : ( v > 0.0 ) ? 1.0 : 0.0;
}

void upnp::qr_solve(Mat * A, Mat * b, Mat * X)
{
  const int nr = A->rows;
  const int nc = A->cols;

  if (max_nr != 0 && max_nr < nr)
  {
    delete [] A1;
    delete [] A2;
  }
  if (max_nr < nr)
  {
    max_nr = nr;
    A1 = new double[nr];
    A2 = new double[nr];
  }

  double * pA = A->ptr<double>(0), * ppAkk = pA;
  for(int k = 0; k < nc; k++)
  {
    double * ppAik1 = ppAkk, eta = fabs(*ppAik1);
    for(int i = k + 1; i < nr; i++)
    {
      double elt = fabs(*ppAik1);
      if (eta < elt) eta = elt;
      ppAik1 += nc;
    }
    if (eta == 0)                                                                           
    {
      A1[k] = A2[k] = 0.0;
      //cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    }
    else
    {
     double * ppAik2 = ppAkk, sum2 = 0.0, inv_eta = 1. / eta;
     for(int i = k; i < nr; i++)
     {
       *ppAik2 *= inv_eta;
       sum2 += *ppAik2 * *ppAik2;
       ppAik2 += nc;
     }
     double sigma = sqrt(sum2);
     if (*ppAkk < 0)
     sigma = -sigma;
     *ppAkk += sigma;
     A1[k] = sigma * *ppAkk;
     A2[k] = -eta * sigma;
     for(int j = k + 1; j < nc; j++)
     {
       double * ppAik = ppAkk, sum = 0;
       for(int i = k; i < nr; i++)
       {
        sum += *ppAik * ppAik[j - k];
        ppAik += nc;
       }
       double tau = sum / A1[k];
       ppAik = ppAkk;
       for(int i = k; i < nr; i++)
       {
        ppAik[j - k] -= tau * *ppAik;
        ppAik += nc;
       }
     }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b
  double * ppAjj = pA, * pb = b->ptr<double>(0);
  for(int j = 0; j < nc; j++)
  {
    double * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)
    {
     tau += *ppAij * pb[i];
     ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++)
    {
     pb[i] -= tau * *ppAij;
     ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  double * pX = X->ptr<double>(0);
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--)
  {
    double * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++)
    {
     sum += *ppAij * pX[j];
     ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }
}
