//
// Created by DAOLIN SHENG on 2022/4/27.
//

#ifndef META_PROJECT_SELF_DRIVING_CARS_PROJECTS_SOLVEPNPRANSAC_H
#define META_PROJECT_SELF_DRIVING_CARS_PROJECTS_SOLVEPNPRANSAC_H


/*  max 注释
*   函数功能：用ransac的方式求解PnP问题
*
*   参数：
*   [in]    _opoints                参考点在世界坐标系下的点集；float or double
*   [in]    _ipoints                参考点在相机像平面的坐标；float or double
*   [in]    _cameraMatrix           相机内参
*   [in]    _distCoeffs             相机畸变系数
*   [out]   _rvec                   旋转矩阵
*   [out]   _tvec                   平移向量
*   [in]    useExtrinsicGuess       若果求解PnP使用迭代算法，初始值可以使用猜测的初始值（true），也可以使用解析求解的结果作为初始值（false）。
*   [in]    iterationsCount         Ransac算法的迭代次数，这只是初始值，根据估计外点的概率，可以进一步缩小迭代次数；（此值函数内部是会不断改变的）,所以一开始可以赋一个大的值。
*   [in]    reprojectionErrr        Ransac筛选内点和外点的距离阈值，这个根据估计内点的概率和每个点的均方差（假设误差按照高斯分布）可以计算出此阈值。
*   [in]    confidence              此值与计算采样（迭代）次数有关。此值代表从n个样本中取s个点，N次采样可以使s个点全为内点的概率。
*   [out]   _inliers                返回内点的序列。为矩阵形式
*   [in]    flags                   最小子集的计算模型；
*                                                 SOLVEPNP_ITERATIVE(此方案，最小模型用的EPNP，内点选出之后用了一个迭代)；
*                                                 SOLVE_P3P(P3P只用在最小模型上，内点选出之后用了一个EPNP)
*                                                 SOLVE_AP3P(AP3P只用在最小模型上，内点选出之后用了一个EPNP)
*                                                 SOLVE_EPnP(最小模型上&内点选出之后都采用了EPNP)
*    返回值：
*         成功返回true，失败返回false；
*/
bool solvePnPRansac(InputArray _opoints, InputArray _ipoints,
                    InputArray _cameraMatrix, InputArray _distCoeffs,
                    OutputArray _rvec, OutputArray _tvec, bool useExtrinsicGuess,
                    int iterationsCount, float reprojectionError, double confidence,
                    OutputArray _inliers, int flags)

#endif //META_PROJECT_SELF_DRIVING_CARS_PROJECTS_SOLVEPNPRANSAC_H


