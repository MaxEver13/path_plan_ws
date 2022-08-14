#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <iostream>
using namespace std;


// 利用最小二乘法求控制点，参考：Least-Squares Fitting of Data with B-Spline Curves
// input :
//      sample : 3 x (K+6) for x, y, z sample
//      ts
// output:
//      control_pts (K+6)x3
// ５次样条参数
void getControlPointEqu5(Eigen::MatrixXd samples, double ts, Eigen::MatrixXd& control_pts)
{
    // 采样点数量
    int K = samples.cols() - 6;

    // write A
    // 5次b样条，参考：B-Spline Collocation Method for Solving Singularly Perturbed Boundary Value Problems
    // 式(6)(7)(8)
    Eigen::VectorXd prow(5), vrow(5), arow(5);
    prow << 1, 26, 66, 26, 1; // 位置的系数
    vrow << -1, -10, 0, 10, 1;// 速度的系数（一阶导数）
    arow << 1, 2, -6, 2, 1;   // 加速度的系数（二阶导数）

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 6, K + 6);

    for (int i = 0; i < K + 2; ++i) 
        A.block(i, i, 1, 5) = prow.transpose();
        
    A.block(0, 0, K + 2, K + 6) = (1 / 120.0) * A.block(0, 0, K + 2, K + 6);

    A.block(K + 2, 0, 1, 5) = A.block(K + 3, K + 1, 1, 5) = vrow.transpose();
    A.block(K + 2, 0, 2, K + 6) = (1 / 24.0 / ts) * A.block(K + 2, 0, 2, K + 6);

    A.block(K + 4, 0, 1, 5) = A.block(K + 5, K + 1, 1, 5) = arow.transpose();
    A.block(K + 4, 0, 2, K + 6) = (1 / 6.0 / ts / ts) * A.block(K + 4, 0, 2, K + 6);

    // write b
    Eigen::VectorXd bx(K + 6), by(K + 6), bz(K + 6);
    for (int i = 0; i < K + 6; ++i)
    {
        bx(i) = samples(0, i);
        by(i) = samples(1, i);
        bz(i) = samples(2, i);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    control_pts.resize(K + 6, 3);
    control_pts.col(0) = px;
    control_pts.col(1) = py;
    control_pts.col(2) = pz;
}

// ３次样条求控制点
// input :
//      sample : 3 x (K+6) for x, y, z sample
//      ts
// output:
//      control_pts (K+6)x3

void getControlPointEqu3(Eigen::MatrixXd samples, double ts, Eigen::MatrixXd& control_pts)
{
    // 采样点数量
    int K = samples.cols() - 6;

    // write A
    Eigen::VectorXd prow(3), vrow(3), arow(3);
    prow << 1, 4, 1; // 位置的系数
    vrow << -1, 0, 1;// 速度的系数（一阶导数）
    arow << 1, -2, 1;// 加速度的系数（二阶导数）

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 6, K + 6);

    for (int i = 0; i < K + 2; ++i) 
        A.block(i, i, 1, 3) =  prow.transpose();  

    A.block(0, 0, K + 2, K + 6) = (1 / 6.0) * A.block(0, 0, K + 2, K + 6);      
   
    A.block(K + 2, 0, 1, 3) = A.block(K + 3, K + 1, 1, 3) = vrow.transpose();
    A.block(K + 2, 0, 2, K + 6) = (1 / 2.0 / ts) * A.block(K + 2, 0, 2, K + 6);

    A.block(K + 4, 0, 1, 3) = A.block(K + 5, K + 1, 1, 3) = arow.transpose();
    A.block(K + 4, 0, 2, K + 6) = (1 / ts / ts) * A.block(K + 4, 0, 2, K + 6);

    // write b
    Eigen::VectorXd bx(K + 6), by(K + 6), bz(K + 6);
    for (int i = 0; i < K + 6; ++i)
    {
        bx(i) = samples(0, i);
        by(i) = samples(1, i);
        bz(i) = samples(2, i);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    control_pts.resize(K + 6, 3);
    control_pts.col(0) = px;
    control_pts.col(1) = py;
    control_pts.col(2) = pz;
}

// input: K: segment, N: sample num, ts: segment time, samples 3x(K+1)*(N+1)
// output: control_pts of b-spline: (K+6)x3
void getControlPointLeastSquare(int K, int N, double ts, Eigen::MatrixXd samples, Eigen::MatrixXd& control_pts)
{
    cout << "K:" << K << endl;

    // write matrix block Ab of A
    Eigen::MatrixXd Ab(N + 1, 6);
    Ab << 0.00833333333333333, 0.216666666666667, 0.55, 0.216666666666667, 0.00833333333333333, 0.0, 0.0019775390625,
        0.124910481770833, 0.519645182291667, 0.328076171875, 0.0253824869791667, 0.0, 0.000260416666666667, 0.06171875,
        0.438020833333333, 0.438020833333333, 0.06171875, 0.000260416666666667, 0, 0.0253824869791667, 0.328076171875,
        0.519645182291667, 0.124910481770833, 0.0019775390625, 0.0, 0.00833333333333331, 0.216666666666667, 0.55,
        0.216666666666667, 0.00833333333333333;
    // cout << "Ab:" << Ab << endl;

    // write A
    Eigen::MatrixXd A((N + 1) * (K + 1), K + 6);
    A.setZero();
    for (int i = 0; i <= K; ++i)
    {
        A.block((N + 1) * i, i, N + 1, 6) = Ab;
    }
    // cout << "A" << A << endl;

    cout << "A.cols(): " << A.cols() << ", A.rows():" << A.rows() << endl;
    cout << "samples.cols(): " << samples.cols() << ", samples.rows():" << samples.rows() << endl;

    // solve px, py, pz
    // Eigen::VectorXd px = A.bdcSvd(ComputeThinU | ComputeThinV).solve(bx);
    // Eigen::VectorXd py = A.bdcSvd(ComputeThinU | ComputeThinV).solve(by);
    // Eigen::VectorXd pz = A.bdcSvd(ComputeThinU | ComputeThinV).solve(bz);
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(samples.row(0).transpose());
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(samples.row(1).transpose());
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(samples.row(2).transpose());
    // cout << "px:" << px.transpose() << ", py:" << py.transpose() << ", pz:" << pz.transpose() << endl;

    control_pts.resize(K + 6, 3);
    control_pts.col(0) = px;
    control_pts.col(1) = py;
    control_pts.col(2) = pz;
}

class UniformBspline
{
  private:
    /* data */
    int p, n, m;
    Eigen::MatrixXd control_points;
    std::vector<Eigen::MatrixXd> M;  // 012,345
    Eigen::VectorXd u;
    double interval;

    Eigen::VectorXd getU(double u);
    Eigen::MatrixXd getPi(int idx);

  public:
    UniformBspline(Eigen::MatrixXd points, int order, double interval, bool auto_extend = true);
    ~UniformBspline();

    Eigen::MatrixXd getControlPoint()
    {
        return control_points;
    }

    void getRegion(double& um, double& um_p);

    Eigen::Vector3d evaluate(double u);

    UniformBspline getDerivative();

    Eigen::MatrixXd getDerivativeControlPoints();
};

// control points is a (n+1)x3 matrix
UniformBspline::UniformBspline(Eigen::MatrixXd points, int order, double interval, bool auto_extend)
{
    // b-spline阶数
    this->p = order;
    if (auto_extend)
    {
        control_points = Eigen::MatrixXd::Zero(points.rows() + 2 * this->p, 3);
        for (int i = 0; i < this->p; ++i)
        {
            control_points.row(i) = points.row(0);
            control_points.row(control_points.rows() - 1 - i) = points.row(points.rows() - 1);
        }
        control_points.block(this->p, 0, points.rows(), 3) = points;
        this->n = points.rows() + 2 * this->p - 1;
    }
    else
    {
        control_points = points;
        // 这里的n表示控制点的最大序列号n-1，n个控制点（P0,P1,...,Pn-1）
        this->n = points.rows() - 1; 
    }
    // 节点向量（U0, U1,..., Un+p) 这里的n表示控制点个数，p表示b-spline阶数
    // 这里m表示节点向量的最大序列号
    this->m = this->n + 1 + this->p;

    // calculate knots vector
    this->interval = interval;
    // 节点向量个数
    this->u = Eigen::VectorXd::Zero(this->m + 1);
    for (int i = 0; i <= this->m; ++i)
    {
        // 前p+1个节点
        if (i <= this->p) {
            // this->u(i) = double(-this->p + i) * this->interval;
            this->u(i) = 0.0;
        }
        else if (i > this->p && i <= this->m - this->p)
        {
            this->u(i) = this->u(i - 1) + this->interval;
        }
        else if (i > this->m - this->p)
        {
            // this->u(i) = this->u(i - 1) + this->interval;
            this->u(i) = 1.0;
        }
    }

    // initialize the M3-6 matrix
    this->M.resize(4);
    Eigen::MatrixXd M3 = Eigen::MatrixXd::Zero(3, 3); // 2次B样条参数
    Eigen::MatrixXd M4 = Eigen::MatrixXd::Zero(4, 4); // 3次B样条参数
    Eigen::MatrixXd M5 = Eigen::MatrixXd::Zero(5, 5); // 4次B样条参数
    Eigen::MatrixXd M6 = Eigen::MatrixXd::Zero(6, 6); // 5次B样条参数

    M3 << 1.0, 1.0, 0.0, -2.0, 2.0, 0.0, 1.0, -2.0, 1.0;
    M4 << 1.0, 4.0, 1.0, 0.0, -3.0, 0.0, 3.0, 0.0, 3.0, -6.0, 3.0, 0.0, -1.0, 3.0, -3.0, 1.0;
    M5 << 1.0, 11.0, 11.0, 1.0, 0, -4.0, -12.0, 12.0, 4.0, 0, 6.0, -6.0, -6.0, 6.0, 0, -4.0, 12.0, -12.0, 4.0, 0, 1.0,
        -4.0, 6.0, -4.0, 1.0;
    M6 << 1, 26, 66, 26, 1, 0, -5, -50, 0, 50, 5, 0, 10, 20, -60, 20, 10, 0, -10, 20, 0, -20, 10, 0, 5, -20, 30, -20, 5,
        0, -1, 5, -10, 10, -5, 1;
    M3 /= 2.0;
    M4 /= 3.0 * 2;
    M5 /= 4.0 * 3.0 * 2.0;
    M6 /= 5.0 * 4.0 * 3.0 * 2.0;
    M[0] = M3;
    M[1] = M4;
    M[2] = M5;
    M[3] = M6;

    // show the result
    // cout << "p: " << p << "  n: " << n << "  m: " << m << endl;
    // cout << "control pts:\n" << control_points << "\nknots:\n" << this->u.transpose() << endl;
    // cout << "M3:\n" << M[0] << "\nM4:\n" << M[1] << "\nM5:\n" << M[2] << endl;
}

UniformBspline::~UniformBspline()
{
}

void UniformBspline::getRegion(double& um, double& um_p)
{
    um = this->u(this->p);
    um_p = this->u(this->m - this->p);
}

Eigen::VectorXd UniformBspline::getU(double u)
{
    Eigen::VectorXd uv = Eigen::VectorXd::Zero(this->p + 1);
    uv(0) = 1.0;
    for (int i = 1; i <= this->p; ++i)
        uv(i) = uv(i - 1) * u;

    return uv;
}

Eigen::MatrixXd UniformBspline::getPi(int idx)
{
    // 控制点的个数比b-spline的次数多１
    Eigen::MatrixXd pi = control_points.block(idx - p, 0, p + 1, 3);
    return pi;
}

Eigen::Vector3d UniformBspline::evaluate(double u)
{
    // 前p+1个和后p个直接返回0
    // 前p+1个u节点都是负数
    if (u < this->u(this->p) || u > this->u(this->m - this->p)) return Eigen::Vector3d::Zero(3);
    // determine which [ui,ui+1] lay in
    int idx = this->p;
    while (true)
    {
        if (this->u(idx + 1) >= u) break;
        ++idx;
    }
    // 由于这里ｕ增量是0.02 所以肯定在相邻的[ui,ui+1]
    // 计算比例系数
    u = (u - this->u(idx)) / (this->u(idx + 1) - this->u(idx));
    // cout << "alpha: " << u << endl;
    // get u vector and control points: Qi-p -> Qi
    Eigen::VectorXd uv = this->getU(u);
    Eigen::MatrixXd pi = this->getPi(idx);
    // cout << "uv:" << uv.transpose() << "\npi: " << pi.transpose() << endl;

    // use p = u'*M*pi
    // uv表示调整的比例系数
    // M[p - 2]表示对应的基函数(coefficients)
    // [ui,ui+1]的轨迹由控制点(Pi-p, ... ,Pi)共同决定
    Eigen::Vector3d val = (uv.transpose() * M[p - 2] * pi).transpose();
    return val;
}

Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
{
    // 参考：https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
    // The derivative of a b-spline is also a b-spline, its order become p-1
    // control point Qi = p*(Pi+1-Pi)/(ui+p+1-ui+1)
    // 控制点个数减１
    Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points.rows() - 1, 3);
    for (int i = 0; i < ctp.rows(); ++i)
    {
        // std::cout << "delta_t: " << (u(i + p + 1) - u(i + 1)) << ", delta s:" << p * (control_points.row(i + 1) - control_points.row(i)) << std::endl;
        ctp.row(i) = p * (control_points.row(i + 1) - control_points.row(i)) / (u(i + p + 1) - u(i + 1));
    }
    return ctp;
}

UniformBspline UniformBspline::getDerivative()
{
    Eigen::MatrixXd ctp = this->getDerivativeControlPoints();
    // the derivative of a B-spline curve is another B-spline curve of degree p - 1
    UniformBspline derivative = UniformBspline(ctp, p - 1, this->interval, false);
    return derivative;
}

#endif