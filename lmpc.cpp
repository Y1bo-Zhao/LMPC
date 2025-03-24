#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Core>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include "rclcpp/rclcpp.hpp"
#include "lmpc/matplotlibcpp.h"  // 绘图库

namespace plt = matplotlibcpp;
//-------------------------------------------------------------路径规划--------------------------------------------------
// 参考路径生成函数：给定时刻 t 返回参考状态 p
Eigen::Vector3d p(double t) {
    Eigen::Vector3d ref;
    double v = 1.0; // x 方向的速度，例如 1.0 单位/秒
    ref << v * t, 0.0, 0.0; // 依次为 x, y, z
    return ref;
}

// 参考路径导数函数：给定时刻 t 返回 p'(t)
// 对于 p(t) = [v*t, 0, 0]，有 p'(t) = [v, 0, 0]
Eigen::Vector3d p_derivative(double t) {
    Eigen::Vector3d ref_der;
    double v = 1.0; // 保持与 p(t) 中一致
    ref_der << v, 0.0, 0.0;
    return ref_der;
}

//-------------------------------------------------------------系统建模--------------------------------------------------
// 构造系统矩阵的函数
// 参数 dt 为离散时间步长，输出参数 A 和 B 分别为系统状态转移矩阵和输入矩阵
void createSystemMatrices(double dt, Eigen::Matrix<double, 12, 12>& A, Eigen::Matrix<double, 12, 4>& B) {
    const int group_dim = 3;  // 每个坐标对应的状态维度（位置、速度、加速度）

    // 对于常数 jerk 的假设，精确积分的结果为：
    // p_{k+1} = p_k + dt * v_k + 0.5 * dt^2 * a_k + dt^3/6 * j
    // v_{k+1} = v_k + dt * a_k + 0.5 * dt^2 * j
    // a_{k+1} = a_k + dt * j
    // 因此单坐标下的状态转移矩阵 A_block 为：
    Eigen::Matrix3d A_block;
    A_block << 1, dt, 0.5 * dt * dt,
               0,  1,       dt,
               0,  0,        1;
    // 输入矩阵 B_block 为：
    Eigen::Vector3d B_block;
    B_block << dt * dt * dt / 6.0, dt * dt / 2.0, dt;

    // 如果采用简单的 Euler 离散化，只对加速度更新使用 jerk，则 B_block 可以写成 [0, 0, dt]
    // 但这样会忽略 jerk 对速度和位置的累积影响，从而精度较低。
    // 你可以根据精度要求选择使用哪种 B_block，这里我们使用精确积分的版本。

    // 初始化 A 和 B 为零矩阵
    A.setZero();
    B.setZero();

    // 系统状态顺序：x, v_x, a_x, y, v_y, a_y, z, v_z, a_z, t, v_t, a_t
    // 将 A_block 和 B_block 分别放入对应的块中（注意 t 也是采用同样的模型）
    for (int i = 0; i < 4; ++i) {
        A.block<group_dim, group_dim>(i * group_dim, i * group_dim) = A_block;
        B.block<group_dim, 1>(i * group_dim, i) = B_block;
    }
}

//-------------------------------------------------------------生成Q矩阵--------------------------------------------------
Eigen::SparseMatrix<double> createQMatrix(int n, double Q_xt, double Q_yt, double Q_zt, double Q_tt) {
    std::vector<Eigen::Triplet<double>> tripletList;
    // 循环赋值对角线和部分 off-diagonals
    for (int i = 0; i < n; i++) {
        // 对角线元素：
        if (i == 3) continue;  // 跳过 (3,3)
        tripletList.push_back(Eigen::Triplet<double>(i, i, static_cast<double>(1)));
    }
    //x和t
    tripletList.push_back(Eigen::Triplet<double>(0, 3, Q_xt));
    tripletList.push_back(Eigen::Triplet<double>(3, 0, Q_xt));
    //y和t
    tripletList.push_back(Eigen::Triplet<double>(1, 3, Q_yt));
    tripletList.push_back(Eigen::Triplet<double>(3, 1, Q_yt));
    //z和t
    tripletList.push_back(Eigen::Triplet<double>(2, 3, Q_zt));
    tripletList.push_back(Eigen::Triplet<double>(3, 2, Q_zt));
    //t和t，注意不能直接覆盖，这个机制会相加
    tripletList.push_back(Eigen::Triplet<double>(3, 3, Q_tt));

    Eigen::SparseMatrix<double> Q(n, n);
    Q.setFromTriplets(tripletList.begin(), tripletList.end());
    std::cout << "Q 4x4:\n" << Q << "\n";
    return Q;
}

//-------------------------------------------------------------生成q矩阵--------------------------------------------------
Eigen::VectorXd createqMatrix(int n, double last_time){
    Eigen::VectorXd q(n);
    double c_x = p_derivative(last_time)(0)*last_time-p(last_time)(0);
    double c_y = p_derivative(last_time)(1)*last_time-p(last_time)(1);
    double c_z = p_derivative(last_time)(2)*last_time-p(last_time)(2);
    q(0)=2*c_x; 
    q(1)=2*c_y; 
    q(2)=2*c_z; 
    q(3)=((-2)*p_derivative(last_time)(0)*c_x+(-2)*p_derivative(last_time)(1)*c_y+(-2)*p_derivative(last_time)(2)*c_z);
    q(4)=-1;//激进系数

    return q;
}

//-------------------------------------------------------------生成A矩阵--------------------------------------------------
Eigen::SparseMatrix<double> createAMatrix(Eigen::Matrix<double, 12, 4> B){
    Eigen::SparseMatrix<double> A_total(22, 16);
    std::vector<Eigen::Triplet<double>> tripletList;

    // 状态约束：对 v_x, a_x, v_y, a_y, v_z, a_z 进行限制
    tripletList.push_back(Eigen::Triplet<double>(0, 1, 1.0)); // 第 0 行对应 v_x
    tripletList.push_back(Eigen::Triplet<double>(1, 2, 1.0)); // 第 1 行对应 a_x
    tripletList.push_back(Eigen::Triplet<double>(2, 4, 1.0)); // 第 2 行对应 v_y
    tripletList.push_back(Eigen::Triplet<double>(3, 5, 1.0)); // 第 3 行对应 a_y
    tripletList.push_back(Eigen::Triplet<double>(4, 7, 1.0)); // 第 4 行对应 v_z
    tripletList.push_back(Eigen::Triplet<double>(5, 8, 1.0)); // 第 5 行对应 a_z

    // 输入约束：对系统输入设置限制，假设输入位于索引 12,13,14,15
    tripletList.push_back(Eigen::Triplet<double>(6, 12, 1.0)); // 第 6 行对应输入1
    tripletList.push_back(Eigen::Triplet<double>(7, 13, 1.0)); // 第 7 行对应输入2
    tripletList.push_back(Eigen::Triplet<double>(8, 14, 1.0)); // 第 8 行对应输入3
    tripletList.push_back(Eigen::Triplet<double>(9, 15, 1.0)); // 第 9 行对应输入4

    A_total.setFromTriplets(tripletList.begin(), tripletList.end());

    Eigen::Matrix<double, 12, 16> A_eq;
    A_eq.setZero();
    // 左边块赋值为 12×12 的单位矩阵
    A_eq.block<12,12>(0, 0) = Eigen::Matrix<double, 12, 12>::Identity();
    // 右边块赋值为 -B
    A_eq.block<12,4>(0, 12) = -B;

    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 16; ++j) {
            double val = A_eq(i, j);
            if (val != 0.0) {
                // 新的行索引从原来的不等式约束行数开始（例如 10）
                tripletList.push_back(Eigen::Triplet<double>(10 + i, j, val));
            }
        }
    }
    A_total.setFromTriplets(tripletList.begin(), tripletList.end());

    return A_total;
}

//-------------------------------------------------------------生成等式约束下界--------------------------------------------------
Eigen::VectorXd createlow(Eigen::Matrix<double, 12, 12> A, Eigen::Matrix<double, 12, 1> x_last = Eigen::Matrix<double, 12, 1>::Zero()){
    // 假设状态约束数值
    Eigen::VectorXd lowerBound(22);
    double v_x_min = -2.0;
    double a_x_min = -1.0;
    double v_y_min = -2.0;
    double a_y_min = -1.0;
    double v_z_min = -2.0;
    double a_z_min = -1.0;

    // 状态约束上下界
    lowerBound(0) = v_x_min;
    lowerBound(1) = a_x_min;
    lowerBound(2) = v_y_min;
    lowerBound(3) = a_y_min;
    lowerBound(4) = v_z_min;
    lowerBound(5) = a_z_min;

    // 假设输入约束数值
    double u_min = -0.5;

    // 输入约束上下界（4 个输入）
    lowerBound(6) = u_min;
    lowerBound(7) = u_min;
    lowerBound(8) = u_min;
    lowerBound(9) = u_min; 

    lowerBound.segment(10, 12) = A * x_last;

    std::cout << "下界:\n" << lowerBound << "\n";
    return lowerBound;
}

//-------------------------------------------------------------生成等式约束上界--------------------------------------------------
Eigen::VectorXd createup(Eigen::Matrix<double, 12, 12> A, Eigen::Matrix<double, 12, 1> x_last){
    // 假设状态约束数值
    Eigen::VectorXd upperBound(22);
    double v_x_max = 3.0;
    double a_x_max = 1.0;
    double v_y_max = 2.0;
    double a_y_max = 1.0;
    double v_z_max = 2.0;
    double a_z_max = 1.0;

    // 状态约束上界
    upperBound(0) = v_x_max;
    upperBound(1) = a_x_max;
    upperBound(2) = v_y_max;
    upperBound(3) = a_y_max;
    upperBound(4) = v_z_max;
    upperBound(5) = a_z_max;

    // 假设输入约束数值
    double u_max = 0.5;

    // 输入约束上界（4 个输入）
    upperBound(6) = u_max;
    upperBound(7) = u_max;
    upperBound(8) = u_max;
    upperBound(9) = u_max;

    upperBound.segment(10, 12) = A * x_last;

    std::cout << "上界:\n" << upperBound << "\n";
    return upperBound;
}

int main() {

//-------------------------------------------------------------"全局变量"--------------------------------------------------
    double last_time = 0;
    Eigen::Matrix<double, 12, 1> x_last = Eigen::Matrix<double, 12, 1>::Zero();
    // 记录轨迹数据，用于绘图
    std::vector<double> time_hist;
    std::vector<double> target_x_hist;
    std::vector<double> actual_x_hist;
    double Q_xt = -p_derivative(last_time)(0);
    double Q_yt = -p_derivative(last_time)(1);
    double Q_zt = -p_derivative(last_time)(2);
    double Q_tt = p_derivative(last_time)(0)*p_derivative(last_time)(0)+p_derivative(last_time)(1)*p_derivative(last_time)(1)+p_derivative(last_time)(2)*p_derivative(last_time)(2);
//-------------------------------------------------------------系统建模--------------------------------------------------
    double dt = 1; // 离散时间步长
    Eigen::Matrix<double, 12, 12> A;
    Eigen::Matrix<double, 12, 4> B;

    createSystemMatrices(dt, A, B);

    std::cout << "状态转移矩阵 A:\n" << A << "\n\n";
    std::cout << "输入矩阵 B:\n" << B << "\n";

//-------------------------------------------------------------初始状态--------------------------------------------------
    // 定义状态向量维度为 12
    Eigen::Matrix<double, 12, 1> x0;
    // 初始状态全设为 0
    x0.setZero();

    std::cout << "初始状态 x0:\n" << x0.transpose() << std::endl;

//-------------------------------------------------------------构建目标函数的准备工作--------------------------------------------------

    // 构造 S1 的提取矩阵（4x12），提取 x, y, z, t
    Eigen::Matrix<double, 4, 16> S1 = Eigen::Matrix<double, 4, 16>::Zero();
    S1(0, 0) = 1.0;  // 提取 x（索引 0）
    S1(1, 3) = 1.0;  // 提取 y（索引 3）
    S1(2, 6) = 1.0;  // 提取 z（索引 6）
    S1(3, 9) = 1.0;  // 提取 t（索引 9）

    // 构造 S2 的提取矩阵（5x12），提取 x, y, z, t, v_t
    Eigen::Matrix<double, 5, 16> S2 = Eigen::Matrix<double, 5, 16>::Zero();
    S2(0, 0) = 1.0;   // 提取 x（索引 0）
    S2(1, 3) = 1.0;   // 提取 y（索引 3）
    S2(2, 6) = 1.0;   // 提取 z（索引 6）
    S2(3, 9) = 1.0;   // 提取 t（索引 9）
    S2(4, 10) = 1.0;  // 提取 v_t（索引 10）

    // 构造 S3 的提取矩阵（6x16），提取 v_x, a_x, v_y, a_y, v_z, a_z
    Eigen::Matrix<double, 6, 16> S3 = Eigen::Matrix<double, 6, 16>::Zero();
    S3(0, 1) = 1.0;  // 提取 v_x
    S3(1, 2) = 1.0;  // 提取 a_x
    S3(2, 4) = 1.0;  // 提取 v_y
    S3(3, 5) = 1.0;  // 提取 a_y
    S3(4, 7) = 1.0;  // 提取 v_z
    S3(5, 8) = 1.0;  // 提取 a_z

//-------------------------------------------------------------大循环-------------------------------------------------
    int simulation_steps = 4; // 例如，运行10步
    for (int k = 0; k < simulation_steps; ++k) {


//-------------------------------------------------------------设置OSQP-------------------------------------------------

        // 请根据实际问题设置变量个数 n 和约束个数 m
        int n = 16/* TODO: 设置变量个数 暂时只预测一步*/;
        int m = 22/* TODO: 设置约束个数 暂时只有上下限两个约束*/;

        // 创建 OSQP 求解器对象
        OsqpEigen::Solver solver;

        // 设置求解器参数
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);

//-------------------------------------------------------------设置Q、p-------------------------------------------------

        // -------------------- 下面是 Q（Hessian）和 q 的定义部分 --------------------
        // OSQP 的目标函数形式为： (1/2) x' P x + q' x
        // 在 OsqpEigen 中，P 称为 Hessian 矩阵，需要是 n×n 的稀疏矩阵
        // 此处的占位代码仅给出一个示例：采用 n 维单位矩阵和零向量
        
        Eigen::SparseMatrix<double> hessianMatrix = createQMatrix(4, Q_xt, Q_yt, Q_zt, Q_tt);   // ！这里生成的Q为4x4,后续还需要左右乘以S矩阵
        Eigen::MatrixXd denseHessian = S1.transpose() * Eigen::MatrixXd(hessianMatrix) * S1;    // 将 hessianMatrix 转换为密集矩阵类型进行计算
        hessianMatrix = denseHessian.sparseView();  // 再转换回稀疏矩阵
        std::cout << "海森矩阵 Q:\n" << hessianMatrix << "\n";
        Eigen::VectorXd gradient = createqMatrix(5, last_time); // ！这里生成的q为5x1,后续还需要右乘以S矩阵  
        gradient = (gradient.transpose() * S2).transpose();  
        std::cout << "梯度矩阵 q:\n" << gradient << "\n";       
        // -------------------- END Q 和 q 部分 --------------------

//-------------------------------------------------------------设置约束-------------------------------------------------

        // -------------------- 下面是约束矩阵 A、下界 l 和上界 u 的定义部分 --------------------
        // OSQP 的约束形式为 l <= A x <= u
        // A 需要是 m×n 的稀疏矩阵，这里给出占位示例
        Eigen::SparseMatrix<double> linearMatrix = createAMatrix(B);
        std::cout << "线性矩阵 A:\n" << linearMatrix << "\n";    

        // 设置下界和上界
        Eigen::VectorXd lowerBound = createlow(A, x_last);
        Eigen::VectorXd upperBound = createup(A, x_last);        
        // -------------------- END 约束部分 --------------------

//-------------------------------------------------------------启动！-------------------------------------------------
        // 向求解器中设置数据
        solver.data()->setNumberOfVariables(n);
        solver.data()->setNumberOfConstraints(m);
        if (!solver.data()->setHessianMatrix(hessianMatrix)) {
            std::cerr << "设置 Hessian 矩阵失败" << std::endl;
            return 1;
        }
        if(!solver.data()->setGradient(gradient)){
            std::cerr << "设置梯度向量出错" << std::endl;
            return 1;
        }

        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)){
            std::cerr << "设置约束矩阵出错" << std::endl;
            return 1;
        }

        if(!solver.data()->setLowerBound(lowerBound)){
            std::cerr << "设置下界出错" << std::endl;
            return 1;
        }

        if(!solver.data()->setUpperBound(upperBound)){
            std::cerr << "设置上界出错" << std::endl;
            return 1;
        }

        if(!solver.initSolver()){
            std::cerr << "初始化求解器出错" << std::endl;
            return 1;
        }

        // 求解 QP 问题
        solver.solveProblem();

        // 获取并输出解
        Eigen::VectorXd solution = solver.getSolution();
        std::cout << "求解结果为： " << solution.transpose() << std::endl;

        // 提取状态 X（前 12 个元素）赋值给 x_last
        x_last = solution.segment(0, 12);

        // 提取控制输入 u（后 4 个元素）赋值给 u_last
        Eigen::Matrix<double, 4, 1> u_last = solution.segment(12, 4);

        std::cout << "x_last (状态): " << x_last.transpose() << std::endl;
        std::cout << "u_last (控制输入): " << u_last.transpose() << std::endl;

        // 保存轨迹数据（时间、目标 x 与实际 x）
        time_hist.push_back(last_time);
        target_x_hist.push_back(p(last_time)(0));  // p(t) 返回目标 x 坐标
        actual_x_hist.push_back(x_last(0));          // 实际状态中 x 位于索引 0

        last_time = last_time + dt;
        Q_xt = -p_derivative(last_time)(0);
        Q_yt = -p_derivative(last_time)(1);
        Q_zt = -p_derivative(last_time)(2);
        Q_tt = p_derivative(last_time)(0)*p_derivative(last_time)(0)+p_derivative(last_time)(1)*p_derivative(last_time)(1)+p_derivative(last_time)(2)*p_derivative(last_time)(2);
    }

    plt::figure();
    plt::named_plot("Target", time_hist, target_x_hist, "r-");
    plt::named_plot("real", time_hist, actual_x_hist, "b-");
    plt::xlabel("Time (s)");
    plt::ylabel("X Position");
    plt::title("Target vs real");
    plt::legend();
    plt::show();

    return 0;
}
