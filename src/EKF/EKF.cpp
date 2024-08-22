#include "EKF.h"

// 默认构造函数
EKF_base::EKF_base() : is_initialized(false) {}

// 析构函数
EKF_base::~EKF_base() {}

// 归一化角度到 [-π, π]
inline double EKF_base::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 初始化状态向量和状态协方差矩阵
void EKF_base::initialize(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, const Eigen::MatrixXd &Q0, const Eigen::MatrixXd &R0, const double dt0) {
    x = x0;
    P = P0;
    Q = Q0;
    R = R0;
    dt = dt0;
    is_initialized = true;
}

// 预测步骤
void EKF_base::predict() {
    if (!is_initialized) {
        throw std::runtime_error("EKF is not initialized!");
    }

    // 预测状态向量
    x = f(x,dt);
    x(2) = normalizeAngle(x(2));

    // 计算雅可比矩阵
    Eigen::MatrixXd Fk = F(x,dt);

    // 更新状态协方差矩阵
    P = Fk * P * Fk.transpose() + Q;
}

// 自预测步骤
void EKF_base::self_predict(const int step, std::vector<Eigen::VectorXd>& X_pred, std::vector<Eigen::MatrixXd>& P_pred) {
    if (!is_initialized) {
        throw std::runtime_error("EKF is not initialized!");
    }
    Eigen::VectorXd x_tmp;
    Eigen::MatrixXd P_tmp;  
    x_tmp = x;
    P_tmp = P;
    X_pred.push_back(x_tmp);
    P_pred.push_back(P_tmp);
    for(int k=0;k<step;++k){
        x_tmp = f(x_tmp,dt);
        x_tmp(2) = normalizeAngle(x_tmp(2)); // 归一化朝向角
        Eigen::MatrixXd Fk = F(x_tmp,dt);
        P_tmp = Fk * P_tmp * Fk.transpose() + Q;
        X_pred.push_back(x_tmp);
        P_pred.push_back(P_tmp);
    }
    
}

// 更新步骤
void EKF_base::update(const Eigen::VectorXd &z) {
    if (!is_initialized) {
        throw std::runtime_error("EKF is not initialized!");
    }

    // 计算测量预测值
    Eigen::VectorXd z_pred = h(x);

    // 计算测量预测雅可比矩阵
    Eigen::MatrixXd Hk = H(x);

    // 计算卡尔曼增益
    Eigen::MatrixXd S = Hk * P * Hk.transpose() + R;
    Eigen::MatrixXd K = P * Hk.transpose() * S.inverse();

    // 更新状态向量
    x = x + K * (z - z_pred);

    x(2) = normalizeAngle(x(2));

    // 更新状态协方差矩阵
    int size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
    P = (I - K * Hk) * P;
}

// 获取当前状态向量
Eigen::VectorXd EKF_base::getState() const {
    return x;
}

// 获取当前状态协方差矩阵
Eigen::MatrixXd EKF_base::getCovariance() const {
    return P;
}


// CV model
EKF_CV::EKF_CV() {
    // 构造函数实现
}

EKF_CV::~EKF_CV() {
    // 析构函数实现
}

Eigen::VectorXd EKF_CV::f(const Eigen::VectorXd &x, double dt) {
    // State vector: [x, y, theta, v, w, a]
    Eigen::VectorXd x_pred = x;
    double theta = x(2);
    double v = x(3);
    double w = x(4);

    // Update the state prediction based on the motion model
    x_pred(0) += v * cos(theta) * dt;
    x_pred(1) += v * sin(theta) * dt;
    x_pred(2) += w * dt;
    x_pred(3) = v; // Assuming constant velocity model
    x_pred(4) = w; // Assuming constant angular velocity model
    x_pred(5) = x(5); // Assuming constant acceleration model

    return x_pred;
}

Eigen::MatrixXd EKF_CV::F(const Eigen::VectorXd &x, double dt) {
    // State transition Jacobian matrix
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(x.size(), x.size());
    double theta = x(2);
    double v = x(3);

    F(0, 2) = -v * sin(theta) * dt;
    F(0, 3) = cos(theta) * dt;
    F(1, 2) = v * cos(theta) * dt;
    F(1, 3) = sin(theta) * dt;
    F(2, 4) = dt;

    return F;
}

Eigen::MatrixXd EKF_CV::H(const Eigen::VectorXd &x) {
    // Measurement function Jacobian matrix
    // Assuming we measure [x, y, theta]
    Eigen::MatrixXd H(2, x.size());
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    return H;
}

Eigen::VectorXd EKF_CV::h(const Eigen::VectorXd &x) {
    // Measurement function
    // Assuming we measure [x, y, theta]
    Eigen::VectorXd z_pred(2);
    z_pred(0) = x(0); // x position
    z_pred(1) = x(1); // y position
    return z_pred;
}
