#ifndef _EKF_H
#define _EKF_H

#include <Eigen/Dense>
#include <vector>

class EKF_base {
public:
    EKF_base();
    virtual ~EKF_base();

    inline double normalizeAngle(double angle);
    void initialize(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, const Eigen::MatrixXd &Q0, const Eigen::MatrixXd &R0, const double dt0);
    void predict();
    void self_predict(const int step, std::vector<Eigen::VectorXd>& X_pred, std::vector<Eigen::MatrixXd>& P_pred);
    void update(const Eigen::VectorXd &z);

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

protected:
    virtual Eigen::VectorXd f(const Eigen::VectorXd &x, double dt) = 0;
    virtual Eigen::MatrixXd F(const Eigen::VectorXd &x, double dt) = 0;
    virtual Eigen::MatrixXd H(const Eigen::VectorXd &x) = 0;
    virtual Eigen::VectorXd h(const Eigen::VectorXd &x) = 0;

private:
    Eigen::VectorXd x; // 状态向量
    Eigen::MatrixXd P; // 状态协方差矩阵
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    double dt;
    bool is_initialized;
};

//CV model
class EKF_CV : public EKF_base {
public:
    EKF_CV();
    virtual ~EKF_CV();
protected:
    Eigen::VectorXd f(const Eigen::VectorXd &x, double dt) override;
    Eigen::MatrixXd F(const Eigen::VectorXd &x, double dt) override;
    Eigen::MatrixXd H(const Eigen::VectorXd &x) override;
    Eigen::VectorXd h(const Eigen::VectorXd &x) override;
};

#endif // EKF_H
