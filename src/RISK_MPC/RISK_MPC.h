#ifndef _RISK_MPC_H_
#define _RISK_MPC_H_

#include <vector>
#include <casadi/casadi.hpp>
#include <Eigen/Eigen>

using namespace casadi;

struct obs_circle
{
    double x;
    double y;
    double r;
};
struct cmd
{
    double v;
    double w; 
    cmd() : v(0),w(0){}
};
struct state
{
    double x;
    double y;
    double yaw;
    double v;
    double w;
    state() : x(0),y(0),yaw(0),v(0),w(0){}
};

class RISK_MPC
{
private:
    // struct obs_circle
    // {
    //     double x;
    //     double y;
    //     double r;
    // };
    // struct cmd
    // {
    //     double v;
    //     double w; 
    //     cmd() : v(0),w(0){}
    // };
    // struct state
    // {
    //     double x;
    //     double y;
    //     double yaw;
    //     double v;
    //     double w;
    //     state() : x(0),y(0),yaw(0),v(0),w(0){}
    // };
    double v_max;
    double v_min;
    double omega_max;
    double omega_min;
    int N;
    double dt;
    std::vector<obs_circle> obs_array;
    // MX states;
    // MX controls;
    // MX rhs;
    // Function solver;
    MXDict nlp;
    DM X0_p;
    
public:
    RISK_MPC();
    ~RISK_MPC();
    void set_model_parameters(const double v_max, const double v_min, const double omega_max, const double omega_min);
    void set_prediction_horizon(const int N, const double dt);
    void initialize();
    cmd compute(const std::vector<double>& state_now, const std::vector<double>& goal_state, const std::vector<obs_circle>& obs_array);
};




#endif