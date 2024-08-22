#ifndef TRAJECTORYOPTIMIZATION_H
#define TRAJECTORYOPTIMIZATION_H

#include <casadi/casadi.hpp>
#include <vector>

using namespace casadi;

class TrajectoryOptimization
{
private:
    double Width,Length;
    MX G,e_g;
    std::vector<MX> A,b,F;
    int n_states = 3;
    int n_control = 2;
    int N;
    double T;
    MX states;MX controls;MX rhs;
    MX Lambda;MX Mu;MX Nu;
    MX P;MX U;MX X;
    MX A_set;
    MX b_set;
    MX F_set;


    Function f;
    MX obj;MX g;

    MXDict nlp;
    Function solver;

    DM X0,P0;

    std::vector<double> lbx,ubx;
    std::vector<double> lbg,ubg;

    inline MX Rx(MX theta);
    inline MX tx(MX off);
public:
    TrajectoryOptimization(double Width, double Length, double T_, double N_);
    ~TrajectoryOptimization(); 
    void MX_Init();
    void Add_Constraints(const double x_lower,const double x_upper,
                         const double y_lower,const double y_upper,
                         const double v_min, const double v_max,
                         const double omega_min, const double omega_max,
                         double distance);
    void Warm_Start(std::vector<std::vector<double>>& x, std::vector<std::vector<double>>& u);
    void AFb_Init(std::vector<std::vector<double>>& AFb_array);
    void Solve(std::vector<double>& x_res,std::vector<double>& u_res);
};


#endif //TRAJECTORYOPTIMIZATION_H