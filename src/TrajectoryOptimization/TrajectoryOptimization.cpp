#include "TrajectoryOptimization/TrajectoryOptimization.h"

using namespace casadi;

TrajectoryOptimization::TrajectoryOptimization(double width, double length, double T_, double N_){
    N = N_;
    T = T_;
    Width = width;
    Length = length;
    G = MX::zeros(4,2);
    G(0,0) = 1;G(1,1) = 1;G(2,0) = -1;G(3,1) = -1;
    e_g = MX::zeros(4,1);
    e_g(0) = Width/2;e_g(1) = Length/2;e_g(2) = Width/2;e_g(3) = Length/2;
    
}

TrajectoryOptimization::~TrajectoryOptimization()
{
}

inline MX TrajectoryOptimization::Rx(MX theta){
    return vertcat(horzcat(cos(theta),sin(theta)),horzcat(-sin(theta),cos(theta)));
}

inline MX TrajectoryOptimization::tx(MX off){
    return vertcat(off(0),off(1));
}

void TrajectoryOptimization::MX_Init(){
    states = MX::sym("states",n_states,1);
    controls = MX::sym("controls",n_control,1);
    rhs = vertcat(controls(0)*cos(states(2)),controls(0)*sin(states(2)),controls(1));
    f = Function("f",{states,controls},{rhs});

    Lambda = MX::sym("Lambda",2,N);
    Mu = MX::sym("Mu",4,N);
    Nu = MX::sym("Nu",1,N);

    
    U = MX::sym("U",n_control,N-1);
    X = MX::sym("X",n_states,N);
    // params set
    P = MX::sym("P",n_states,N+1);
    A_set = MX::sym("A_set",4,N);
    b_set = MX::sym("b_set",2,N);
    F_set = MX::sym("F_set",4,N);


    obj = MX::sym("obj");
    g = MX::sym("g");

    obj = 0;

    // weight matrices(states && controls)
    MX Q = MX::zeros(3,3);
    MX J = MX::zeros(3,3);
    MX R1 = MX::zeros(2,2);  
    MX R2 = MX::zeros(2,2); 
    Q(0,0) = 0.5;Q(1,1) = 0.5;Q(2,2) = 0.3;
    R1(0,0) = 0.5;R1(1,1) = 0.05;
    R2(0,0) = 1;R2(1,1) = 0.1;
    J(0,0) = 3;J(1,1) = 3;J(2,2) = 3;

    // the initial state constraint
    g = X(Slice(),0)-P(Slice(),1);

    // the goal state
    MX st_goal = P(Slice(),0);

    // the objective function
    for(int k = 0; k < N - 1; ++k){
        MX con_last;
        if(k){
            con_last = U(Slice(),k);
        }
        else{
            con_last = U(Slice(),k-1);
        }
        MX st = X(Slice(),k);
        MX con = U(Slice(),k);    
        MX st_next = X(Slice(),k+1); 
        MX st_ref = P(Slice(),k);  
        if(k<=4){
            obj += mtimes(mtimes((st-st_ref).T(),J),st-st_ref) + mtimes(mtimes(con.T(),R1),con)
                    +mtimes(mtimes((st_next-st_goal).T(),Q),st_next-st_goal) + mtimes(mtimes((con-con_last).T(),R2),con-con_last);
        }
        else{
            obj += mtimes(mtimes(con.T(),R1),con)
                    +mtimes(mtimes((st_next-st_goal).T(),Q),st_next-st_goal) + mtimes(mtimes((con-con_last).T(),R2),con-con_last);
        }

        auto res = f(MXIList{st,con});
        MX f_value = res[0];
        MX st_next_euler = st + T*f_value;
        g = vertcat(g,st_next-st_next_euler);  
    }
    std::cout << "g_size:" << g.size() <<std::endl;
    // add the constraints for collision avoidance
    for(int k = 0; k < N; ++k){
        MX off_m = X(Slice(0,2),k);
        MX alpha_m = X(2,k);
        MX lambda_m = Lambda(Slice(),k);
        MX nu_m = Nu(Slice(),k);
        MX mu_m = Mu(Slice(),k);
        // 
        MX tmp_A = horzcat(A_set(Slice(0,2),k),A_set(Slice(2,4),k));
        MX tmp_F = horzcat(F_set(Slice(0,2),k),F_set(Slice(2,4),k));
        MX temp_b = b_set(Slice(),k);
        //
        // MX tmp_o = lambda_m.T() * tmp_A.T() / (tmp_A*2*nu_m);
        // MX tmp1 = nu_m * tmp_o * tmp_F * tmp_o.T();
        // MX tmp2 = lambda_m.T() * tmp_A.T() * tmp_o.T();
        // MX g1 = tmp1 - tmp2 - e_g.T() * mu_m + (tx(off_m)-b).T()*lambda_m - nu_m;
        // MX g2 = mu_m.T()*G + lambda_m.T()*Rx(alpha_m);
        // MX g3 = lambda_m.T() * lambda_m;
        MX tmp_o = solve(mtimes(tmp_A * 2, nu_m),mtimes(lambda_m.T(), tmp_A.T()).T()).T();
        MX tmp1 = mtimes(mtimes(nu_m, tmp_o), mtimes(tmp_F, tmp_o.T()));
        MX tmp2 = mtimes(mtimes(lambda_m.T(), tmp_A.T()), tmp_o.T());
        MX g1 = tmp1 - tmp2 - mtimes(e_g.T(), mu_m) + mtimes((off_m - temp_b).T(), lambda_m) - nu_m;
        MX g2 = mtimes(mu_m.T(), G) + mtimes(lambda_m.T(), Rx(alpha_m));
        MX g3 = mtimes(lambda_m.T(), lambda_m);
        g = vertcat(g,g1,g2.T(),g3);  
    }
    // g = vertcat(g,mtimes((X(Slice(),N-1)-P(Slice(),N-1)).T(),(X(Slice(),N-1)-P(Slice(),N-1))));
    std::cout << "g_size:" << g.size() <<std::endl;
    std::vector<MX> vars = {reshape(X,n_states*N,1),reshape(U,n_control*(N-1),1),reshape(Lambda,2*N,1),reshape(Mu,4*N,1),reshape(Nu,N,1)};
    MX OPT_variables = vertcat(vars);
    std::vector<MX> p_vars = {reshape(P,n_states*(N+1),1),reshape(A_set,4*N,1),reshape(F_set,4*N,1),reshape(b_set,2*N,1)};
    MX Params_variables = vertcat(p_vars);
    // NLP
    nlp = {{"x",OPT_variables},{"f",obj},{"p",Params_variables},{"g",g}};
    solver = nlpsol("solver", "ipopt", nlp);
}

void TrajectoryOptimization::Add_Constraints(const double x_lower,const double x_upper,
                                             const double y_lower,const double y_upper,
                                             const double v_min, const double v_max,
                                             const double omega_min, const double omega_max,
                                             double distance){
    lbx.clear();
    ubx.clear();
    lbg.clear();
    ubg.clear();
    // the constraints for the states
    for(int k = 0; k < N; ++k){
        lbx.push_back(x_lower);
        ubx.push_back(x_upper);
        lbx.push_back(y_lower);
        ubx.push_back(y_upper);
        lbx.push_back(-inf);
        ubx.push_back(inf);
    }
    // the constraints for the controls
    for(int k = 0; k < N-1; ++k){
        lbx.push_back(v_min);
        ubx.push_back(v_max);
        lbx.push_back(omega_min);
        ubx.push_back(omega_max);
    }
    // the constraints for the lambda
    for(int k = 0; k < 2*N; ++k){
        lbx.push_back(-inf);
        ubx.push_back(inf);
    }
    // the constraints for the mu and nu
    for(int k = 0; k < 5*N; ++k){
        lbx.push_back(0);
        ubx.push_back(inf);
    }
    std::cout << "lbx_size:" << lbx.size() <<std::endl;
    std::cout << "ubx_size:" << ubx.size() <<std::endl;

    // the constraints for the kinematic model
    for(int k = 0;k<n_states*N;++k){
        lbg.push_back(0);
        ubg.push_back(0);
    }
    // the constraints for g1, g2, g3
    for(int k = 0;k < N; ++k){
        // g1
        lbg.push_back(distance);
        ubg.push_back(inf);
        // g2
        lbg.push_back(0);
        ubg.push_back(0);
        lbg.push_back(0);
        ubg.push_back(0);
        // g3
        lbg.push_back(0);
        ubg.push_back(1);
    }
    // // the constraints for the final state
    // lbg.push_back(0);
    // ubg.push_back(0.2);
    std::cout << "lbg_size:" << lbg.size() <<std::endl;
    std::cout << "ubg_size:" << ubg.size() <<std::endl;
}

void TrajectoryOptimization::Warm_Start(std::vector<std::vector<double>>& x, std::vector<std::vector<double>>& u){
    DM lambda0 = DM::ones(2,N);
    DM mu0 = DM::ones(4,N);
    DM nu0 = DM::ones(1,N);
    DM x0 = DM::zeros(n_states*N,1);
    DM u0 = DM::zeros(n_control*(N-1),1);
    int k = 0;
    for(auto xi:x){
        x0(k++) = xi[0];
        x0(k++) = xi[1];
        x0(k++) = xi[2];
    }
    // std::cout << "x0_size:" << k-1 << std::endl;
    // k = 0;
    // for(auto ui:u){
    //     u0(k++) = ui[0];
    //     u0(k++) = ui[1];
    // }
    // std::cout << "u0_size:" << k-1 << std::endl;
    DM p0 = x0;
    std::vector<DM> vert_x0 = {x0,u0,reshape(lambda0,2*N,1),reshape(mu0,4*N,1),reshape(nu0,N,1)};
    X0 = vertcat(vert_x0);
    P0 =p0;
}

void TrajectoryOptimization::AFb_Init(std::vector<std::vector<double>>& AFb_array){
    A.clear();
    b.clear();
    F.clear();
    for(auto afb:AFb_array){
        MX b_m = MX::zeros(2,1);
        b_m(0) = afb[0];b_m(1) = afb[1];
        b.push_back(b_m);
        MX F_m = MX::zeros(2,2);
        F_m(0,0) = 1/(afb[2]*afb[2]);F_m(1,1) = 1/(afb[3]*afb[3]);
        F.push_back(F_m);
        MX A_m = MX::zeros(2,2);
        A_m(0,0) = cos(afb[4]);A_m(0,1) = sin(afb[4]);A_m(1,0) = -sin(afb[4]);A_m(1,1) = cos(afb[4]);
        A.push_back(A_m);
    }
}

void TrajectoryOptimization::Solve(std::vector<double>& x_res,std::vector<double>& u_res){
    std::map<std::string, DM> arg, res;
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = X0;
    arg["p"] = P0;
    res = solver(arg); 
    auto result = res["x"].nonzeros();
    for(int k = 0; k < n_states*N; ++k){
        x_res.push_back(result[k]);
    }
    for(int k = n_states*N; k < n_states*N+n_control*(N-1); ++k){
        u_res.push_back(result[k]);
    }
}