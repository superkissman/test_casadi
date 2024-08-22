#include "RISK_MPC.h"

RISK_MPC::RISK_MPC():v_max(0.6),v_min(-0.6),omega_max(M_PI/4),omega_min(-M_PI/4),N(10),dt(0.1){};

RISK_MPC::~RISK_MPC(){};

void RISK_MPC::set_model_parameters(const double v_max, const double v_min, const double omega_max, const double omega_min){
    this->v_max = v_max;
    this->v_min = v_min;
    this->omega_max = omega_max;
    this->omega_min = omega_min;
}

void RISK_MPC::set_prediction_horizon(const int N, const double dt){
    this->N = N;
    this->dt = dt;
}

void RISK_MPC::initialize(){
    int n_states = 3;
    int n_controls = 2;

    MX states = MX::sym("states",n_states,1);
    MX controls = MX::sym("controls",n_controls,1);
    MX rhs = vertcat(controls(0)*cos(states(2)),controls(0)*sin(states(2)),controls(1));
    X0_p = DM::zeros(5*N + 3,1);
    // std::cout << "rhs:" << rhs.size()<<std::endl;

    // Objective
    Function f = Function("f",{states,controls},{rhs});

    //Decision variables (controls)
    MX U = MX::sym("U",n_controls,N);

    //parameters( which include at the initial state of the robot and the reference state)
    MX P_state = MX::sym("P_state",2*n_states,1);
    MX P_obs = MX::sym("P_obs",N+1,2);

    //A vector that represents the states over the optimization problem
    MX X = MX::sym("X",n_states,N+1);

    // Objective function
    MX obj = MX::sym("obj",1);

    obj = 0;

    MX st = MX::sym("st",n_states,1);
    st = X(Slice(),0);
    // std::cout << "sX(Slice(),0):" << X(Slice(),0) <<std::endl;
    // std::cout << "st:" << st.size()<<std::endl;
    // std::cout << "P:" << P.rows() <<std::endl;

    //constraints vector
    MX g = MX::sym("g");
    g = st-P_state(Slice(0,3));
    // std::cout << "P(Slice(0,2)):" << P(Slice(3,6)).size()<<std::endl;
    
    
    MX Q = MX::zeros(3,3);
    MX R = MX::zeros(2,2);
    Q(0,0) = 5;Q(1,1) = 5;Q(2,2) = 0.1;
    R(0,0) = 0.5;R(1,1) = 0.05;

    MX con = MX::sym("con",n_controls,1);
    for(int k = 0; k < N; ++k){
        st = X(Slice(),k);
        con = U(Slice(),k);       
        obj +=  mtimes(mtimes((st-P_state(casadi::Slice(3, 6))).T(),Q),st-P_state(casadi::Slice(3, 6)))+mtimes(mtimes(con.T(),R),con);
        MX st_next = MX::sym("st_next",n_states,1);
        st_next = X(Slice(),k+1);
        auto res = f(MXIList{st,con});
        MX f_value = res[0];
        MX st_next_euler = st + dt*f_value;
        g = vertcat(g,st_next-st_next_euler);  
        // std::cout << "g:" << g.size()<<std::endl;              
    }

    MX obs_x;
    MX obs_y;
    MX obs_diam = 0.5;
    MX rob_diam = 1;

    for(int k = 0;k< N+1;++k){
        obs_x = P_obs(k,0);
        obs_y = P_obs(k,1);
        g = vertcat(g, -sqrt(pow((X(1,k)-obs_x),2)+pow((X(2,k)-obs_y),2)) + (rob_diam/2 + obs_diam));        
    }
    std::cout << "g:" << g.size() <<std::endl;

    MX OPT_variables = vertcat(reshape(X,3*(N+1),1),reshape(U,2*N,1));

    // NLP
    nlp = {{"x",OPT_variables},{"f",obj},{"p",vertcat(P_state,reshape(P_obs,2*(N+1),1))},{"g",g}};
}

cmd RISK_MPC::compute(const std::vector<double>& state_now, const std::vector<double>& goal_state, const std::vector<obs_circle>& obs_array){
    cmd cmd_;
    // Initial guess and bounds for the optimization variables
    std::vector<double> lbx,ubx;
    for(int k = 0;k<(N+1);++k){
        lbx.push_back(-inf);
        ubx.push_back(inf);
        lbx.push_back(-inf);
        ubx.push_back(inf);
        lbx.push_back(-inf);
        ubx.push_back(inf);
    }
    for(int k = 0;k<N;++k){
        lbx.push_back(v_min);
        ubx.push_back(v_max);
        lbx.push_back(omega_min);
        ubx.push_back(omega_max);
    }
    std::cout << "lbx:" << lbx.size() <<std::endl;
    std::cout << "ubx:" << ubx.size() <<std::endl;

    // Nonlinear bounds
    std::vector<double> lbg,ubg;
    for(int k = 0;k<3*(N+1);++k){
        lbg.push_back(0);
        ubg.push_back(0);
    }
    for(int k = 3*(N+1);k < 4*(N+1); ++k){
        lbg.push_back(-inf);
        ubg.push_back(0);
    }
    std::cout << "lbg:" << lbg.size() <<std::endl;
    std::cout << "ubg:" << ubg.size() <<std::endl;

    //x0 is the real position, xs is the goal position
    DM x0 = DM::zeros(3,1);x0(0)=state_now[0];x0(1)=state_now[1];x0(2)=state_now[2];
    DM xs = DM::zeros(3,1);xs(0)=goal_state[0];xs(1)=goal_state[1];xs(2)=goal_state[2];
    DM p_obs = DM::zeros(2*(N+1),1);
    for(int k = 0;k<obs_array.size();++k){
        p_obs(2*k) = obs_array[k].x;
        p_obs(2*k+1) = obs_array[k].y;
    }
    // X0_p
    // DM X0 = DM::zeros(5*N + 3,1);

    // Create NLP solver and buffers
    Function solver = nlpsol("solver", "ipopt", nlp);//, {{"ipopt.linear_solver", "ma27"}});    
    std::map<std::string, DM> arg, res;

    // Solve the NLP
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = X0_p;
    arg["p"] = vertcat(x0,xs,p_obs);
    res = solver(arg); 
    auto result = res["x"].nonzeros();
    
    // auto result = Eigen::VectorXd(res["x"]);
    cmd_.v = result[3*(N+1)];
    cmd_.w = result[3*(N+1)+1];

    auto X0_x = reshape(res["x"](Slice(0,3*(N+1))),3,N+1).T();
    auto X0_X = vertcat(X0_x(Slice(1,N+1),Slice()),X0_x(N,Slice()));
    auto X0_u = reshape(res["x"](Slice(3*(N+1),5*N+3)),2,N).T();
    X0_p = vertcat(reshape(X0_X,3*(N+1),1),reshape(X0_u,2*N,1));
    return cmd_;
}