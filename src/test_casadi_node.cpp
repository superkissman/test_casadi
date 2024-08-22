#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <casadi/casadi.hpp>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

using namespace casadi;

class risk_mpc{

    private:
        ros::NodeHandle nh;
        ros::Subscriber mot_tracking_sub,odom_sub,goal_sub;
        ros::Publisher cmd_pub;
        ros::Timer risk_mpc_timer;
        
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
            double a;
            state() : x(0),y(0),yaw(0),v(0),w(0),a(0){}
        };
        
        
        

    public:
        //全局变量
        // std::vector<double> pos = {0,0,0};
        // std::vector<double> cmd = {0,0};
        std::vector<obs_circle> obs_array;
        std::vector<cmd> cmd_history_array;
        cmd cmd_now;
        state state_now;
        state goal_state;
        const int N = 10;
        const double dt = 0.1;
        DM X0_p = DM::zeros(5*N + 3,1);

        risk_mpc(){
            mot_tracking_sub = nh.subscribe<visualization_msgs::MarkerArray>("/obstacle_markers",10,boost::bind(&risk_mpc::mot_tracking_callback,this,_1));
            cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
            risk_mpc_timer = nh.createTimer(ros::Duration(dt),boost::bind(&risk_mpc::risk_mpc_timer_callback,this,_1));
            odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 5, boost::bind(&risk_mpc::odom_sub_callback,this,_1));
            goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, boost::bind(&risk_mpc::goal_sub_callback,this,_1));
        }

        void goal_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
            geometry_msgs::Pose pose = msg->pose;
            geometry_msgs::Quaternion quat = pose.orientation;
            // 使用 tf2 提供的工具函数将 Quaternion 转换为欧拉角
            double roll, pitch, yaw;
            tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
            tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

            goal_state.x = pose.position.x;
            goal_state.y = pose.position.y;
            goal_state.yaw = yaw;
            
        }

        void odom_sub_callback(const nav_msgs::Odometry::ConstPtr& msg){
            geometry_msgs::Pose pose = msg->pose.pose;
            // 提取 Quaternion
            geometry_msgs::Quaternion quat = pose.orientation;
            // 使用 tf2 提供的工具函数将 Quaternion 转换为欧拉角
            double roll, pitch, yaw;
            tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
            tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

            state_now.x = pose.position.x;
            state_now.y = pose.position.y;
            state_now.yaw = yaw;
            casadi_main();
        }

        void casadi_main(){
            std::cout << "casadi_test" << std::endl;

            // set the prediction horizon
            
            // set the delta-time
            // double dt = 0.1;
            //set the v and w
            double v_max = 0.6;
            double v_min = -v_max;
            double omega_max = M_PI/4;
            double omega_min = -omega_max;

            // set the size of the X and controls
            int n_states = 3;
            int n_controls = 2;

            MX states = MX::sym("states",n_states,1);
            MX controls = MX::sym("controls",n_controls,1);
            MX rhs = vertcat(controls(0)*cos(states(2)),controls(0)*sin(states(2)),controls(1));
            // std::cout << "rhs:" << rhs.size()<<std::endl;

            // Objective
            Function f = Function("f",{states,controls},{rhs});

            //Decision variables (controls)
            MX U = MX::sym("U",n_controls,N);

            //parameters( which include at the initial state of the robot and the reference state)
            MX P = MX::sym("P",2*n_states,1);

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
            g = st-P(Slice(0,3));
            // std::cout << "P(Slice(0,2)):" << P(Slice(3,6)).size()<<std::endl;
            
            
            MX Q = MX::zeros(3,3);
            MX R = MX::zeros(2,2);
            Q(0,0) = 5;Q(1,1) = 5;Q(2,2) = 0.1;
            R(0,0) = 0.5;R(1,1) = 0.05;

            MX con = MX::sym("con",n_controls,1);
            for(int k = 0; k < N; ++k){
                st = X(Slice(),k);
                con = U(Slice(),k);       
                obj +=  mtimes(mtimes((st-P(casadi::Slice(3, 6))).T(),Q),st-P(casadi::Slice(3, 6)))+mtimes(mtimes(con.T(),R),con);
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
            MX obs_diam;
            MX rob_diam = 1.0;

            for(int k = 0;k< N+1;++k){
                for(auto obsi : obs_array){
                    obs_x = obsi.x;
                    obs_y = obsi.y;
                    obs_diam = obsi.r;
                    g = vertcat(g, -sqrt(pow((X(1,k)-obs_x),2)+pow((X(2,k)-obs_y),2)) + (rob_diam/2 + obs_diam));
                }
                    
            }
            std::cout << "g:" << g.size() <<std::endl;

            MX OPT_variables = vertcat(reshape(X,3*(N+1),1),reshape(U,2*N,1));

            // NLP
            MXDict nlp = {{"x",OPT_variables},{"f",obj},{"p",P},{"g",g}};

            // Initial guess and bounds for the optimization variables
            std::vector<double> lbx,ubx;
            for(int k = 0;k<(N+1);++k){
                lbx.push_back(-5);
                ubx.push_back(5);
                lbx.push_back(-5);
                ubx.push_back(5);
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
            for(int k = 3*(N+1);k < (3+obs_array.size())*(N+1);++k){
                lbg.push_back(-inf);
                ubg.push_back(0);
            }
            std::cout << "lbg:" << lbg.size() <<std::endl;
            std::cout << "ubg:" << ubg.size() <<std::endl;

            //x0 is the real position, xs is the goal position
            DM x0 = DM::zeros(3,1);x0(0)=state_now.x;x0(1)=state_now.y;x0(2)=state_now.yaw;
            DM xs = DM::zeros(3,1);xs(0)=goal_state.x;xs(1)=goal_state.y;xs(2)=goal_state.yaw;
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
            arg["p"] = vertcat(x0,xs);
            res = solver(arg); 
            auto result = res["x"].nonzeros();
            
            // auto result = Eigen::VectorXd(res["x"]);
            cmd_now.v = result[3*(N+1)];
            cmd_now.w = result[3*(N+1)+1];

            auto X0_x = reshape(res["x"](Slice(0,3*(N+1))),3,N+1).T();
            auto X0_X = vertcat(X0_x(Slice(1,N+1),Slice()),X0_x(N,Slice()));
            auto X0_u = reshape(res["x"](Slice(3*(N+1),5*N+3)),2,N).T();
            X0_p = vertcat(reshape(X0_X,3*(N+1),1),reshape(X0_u,2*N,1));

            // Print the solution
            std::cout << "--------------------------------" << std::endl;
            std::cout << "result:" << result.size()<<std::endl;
            std::cout << "The receive goal: x->" << goal_state.x << "   y->" << goal_state.y << "   yaw->" << goal_state.yaw << std::endl;
            std::cout << "The current state: x->" << state_now.x << "   y->" << state_now.y << "   yaw->" << goal_state.yaw << std::endl;
            std::cout << "cmd_v: " << cmd_now.v << std::endl;
            std::cout << "cmd_w: " << cmd_now.w << std::endl;
        }

        void mot_tracking_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){
            std::cout << "The obs num :" << msg->markers.size()<<std::endl;
            obs_array.clear();
            obs_array.swap(obs_array);
            for(auto obs : msg->markers){
                obs_circle temp;
                temp.x = obs.pose.position.x;
                temp.y = obs.pose.position.y;
                temp.r = sqrt(pow(obs.scale.x/2,2)+pow(obs.scale.y/2,2));
                obs_array.push_back(temp);
            }
        }

        void risk_mpc_timer_callback(const ros::TimerEvent& event){
            geometry_msgs::Twist Cmd;
            Cmd.linear.x = cmd_now.v;
            Cmd.angular.z = cmd_now.w;
            // std::cout << "The pub Cmd: " << Cmd << std::endl;
            cmd_pub.publish(Cmd);
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_casadi_node");
    risk_mpc rmpc;
    
    ros::spin();
    return 0;
}

