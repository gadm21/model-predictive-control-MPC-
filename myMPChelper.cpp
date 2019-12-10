#include <bits/stdc++.h>
using namespace std;
#define M_PI 3.14159265358979323846
#define vectorD vector<double>
#define matrixD vector<vector<double>>
#define ui unsigned int

bool path_pts_wrt_origin = false;
double voltage2speed_weight = 2;
double dt = 0.4;
vectorD coeff;

// const double cte_weight = 1000;
// const double epsi_weight = 10;
// const double v_weight = 10;
// const double acceleration_cost_weight = 5;
// const double steering_cost_weight = 0.7;
// const double change_steer_cost_weight = 40;
// const double change_motor_cost_weight = 10;

const double cte_weight = 6000;
const double epsi_weight = 1000;
const double v_weight = 100;
const double acceleration_cost_weight = 10;
const double steering_cost_weight = 10;
const double change_steer_cost_weight = 6000;
const double change_motor_cost_weight = 800;

vectorD motor_dof = {1, 2, 5};
vectorD steering_dof = {-20, -10, -5, -2, 0, 2, 5, 10, 20};

int N = 3;
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = motor_dof[1] * voltage2speed_weight;

double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x / 180 * M_PI; }

double polyeval(vectorD coeffs, double x)
{

    //return sin(x);

    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
        result += coeffs[i] * pow(x, i);

    return result;
}

class State
{
public:
    double x, y, psi, v, cte, epsi;
    double MV = 0, steering_angle = 0;
    bool activated = false;

    State(double x_, double y_, double psi_,
          double v_, double cte_, double epsi_)
    {
        x = x_;
        psi = psi_;
        y = y_;
        cte = cte_;
        v = v_;
        epsi = epsi_;
    }
    State activate(double mv, double s_a)
    {
        activated = true;
        MV = mv;
        steering_angle = s_a;

        double new_v = voltage2speed_weight * MV;
        double new_psi = psi + (new_v * deg2rad(steering_angle) * dt);

        double new_x = x + (cos(new_psi) * new_v * dt);
        double new_y = y + (sin(new_psi) * new_v * dt);
        double new_cte = polyeval(coeff, new_x) - new_y;
        double psi_des = atan(coeff[1] + (2 * coeff[2] * new_x) + (3 * coeff[2] * new_x * new_x));
        double new_epsi = new_psi - psi_des;

        return State(new_x, new_y, new_psi,
                     new_v, new_cte, new_epsi);
    }

    void print_state_report()
    {
        cout << "........................state report......................" << endl;
        cout << "x:" << x << endl;
        cout << "y:" << y << endl;
        cout << "v:" << v << endl;
        cout << "cte:" << cte << endl;
        cout << "epsi:" << epsi << endl;
        cout << "MV:" << MV << endl;
        cout << "steering angle:" << steering_angle << endl;
        cout << "activated:" << activated << endl;
        cout << ".............................................................." << endl;
    }
};

class Path
{
public:
    double path_cost = 0.0;

    vector<State> states;

    Path() { states = vector<State>(); }
    Path(State s)
    {
        states = vector<State>();
        states.push_back(s);
    }
    Path(vector<State> ss)
    {
        states = ss;
    }
    Path add_state(double mv, double steering_angle)
    {
        State last_state = states[states.size() - 1];

        vector<State> new_states = states;
        State new_state = new_states[new_states.size() - 1].activate(mv, steering_angle);
        new_states.push_back(new_state);

        return Path(new_states);
    }
    double calculate_cost()
    {
        double cost = 0.0;

        double cte_cost = 0;
        double epsi_cost = 0;
        double v_cost = 0;
        double steering_cost = 0;
        double accel_cost = 0;
        double change_steering_cost = 0;
        double change_motor_cost = 0;

        for (int i = 0; i < states.size(); i++)
        {
            cte_cost += cte_weight * abs(states[i].cte - ref_cte);
            epsi_cost += epsi_weight * abs(states[i].epsi - ref_epsi);
            v_cost += v_weight * abs(states[i].v - ref_v);
        }

        //states.size()-1 becuase the last state doens't
        //have control inputs (isn't activated)
        for (int i = 0; i < states.size() - 1; i++)
        {
            steering_cost += steering_cost_weight * abs(states[i].steering_angle);
            accel_cost += acceleration_cost_weight * abs(states[i].MV);
        }

        //states.size()-2 becuase we're compaing two states each
        //time, and becuase the last state doesn't have control inputs
        for (int i = 0; i < states.size() - 2; i++)
        {
            change_steering_cost += change_steer_cost_weight * abs(states[i + 1].steering_angle - states[i].steering_angle);
            change_motor_cost += change_motor_cost_weight * abs(states[i + 1].MV - states[i].MV);
        }

        cost += cte_cost + epsi_cost + v_cost + steering_cost + accel_cost +
                change_steering_cost + change_motor_cost;

        path_cost = cost;
        return cost;
    }
    void print_path_report()
    {
        cout << "........................path report......................" << endl;
        cout << "number of states= " << states.size() << endl;
        cout << "next_x:" << states[1].x << endl;
        cout << "next_y:" << states[1].y << endl;
        cout << "next_psi:" << states[1].psi << endl;
        cout << "next_v:" << states[1].v << endl;
        cout << "next_cte:" << states[1].cte << endl;
        cout << "next_epsi:" << states[1].epsi << endl;
        cout << "next_steering:" << states[1].steering_angle << endl;
        cout << "next_MV:" << states[1].MV << endl;
        cout << ".............................................................." << endl;
    }
    void print_points(){
        int n= states.size();
        for(int i=0; i<n; i++)
            cout<<states[i].x<<(i==(n-1)? '\n': ',');
        
        for(int i=0; i<n; i++)
            cout<<states[i].y<<(i==(n-1)? '\n': ',');
    }
};

class Optimizer
{
private:
    time_t optimization_time = 0;
    ui num_paths = 0;
    double lowest_cost_found = 0.0;
    double highest_cost_found = 0.0;

public:
    vector<Path> paths;

    Optimizer(State s)
    {
        paths = vector<Path>();
        paths.push_back(Path(s));
    }

    Path optimize()
    {
        time_t start = time(NULL);

        //iterate over all possible combinations of control inputs
        for (int n = 0; n < N; n++)
        {
            int current_pathes_length = paths.size();
            vector<Path> new_paths = vector<Path>();
            for (int path = 0; path < current_pathes_length; path++)
            {
                for (int i = 0; i < motor_dof.size(); i++)
                {
                    for (int j = 0; j < steering_dof.size(); j++)
                    {
                        double m = motor_dof[i], s = steering_dof[j];
                        new_paths.push_back(paths[path].add_state(m, s));
                    }
                }
            }
            paths = new_paths;
        }

        //find the lowest cost between all calculated pathes
        double lowest_cost = numeric_limits<double>::max();
        double highest_cost = numeric_limits<double>::min();
        Path lowest_cost_path;

        for (ui i = 0; i < paths.size(); i++)
        {
            double path_cost = paths[i].calculate_cost();
            if (path_cost < lowest_cost)
            {
                lowest_cost_path = paths[i];
                lowest_cost = path_cost;
            }
            if (path_cost > highest_cost)
                highest_cost = path_cost;
        }

        time_t end = time(NULL);

        optimization_time = end - start;
        lowest_cost_found = lowest_cost;
        highest_cost_found = highest_cost;
        num_paths = paths.size();
        cout << "choosing mv:" << lowest_cost_path.states[0].MV << " steering:" << lowest_cost_path.states[0].steering_angle << endl;
        return lowest_cost_path;
    }

    void print_optimizer_report()
    {
        cout << "........................optimizer report......................" << endl;
        cout << "optimization took:" << optimization_time << " and " << num_paths
             << " were generated." << endl;
        cout << "the highest cost path has a cost of:" << highest_cost_found << endl;
        cout << "and the lowest cost path has a cost of:" << lowest_cost_found << endl;
        cout << "................................................................" << endl
             << endl;
    }
};