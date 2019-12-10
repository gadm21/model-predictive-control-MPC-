#include "myMPChelper.cpp"
#include "myMPCfit.cpp"



vectorD polyfit(vectorD xvals, vectorD yvals)
{
    //for ax3+ bx2+ cx+ d, the pattern returned is {d, c, b, a}
    vectorD coeff;
    fitIt(xvals ,xvals , 3, coeff);

    cout<<"coeff: ";
    for(int i=0; i<coeff.size(); i++) cout<<coeff[i]<<" ";
    cout<<endl;

    return {0, 0, 1, 1};

    int order = 3;
    matrixD A(xvals.size(), vectorD(order + 1, 0.0));

    for (int i = 0; i < xvals.size(); i++)
        A[i][0] = 1.0;

    for (int j = 0; j < xvals.size(); j++)
        for (int i = 0; i < order; i++)
            A[j][i + 1] = A[j][i] * xvals[j];

    vectorD result;
    return result;
}



void get_state(vectorD &states, vectorD &pts_x, vectorD &pts_y)
{
    pts_x = { -5, -1, 0, 1, 5};
    pts_y = { 25, 1, 5, 1, 25};
    states = {-5, -5, deg2rad(0), 2};
}

void change_reference_system(vectorD &pts_x, vectorD &pts_y, double x, double y, double psi)
{
    for (int i = 0; i < pts_x.size(); i++)
    {

        //shift
        double shift_x = pts_x[i] - x;
        double shift_y = pts_y[i] - y;

        //rotate
        pts_x[i] = (shift_x * cos(-psi)) - (shift_y * sin(-psi));
        pts_y[i] = (shift_x * sin(-psi)) - (shift_y * cos(-psi));
    }
}

void clean_and_print(vectorD ptsx, vectorD ptsy){

    cout<<"plt.plot([";
    for(int i=0; i<ptsx.size(); i++) cout<<ptsx[i]<<(i==ptsx.size()-1?']':',');
    cout<<", ["<<endl;
    for(int i=0; i<ptsy.size(); i++) cout<<ptsy[i]<<(i==ptsx.size()-1?']':',');
    cout<<", 'r')"<<endl;
}

int main()
{
    

    vectorD state, pts_x, pts_y;
    get_state(state, pts_x, pts_y);

    double x = state[0], y = state[1], psi = state[2], v= state[3];

    if (path_pts_wrt_origin)
        change_reference_system(pts_x, pts_y, x, y, psi);

    //coeff = polyfit(pts_x, pts_y);
    //fitIt(pts_x, pts_y, 4, coeff);
    coeff= {0, 0, 1, 1};
    
    //Anyway, cte & epsi seem not to be used for the first (current) state
    double cte = polyeval(coeff, x)- y;
    double psi_dst = atan(coeff[1] + (2 * coeff[2] * x) + (3 * coeff[2] * x * x));
    double epsi= psi- psi_dst;

    //x, y, and psi are zero in the new reference system
    //State st(0.0, 0.0, 0.0, 10, cte, epsi);
    State st(x, y, psi, v, cte, epsi);

    vectorD ptsx, ptsy;
    ptsx.push_back(x);
    ptsy.push_back(y);

    for(int i=0; i<50; i++)
    {

        Optimizer o(st);

        Path p = o.optimize();

        st = p.states[1];
        //st.print_state_report();
        ptsx.push_back(st.x);
        ptsy.push_back(st.y);
        
        
    }
    clean_and_print(ptsx, ptsy);
}