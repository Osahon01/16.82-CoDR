#include "82opt.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <initializer_list>
#include <utility>


using namespace pagmo;

vector_double::size_type problem_fvd::get_nec() const{
    return 0;
}
vector_double::size_type problem_fvd::get_nic() const{
    return 0;
}

std::pair<vector_double, vector_double> problem_fvd::get_bounds() const{
    vector_double lb = {};
    vector_double ub = {};
    std::pair<vector_double, vector_double> ret(lb,ub);
    return ret;
}

// Calculates the fitness function and associated constraints,
// in the form {fitness, eq..., ineq...}
vector_double problem_fvd::fitness(const vector_double &x) const{
    
    try{

        // This is a helpful block of code. Put it immediately after a value prone to mishaps
        // to immediately skip all downstream computation and return a high penalty instead
        
        // if(std::isnan(u_Toa)){
        //     vector_double ret(1+static_cast<int>(get_nec())+static_cast<int>(get_nic()),1e+6);
        //     return ret;
        // }

        auto m = x[0];
        auto f = x[1];
        auto S = x[2];
        auto AR = x[3];
        auto T_TO = x[4];
        auto alpha_TO = x[5];
        auto h_d = x[6];
        auto delta_F = x[7];

        // Package objective and constraint values
        vector_double ret = {};
        // A physically impossible situation will usually result in a bunch of NaNs,
        // and bad inputs might give Inf due to division by zero.
        // If this happens, return a big penalty
        if (invalid_ret(ret)){
            ret = vector_double(1+static_cast<int>(get_nec())+static_cast<int>(get_nic()),1e+6);
        }
        return ret;
    }catch(...){
        // If anything goes wrong, return a high penalty
        // Should change this so it doesn't fail silently
        vector_double ret(1+static_cast<int>(get_nec())+static_cast<int>(get_nic()),1e+6);
        return ret;
    }
}

double problem_fvd::blown_Cl_surr(double alpha, double Delta_CJ, double Ad_S, double delta_F) const{
    auto delta_J = delta_J_surr(Ad_S, delta_F);
    double CL = X1 + X2*delta_J*X3*alpha
                + std::pow(Delta_CJ,0.5)*(X4*delta_J+X5*alpha*X6)
                + Delta_CJ*(X7*delta_J+X8*alpha*X9);
    return CL;
}

double problem_fvd::delta_J_surr(double Ad_S, double delta_F) const{
    //Surrogate model for flap efficiency as a function of flap deflection and 
    //Nondimensional jet wash height
    double eta_40 = 1./(f1_40*std::exp(f2_40*(Ad_S))+f3_40*std::exp(f4_40*(Ad_S)));
    double eta_60 = 1./(f1_60*std::exp(f2_60*(Ad_S))+f3_60*std::exp(f4_60*(Ad_S)));
    double k_b = delta_F/20. - 2.;
    if (delta_F<40){
        return eta_40;
    }
    if (delta_F>60){
        return eta_60;
    }
    return eta_40 + k_b*(eta_60 - eta_40);
}

// Checks if anything in a vector_double is inf or nan
bool problem_fvd::invalid_ret(vector_double &x) const{
    for (double xi : x){
        if(std::isinf(xi) || std::isnan(xi)){
            return true;
        }
    }
    return false;
}
