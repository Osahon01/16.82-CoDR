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
    // auto omega = x[0]; // rad/s - shaft speed
    // auto u_i = x[1];   // m/s   - compressor inlet velocity
    // auto T_4 = x[2];   // K     - combustor exit total temp
    // auto R_Cih = x[3]; // m     - compressor inlet hub radius
    // auto R_Cit = x[4]; // m     - compressor inlet tip radius
    // auto A_Co = x[5];  // m^2   - compressor outlet area
    // auto R_Com = x[6]; // m     - compressor outlet meanline radius
    // auto D_T_C = x[7]; // K     - compressor total temperature change
    // auto R_Tih = x[8]; // m     - turbine inlet hub radius
    // auto R_Tit = x[9]; // m     - Turbine inlet tip radius
    // auto A_To = x[10]; // m^2   - Turbine outlet area
    // auto R_Tom = x[11];// m     - Turbine exit meanline radius
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


// Checks if anything in a vector_double is inf or nan
bool problem_fvd::invalid_ret(vector_double &x) const{
    for (double xi : x){
        if(std::isinf(xi) || std::isnan(xi)){
            return true;
        }
    }
    return false;
}
