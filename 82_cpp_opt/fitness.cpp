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
