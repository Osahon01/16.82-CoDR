#include "82opt.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <initializer_list>
#include <utility>


using namespace pagmo;

// double - minimum cruise velocity
// double - minimum cruise range
problem_fvd make_pfvd_obj(double v, double r){
    problem_fvd x;
    x.min_V_cruise = v;
    x.min_Range = r;
    return x;
}


vector_double::size_type problem_fvd::get_nec() const{
    return 2;
}
vector_double::size_type problem_fvd::get_nic() const{
    return 4;
}

std::pair<vector_double, vector_double> problem_fvd::get_bounds() const{
    vector_double lb = {0.5,0.,10.,3.,0.,5.,0.,30.,0.,0.};
    vector_double ub = {2.5,2.,500.,15.,50000.,20.,1.,60.,20.,1.4};
    std::pair<vector_double, vector_double> ret(lb,ub);
    return ret;
}

// Calculates the fitness function and associated constraints,
// in the form {fitness, eq..., ineq...}
// All units in radians
vector_double problem_fvd::fitness(const vector_double &x, bool eval) const{


    try{

        // This is a helpful block of code. Put it immediately after a value prone to mishaps
        // to immediately skip all downstream computation and return a high penalty instead
        
        // if(std::isnan(u_Toa)){
        //     vector_double ret(1+static_cast<int>(get_nec())+static_cast<int>(get_nic()),1e+6);
        //     return ret;
        // }

        auto m_fudge = x[0];// Fudge factor to ensure the mass estimate going in is correlated to the final mass
        auto log_f = x[1];// Equal to -ln(1-f)
        auto S = x[2];
        auto AR = x[3];
        auto T_TO = x[4];
        auto alpha_TO = x[5];
        auto h_d = x[6]; // Span-averaged disk height
        auto delta_F_TO = x[7];
        auto CL_TO = x[8];
        auto CL_cruise = x[9];


        double S_wet = 2.*S*(1.2) + 69.;// The 69 is a rough estimate for the fusl area of the EL9
        double m_struc = 1000. * (S_wet / 140.);
        double m = 1.5 * m_struc * std::exp(log_f) * m_fudge;
        double con_max_mass = m - 8168.;
        // Calculate generally useful stuff, avoid recalculation
        double b = std::pow(AR*S,0.5);
        double c = b/AR;
        double A_d = b*h_d;
        // Calculate TO velocity and required jet velocity to make thrust, and finally delta CJ
        double V_TO = std::pow((2*m*g)/(S*CL_TO*rho_0k),0.5);
        double Tprimec_TO = T_TO / (0.5*rho_0k*V_TO*V_TO*S);
        double con_max_Tprimec_TO = (Tprimec_TO - 3.5)/3.5;
        double Vj_TO = std::pow((2*T_TO)/(rho_0k*A_d)+V_TO*V_TO,0.5);
        double delta_CJ_TO = ((2*h_d)/c)*((Vj_TO/V_TO)*(Vj_TO/V_TO)-1);
        // Use models to get the blown CL and enforce closure via residual
        double CL_TO_surr = blown_CL_surr(alpha_TO,delta_CJ_TO,h_d / S, delta_F_TO);
        double resid_CL_TO = (CL_TO_surr - CL_TO)/CL_TO;
        double eta_fr_TO = (2*V_TO)/(V_TO+Vj_TO);
        double P_TO_shaft = (T_TO*V_TO)/eta_fr_TO;

        double x_TO = (m*m*g)/(T_TO*S*rho_0k*CL_TO);

        double q_cruise = (m*g)/(S*CL_cruise);
        double V_cruise = std::pow((2*q_cruise)/rho_10k,0.5);
        double D_cruise = S_wet*Cd_v*q_cruise + ((CL_cruise*CL_cruise)/(M_PI*spaneff*AR))*q_cruise*S;
        double LD_cruise = (m*g)/(D_cruise);
        double Vj_cruise = std::pow((2*D_cruise)/(rho_10k*A_d)+V_cruise*V_cruise,0.5);
        double eta_fr_cruise = (2*V_cruise)/(Vj_cruise+V_cruise);
        double P_cruise_shaft = (D_cruise*V_cruise) / eta_fr_cruise;
        double Range = ((h_avgas*eta_gen*eta_fr_cruise*0.95)/g)*LD_cruise*log_f;

        double con_min_range = (min_Range - Range)/min_Range;
        double max_Power = std::max(P_TO_shaft,P_cruise_shaft);

        double m_prop = std::max(P_TO_shaft,P_cruise_shaft) / P_spec_prop;
        double m_gen = max_Power / P_spec_gen;
        double m_calc_nofuel = m_struc+m_prop+m_pax+m_gen;
        double resid_mass = (m_calc_nofuel - m*std::exp(-log_f))/std::max(m,m_calc_nofuel);
        double con_min_V_cruise = (min_V_cruise - V_cruise)/min_V_cruise;

        // Package objective and constraint values
        vector_double ret{};
        if (eval){
            ret = {Range,
                    x_TO,
                    m,
                    1-std::exp(-log_f),
                    x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9],
                    D_cruise / (q_cruise*S),
                    resid_CL_TO,
                    resid_mass,
                    con_min_range,
                    con_max_Tprimec_TO,
                    con_min_V_cruise,
                    con_max_mass,
                    P_TO_shaft,
                    P_cruise_shaft,};
        }else{
            ret = {x_TO/40.,
                                resid_CL_TO,
                                resid_mass,
                                con_min_range,
                                con_max_Tprimec_TO,
                                con_min_V_cruise,
                                con_max_mass};
        }
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

// Surrogate model for CL in the blown lift condition
double problem_fvd::blown_CL_surr(double alpha, double Delta_CJ, double hd_S, double delta_F) const{
    auto delta_J = delta_J_surr(hd_S, delta_F);
    double CL = X1 + X2*delta_J*X3*alpha
                + std::pow(Delta_CJ,0.5)*(X4*delta_J+X5*alpha*X6)
                + Delta_CJ*(X7*delta_J+X8*alpha*X9);
    return CL;
}

// Surrogate model for flap efficiency as a function of flap deflection and 
// nondimensional jet wash height
double problem_fvd::delta_J_surr(double hd_S, double delta_F) const{

    double eta_40 = 1./(f1_40*std::exp(f2_40*(hd_S))+f3_40*std::exp(f4_40*(hd_S)));
    double eta_60 = 1./(f1_60*std::exp(f2_60*(hd_S))+f3_60*std::exp(f4_60*(hd_S)));
    double k_b = delta_F/20. - 2.;
    if (delta_F<40){
        return eta_40*delta_F;
    }
    if (delta_F>60){
        return eta_60*delta_F;
    }
    // Simple linear interpolation since the one in the paper is ass
    return (eta_40 + k_b*(eta_60 - eta_40))*delta_F;
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
