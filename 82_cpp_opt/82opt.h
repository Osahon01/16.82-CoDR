#ifndef FVD_FITNESS
#define FVD_FITNESS


#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>

using namespace pagmo;
// "flight vehicle design"
struct problem_fvd{
    public:
        // Non-varying parameters
        // Define things like environment parameters and compenent efficiencies here
        double min_V_cruise;
        double min_Range;
        static constexpr double g = 9.8066;
        static constexpr double rho_0k = 1.2;

        static constexpr double rho_10k = 0.9;

        static constexpr double Cd_v = 0.01;
        // Blown aero coefficients surrogate
        static constexpr double X1 = 0.1856;
        static constexpr double X2 = 0.0334;
        static constexpr double X3 = 0.0667;
        static constexpr double X4 = 0.0121;
        static constexpr double X5 = -0.085;
        static constexpr double X6 = 0.0426;
        static constexpr double X7 = 0.0140;
        static constexpr double X8 = 0.0226;
        static constexpr double X9 = 0.1407;

        static constexpr double f1_40 = 0.9790;
        static constexpr double f2_40 = -0.0782;
        static constexpr double f3_40 = 0.0076;
        static constexpr double f4_40 = 6.006;

        static constexpr double f1_60 = 1.0348;
        static constexpr double f2_60 = 0.688;
        static constexpr double f3_60 = 0.0001;
        static constexpr double f4_60 = 12.446;


        static constexpr double spaneff = 0.95;

        static constexpr double eta_gen = 0.2;
        static constexpr double P_spec_prop = 6000.;
        static constexpr double P_spec_gen = 6300.;

        static constexpr double m_pax = 900.;
        static constexpr double h_avgas = 43.5e+6;



        // Interface methods, do not change
        vector_double::size_type get_nec() const;
        vector_double::size_type get_nic() const;
        vector_double fitness(const vector_double &x) const;
        std::pair<vector_double, vector_double> get_bounds() const;
        // TODO move these to private after testing for units
    private: 
        // Helper methods that other classes shouldn't be using go here
        bool invalid_ret(vector_double &x) const;
        double blown_CL_surr(double alpha, double Delta_CJ,double hd_S, double delta_F) const;
        double delta_J_surr(double hd_S, double delta_F) const;
};

problem_fvd make_pfvd_obj(double v, double r);

#endif