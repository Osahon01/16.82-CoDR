#ifndef FVD_FITNESS
#define FVD_FITNESS

// #define FVD_EVAL

#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>

using namespace pagmo;
// "flight vehicle design"
struct problem_fvd{
    public:
        // Non-varying parameters
        // Define things like environment parameters and compenent efficiencies here
        // These need to be known at compile time and cannot be changed while running
        static constexpr double T0 = 288.15;      // Sea level standard temperature (K)
        static constexpr double P0 = 101325.0;    // Sea level standard pressure (Pa)
        static constexpr double R  = 287.053;     // Gas constant for dry air (J/(kg·K))
        static constexpr double g  = 9.80665;     // Gravity (m/s^2)
        static constexpr double L  = 0.0065;      // Temperature lapse rate (K/m)

        // Interface methods, do not change
        vector_double::size_type get_nec() const;
        vector_double::size_type get_nic() const;
        vector_double fitness(const vector_double &x) const;
        std::pair<vector_double, vector_double> get_bounds() const;
    private: 
        // Helper methods that other classes shouldn't be using go here
        bool invalid_ret(vector_double &x) const;
        std::tuple<double, double, double> standardAtmosphere(double altitudeMeters) const;
};

#endif