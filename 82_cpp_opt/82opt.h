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
        static constexpr double g = 9.8066;

        // Interface methods, do not change
        vector_double::size_type get_nec() const;
        vector_double::size_type get_nic() const;
        vector_double fitness(const vector_double &x) const;
        std::pair<vector_double, vector_double> get_bounds() const;
    private: 
        // Helper methods that other classes shouldn't be using go here
        bool invalid_ret(vector_double &x) const;
};

#endif