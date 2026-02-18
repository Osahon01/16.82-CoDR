#include "82opt.h"

#include <iostream>
#include <chrono>
#include <fstream>  // For file output

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/algorithms/cstrs_self_adaptive.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/population.hpp>
#include <limits>

int main() {

    auto pfvd_obj = make_pfvd_obj(125.,2420000.);

    pagmo::vector_double x1 = {8602.03,0.762815,99.6058,13.3569,34166.2,14.9924,0.964673,51.4516,8.63691,0.120406};
    pfvd_obj.fitness(x1);

    // Design variable headers
    const char* headers[] = {
        "min_V_cruise",
        "best_takeoff",
        "m",
        "f",
        "S",
        "AR",
        "T_TO",
        "alpha_TO",
        "h_d",
        "delta_F_TO",
        "CL_TO",
        "CL_cruise"
    };

    // Open CSV file
    std::ofstream outfile("results.csv");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open output file.\n";
        return 1;
    }

    // Write headers
    for (int i = 0; i < 12; i++) {
        outfile << headers[i];
        if (i < 11) outfile << ",";
        else outfile << "\n";
    }

    double min_V_cruises[] = {70., 85., 100., 115., 130., 145., 160.};
    double min_Ranges[] = {2420e+3*0.5, 
                            2420e+3*1., 
                            2420e+3*1.5, 
                            2420e+3*2., 
                            2420e+3*2.5, 
                            2420e+3*3., 
                        };

    for (auto min_Param : min_V_cruises) {
        pfvd_obj.min_V_cruise = min_Param;
        pagmo::problem pfvd{pfvd_obj};
        algorithm inner_algo{sade(300)};
        algorithm algo{
            cstrs_self_adaptive(
                256u,
                inner_algo
            )
        };
        algo.set_seed(42070);
        archipelago archi(12u, algo, pfvd, 200u);
        archi.evolve(1);
        archi.wait_check();

        auto best_takeoff_dist = std::numeric_limits<double>::infinity();
        pagmo::vector_double best_plane;

        for (const auto &isl : archi) {
            auto curr_dist = isl.get_population().champion_f()[0];
            if (curr_dist < best_takeoff_dist) {
                best_plane = isl.get_population().champion_x();
                best_takeoff_dist = curr_dist;
            }
        }
        // Write results to CSV
        outfile << min_Param << "," << best_takeoff_dist << ",";
        for (int i = 0; i < best_plane.size(); i++) {
            outfile << best_plane[i];
            if (i < best_plane.size() - 1) outfile << ",";
        }
        outfile << "\n";

        // Optional: also print to console
        std::cout << "Min cruise velocity : " << min_Param << '\n';
        std::cout << "Best takeoff : " << best_takeoff_dist << '\n';
        std::cout << "Best design : {";
        for (int i = 0; i < best_plane.size(); i++) {
            std::cout << best_plane[i];
            if (i < best_plane.size() - 1){
                std::cout << ",";
            }
        }
        std::cout << "}\n";
        std::cout << "CL_TO residual : " << pfvd_obj.fitness(best_plane)[1] << '\n';
        std::cout << "Mass residual : " << pfvd_obj.fitness(best_plane)[2] << '\n';
        std::cout << "=================================================================\n";
    }

    outfile.close();
    std::cout << "Results written to results.csv\n";

    return 0;
}