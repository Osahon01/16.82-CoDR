#include "82opt.h"

#include <iostream>
#include <chrono>
#include <fstream>  // For file output

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/population.hpp>
#include <limits>

int main() {

    auto pfvd_obj = make_pfvd_obj(125.);
    pfvd_obj.min_V_cruise = 125.;

    pagmo::vector_double x1 = {8260.27,0.597822,105.238,6.73066,25193.1,11.0429,0.716945,53.9623,10.8699,0.105031};
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

    double min_V_cruises[] = {50., 75., 100., 125., 150., 175., 200.};

    for (auto min_V_cruise : min_V_cruises) {
        pfvd_obj.min_V_cruise = min_V_cruise;
        pagmo::problem pfvd{pfvd_obj};

        algorithm algo{gaco(10000, 63, 1, 0, 0.01, 1U, 7, 10000)};
        algo.set_seed(42069);
        archipelago archi(32u, algo, pfvd, 10000u);
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
        outfile << min_V_cruise << "," << best_takeoff_dist << ",";
        for (int i = 0; i < best_plane.size(); i++) {
            outfile << best_plane[i];
            if (i < best_plane.size() - 1) outfile << ",";
        }
        outfile << "\n";

        // Optional: also print to console
        std::cout << "Min v cruise : " << min_V_cruise << '\n';
        std::cout << "Best takeoff : " << best_takeoff_dist << '\n';
        std::cout << "=================================================================\n";
    }

    outfile.close();
    std::cout << "Results written to results.csv\n";

    return 0;
}