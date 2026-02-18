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

void save_to_csv(const std::vector<std::vector<double>> &data, const std::vector<std::string> &headers, const std::string &filename) {
    /*
    Given an m,n array of doubles, a vector of headers, and a filename, writes the data to a csv at the passed filename. 
    Checks that the length of the headers is n, and that all rows are of length n as well
    */
    
    size_t num_cols = headers.size();
    
    // Validate that all rows have the correct number of columns
    for (const auto& row : data) {
        if (row.size() != num_cols) {
            throw std::invalid_argument("All rows must have " + std::to_string(num_cols) + 
                                      " columns, but found row with " + std::to_string(row.size()) + " columns");
        }
    }
    
    // Open file for writing
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    
    // Write headers
    for (size_t i = 0; i < headers.size(); ++i) {
        file << headers[i];
        if (i < headers.size() - 1) {
            file << ",";
        }
    }
    file << "\n";
    
    // Write data rows
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }
    
    file.close();
}

int main() {

    auto pfvd_obj = make_pfvd_obj(125.,2420000.);

    pagmo::vector_double x1 = {8602.03,0.762815,99.6058,13.3569,34166.2,14.9924,0.964673,51.4516,8.63691,0.120406};
    pfvd_obj.fitness(x1);

    // Design variable headers
    std::vector<std::string> headers = {
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
        "CL_cruise",
        "CL residual",
        "Mass residual",
        "Range constraint",
        "Tprime constraint",
        "Cruise velocity constraint",
        "Max mass constraint"
    };

    std::vector<std::vector<double>> data{};


    double min_V_cruises[] = {60.,65.,70.,75.,80.,85.,90.,95.,100.,105.,110.,115.,120.,125.,130.,135.,140.,145.,150.,155.,160.,};
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
                64u,
                inner_algo
            )
        };
        algo.set_seed(42069);
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
        data.push_back(pfvd_obj.fitness(best_plane, true));
    }
    save_to_csv(data,headers,"results.csv");
    std::cout << "Results written to results.csv\n";

    return 0;
}