#include "82opt.h"

#include <iostream>
#include <chrono>

#include <pagmo/algorithm.hpp>

#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/population.hpp>

int main(){

    problem_fvd pfvd_obj;
    #ifdef FVD_EVAL
    // Put stuff here which only is for debugging / output inspection
    // Basically anything that shouldn't be slowing down the main optimizer loop
    #endif
    pagmo::problem pfvd{pfvd_obj};
    std::cout << pfvd;
    algorithm algo{gaco(10000,63,1,0,0.01,100,7,10000)};    
    archipelago archi(32u, algo, pfvd, 6000u);
    archi.evolve(1);
    archi.wait_check();

    for (const auto &isl : archi) {
        std::cout << isl.get_population().champion_f()[0] << '\n';
        pagmo::vector_double ch_x = isl.get_population().champion_x();
        std::cout << "{";
        for(int i =0; i<ch_x.size(); i++){
            std::cout << ch_x[i] << ", ";
        }
        std::cout << "}\n";
    }
    return 0;
}