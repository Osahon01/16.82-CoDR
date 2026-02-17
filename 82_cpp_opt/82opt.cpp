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

    // Put stuff here which only is for debugging / output inspection
    // Basically anything that shouldn't be slowing down the main optimizer loop    pagmo::vector_double x1 = {4648.1, 136.75, 1099.75, 0.0150023, 0.0507001, 0.00750535, 0.0799953, 73.5045, 0.0511641, 0.0774573, 0.00759833, 0.0576068, };
    
    pagmo::vector_double x1 = {4823.46, 0.532962, 38.7987, 13.5026, 28644.5, -1.15711, 0.601813, 51.8072, 5.21805, 0.172021, };
    pfvd_obj.fitness(x1);

    pagmo::problem pfvd{pfvd_obj};
    std::cout << pfvd;
    algorithm algo{gaco(10000,63,1,0,0.01,1U,7,10000)};    
    archipelago archi(32u, algo, pfvd, 10000u);
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