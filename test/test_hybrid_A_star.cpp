//
// Created by lacie on 15/06/2023.
//

#include <iostream>
#include <chrono>

#include "hybrid_a_star/HybridAStarFlow.h"

using namespace std;

int main(const int argc, char **const argv)
{
    std::cout << "test main" << std::endl;

    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <map_file> <config_file>" << std::endl;
        return 1;
    }

    HybridAStarFlow solver = HybridAStarFlow(argv[1], argv[2]);

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Run solver
    bool result = solver.Run();

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout<<"Time: " << time_used.count() << " seconds" << std::endl;

    if(result)
    {
        std::cout << "Success" << std::endl;
    }
    else
    {
        std::cout << "Something wrong" << std::endl;
    }

    return 0;
}
