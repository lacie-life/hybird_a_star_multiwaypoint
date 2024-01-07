#include <iostream>
#include <fstream>
#include <math.h>
#include <memory>

#include <eigen3/Eigen/Dense>

#include "thirdparty/matplotlibcpp.h"
#include "global/cvector_3d.hpp"

#include "rs_convert/ReedsShepp.hpp"
#include "rs_convert/PathSegmentLengths.hpp"
#include "rs_convert/RSCar.hpp"

namespace plt = matplotlibcpp;

using namespace std;


using namespace PathfindingForVehicles::ReedsSheppPaths;

int32_t main(const int32_t argc, char **const argv)
{
    std::cout << "test main" << std::endl;

    Eigen::Vector3d start_pos1(1.0, 2.0, 3.0);
    Eigen::Vector3d end_pos1(23.0, 45.0, 6.0);

    std::cout << start_pos1.x() << "," << start_pos1.y() << std::endl;

    double pi = M_PI;
    double pi2 = 2 * pi;
    double pi_2 = pi * 0.5;

    CVector3d startPos(10.0, 50.0, 0.0); // (x, y, z=0)
    CVector3d endPos(150.0, 50.0, 0.0);

    double startRot = -45 * (M_PI / 180);
    double endRot = 150 * (M_PI / 180);

    // Get the shortest path
    auto shortestPath = ReedsShepp::GetShortestPath(startPos, startRot, endPos, endRot, 12.0, 1.0, false);

    std::fstream fs("data2.csv", std::ios::out);
    fs << "x,y" << std::endl;
    for (const RSCar &s : shortestPath)
    {
        std::cout << s.getX() << "," << s.getY() << std::endl;
        fs << s.getX() << "," << s.getY() << std::endl;
    }

    return 0;
}
