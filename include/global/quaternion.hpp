#pragma once
#include "cvector_3d.hpp"
#include <Eigen/Eigen>

namespace PathfindingForVehicles::ReedsSheppPaths
{
    class Quaternion {
    private:
        Eigen::Quaternion<double> _q;

    public:
        static Quaternion Euler(double x, double y, double z);

        Quaternion(const Eigen::Quaternion<double> q);
        Quaternion &operator = (const Quaternion &other);
        CVector3d operator * (const CVector3d &vec);
    };
}