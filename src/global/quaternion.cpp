#include "global/quaternion.hpp"

namespace PathfindingForVehicles::ReedsSheppPaths
{

    Quaternion::Quaternion(const Eigen::Quaternion<double> q)
    {
        _q = q;
    }

    Quaternion Quaternion::Euler(double x, double y, double z)
    {
        Eigen::Quaternion<double> q =
            Eigen::AngleAxis<double>(x, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis<double>(y, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(z, Eigen::Vector3d::UnitZ());
        return Quaternion(q);
    }

    CVector3d Quaternion::operator*(const CVector3d &vec)
    {
        auto rot = _q * Eigen::Vector3d(vec.x, vec.y, vec.z);
        return CVector3d(rot.x(), rot.y(), rot.z());
    }

}