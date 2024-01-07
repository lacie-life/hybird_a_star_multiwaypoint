#pragma once

namespace PathfindingForVehicles::ReedsSheppPaths
{
    // Each path consists of 3 segments calles tuv that have different lengths
    // There may be more segment, but they have constant length
    class PathSegmentLengths
    {
    public:
        double t = 0, u = 0, v = 0;

        PathSegmentLengths(double t, double u, double v);

        PathSegmentLengths() = default;
    };
}
