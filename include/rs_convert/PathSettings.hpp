#pragma once

#include "rs_convert/PathWords.hpp"
#include <vector>

// C# TO C++ CONVERTER NOTE: Forward class declarations:
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class PathSegmentLengths;
}
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class SegmentSettings;
}



namespace PathfindingForVehicles::ReedsSheppPaths
{
    // Calculate the settings a car need to complete a given reeds-shepp path
    class PathSettings final
    {
    public:
        static std::vector<SegmentSettings> GetSettings(PathWords word, PathSegmentLengths pathLengths);

        // Time-flip transform method from the report, which interchanges + and -
        // l+ r- s- l- -> l- r+ s+ l+
    private:
        static std::vector<SegmentSettings> TimeFlip(std::vector<SegmentSettings> pathSettings);

        // Reflect transform fromt the report, which interchanges r and l
        // l+ r- s- l- -> r+ l- s- r-
        static std::vector<SegmentSettings> Reflect(std::vector<SegmentSettings> pathSettings);

        // Backwards transform, which follow the path in reverse order,
        // but with timeflip so the individual segments are transversed in the same direction

        //
        // Settings for individual paths
        //

        // 8.1: CSC, same turn
    public:
        static std::vector<SegmentSettings> getLfSfLfPath(PathSegmentLengths pathLength);

        // 8.2: CSC, different turn
        static std::vector<SegmentSettings> getLfSfRfPath(PathSegmentLengths pathLength);

        // 8.3: C|C|C
        static std::vector<SegmentSettings> getLfRbLfPath(PathSegmentLengths pathLength);

        // 8.4: C|CC
        static std::vector<SegmentSettings> getLfRbLbPath(PathSegmentLengths pathLength);

        // 8.4: CC|C
        static std::vector<SegmentSettings> getLfRfLbPath(PathSegmentLengths pathLength);

        // 8.7: CCu|CuC
        static std::vector<SegmentSettings> getLfRufLubRbPath(PathSegmentLengths pathLength);

        // 8.8: C|CuCu|C
        static std::vector<SegmentSettings> getLfRubLubRfPath(PathSegmentLengths pathLength);

        // 8.9: C|C(pi/2)SC, same turn
        static std::vector<SegmentSettings> getLfRbpi2SbLbPath(PathSegmentLengths pathLength);

        // 8.10: C|C(pi/2)SC, different turn
        static std::vector<SegmentSettings> getLfRbpi2SbRbPath(PathSegmentLengths pathLength);

        // 8.9 (reversed): CSC(pi/2)|C, same turn
        static std::vector<SegmentSettings> getLfSfRfpi2LbPath(PathSegmentLengths pathLength);

        // 8.10 (reversed): CSC(pi/2)|C, different turn
        static std::vector<SegmentSettings> getLfSfLfpi2RbPath(PathSegmentLengths pathLength);

        // 8.11: C|C(pi/2)SC(pi/2)|C
        static std::vector<SegmentSettings> getLfRbpi2SbLbpi2RfPath(PathSegmentLengths pathLength);
    };
}
