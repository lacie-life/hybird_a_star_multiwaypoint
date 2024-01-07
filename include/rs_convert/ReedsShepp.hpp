#pragma once

#include "PathWords.hpp"
#include "global/wrap2pi.hpp"

#include <vector>
#include <cmath>
#include <limits>


// C# TO C++ CONVERTER NOTE: Forward class declarations:
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class RSCar;
}
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class PathSegmentLengths;
}
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class CVector3d;
}


namespace PathfindingForVehicles::ReedsSheppPaths
{
    // Generates Reeds-Shepp paths
    // Some code from https://github.com/mattbradley/AutonomousCar/tree/master/AutonomousCar/AutonomousCar/PathFinding/ReedsShepp
    class ReedsShepp final
    {
        /// <summary>
        /// Generate the shortest Reeds-Shepp path
        /// </summary>
        /// <param name="startPos">The position of the car where the path starts</param>
        /// <param name="startHeading">The heading of the car where the path starts [rad]</param>
        /// <param name="endPos">The position of the car where the path ends</param>
        /// <param name="endHeading">The heading of the car where the path ends [rad]</param>
        /// <param name="turningRadius">The truning radius of the car [m]</param>
        /// <param name="wpDistance">The distance between the waypoints that make up the final path</param>
        /// <param name="generateOneWp">Should we generate just 1 waypoint and not all</param>
        /// <returns></returns>
    public:
        static std::vector<RSCar> GetShortestPath(CVector3d &startPos, double startHeading, CVector3d &endPos, double endHeading, double turningRadius, double wpDistance, bool generateOneWp);

        // Same as above but we just want the shortest distance
        static double GetShortestDistance(CVector3d &startPos, double startHeading, CVector3d &endPos, double endHeading, double turningRadius);

        // Loop through all paths and find the shortest one (if one can be found)
    private:
        static double FindShortestPathLength(RSCar carEndMod, PathSegmentLengths &bestPathLengths, PathWords &bestWord);

        // The formulas assume we move from(0, 0, 0) to (x, y, theta), and that the turning radius is 1
        // This means we have to move and rotate the goal's position and heading as if the start's position and heading had been at (0,0,0)
        // But we are using Unity, so the rotation of the start car has to be along the x-axis and not z-axis which is Unity's zero-rotation
    public:
        static RSCar NormalizeGoalCar(RSCar carStart, RSCar carEnd, double turningRadius);

        // Find the shortest Reeds-Shepp path and generate waypoints to follow that path
    private:
        static std::vector<RSCar> FindShortestPath(RSCar carStart, RSCar carEnd, RSCar carEndMod, double wpDistance, double turningRadius, bool generateOneWp);

        // Add waypoints to a given path
        static std::vector<RSCar> AddWaypoints(PathWords word, PathSegmentLengths pathSegmentLengths, RSCar carStart, RSCar carEnd, double wpDistance, double turningRadius, bool generateOneWp);
    };
}
