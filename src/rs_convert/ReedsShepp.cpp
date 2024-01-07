#include "rs_convert/ReedsShepp.hpp"
#include "rs_convert/RSCar.hpp"
#include "rs_convert/PathSegmentLengths.hpp"
#include "rs_convert/PathLengthMath.hpp"
#include "rs_convert/PathSettings.hpp"
#include "rs_convert/SegmentSettings.hpp"
#include "global/quaternion.hpp"

//#include "lib/matplotlibcpp.h"
//namespace plt = matplotlibcpp;

namespace PathfindingForVehicles::ReedsSheppPaths
{

    std::vector<RSCar> ReedsShepp::GetShortestPath(CVector3d &startPos, double startHeading, CVector3d &endPos, double endHeading, double turningRadius, double wpDistance, bool generateOneWp)
    {
        RSCar carStart(startPos, startHeading);
        RSCar carEnd(endPos, endHeading);

        // The formulas assume we move from(0, 0, 0) to(x, y, theta), and that the turning radius is 1
        // This means we have to move and rotate the goal's position and heading as if the start's position and heading had been at (0,0,0)
        RSCar carEndMod = NormalizeGoalCar(carStart, carEnd, turningRadius);

        std::vector<RSCar> shortestPath = FindShortestPath(carStart, carEnd, carEndMod, wpDistance, turningRadius, generateOneWp);

        return shortestPath;
    }

    double ReedsShepp::GetShortestDistance(CVector3d &startPos, double startHeading, CVector3d &endPos, double endHeading, double turningRadius)
    {
        RSCar carStart(startPos, startHeading);
        RSCar carEnd(endPos, endHeading);

        // The formulas assume we move from(0, 0, 0) to(x, y, theta), and that the turning radius is 1
        // This means we have to move and rotate the goal's position and heading as if the start's position and heading had been at (0,0,0)
        RSCar carEndMod = NormalizeGoalCar(carStart, carEnd, turningRadius);

        PathSegmentLengths bestPathLengths;
        PathWords bestWord;

        double shortestPathLength = FindShortestPathLength(carEndMod, bestPathLengths, bestWord);

        // No path could be found
        if (shortestPathLength == std::numeric_limits<double>::infinity())
        {
            std::cout << "Cant find a Reeds-Shepp path" << std::endl;
            return shortestPathLength;
        }

        // Convert back to the actual diistance by using the turning radius
        shortestPathLength *= turningRadius;

        return shortestPathLength;
    }

    double ReedsShepp::FindShortestPathLength(RSCar carEndMod, PathSegmentLengths &bestPathLengths, PathWords &bestWord)
    {
        // How many paths are we going to check
        int numPathWords = 48;

        // Better than using double.MaxValue because we can use double.IsPositiveInfinity(bestPathLength) to test if its infinity
        double shortestPathLength = std::numeric_limits<double>::max();

        bestWord = static_cast<PathWords>(0);

        // Will keep track of the length of the best path
        // Some Reeds-Shepp segments have 5 lengths, but 2 of those are known, so we only need 3 to find the shortest path
        bestPathLengths = PathSegmentLengths(0.0, 0.0, 0.0);

        // Loop through all paths that are enums to find the shortest
        for (int w = 0; w < numPathWords; w++)
        {
            PathWords word = static_cast<PathWords>(w);

            PathSegmentLengths pathSegmentLengths;

            double pathLength = PathLengthMath::GetLength(carEndMod, word, pathSegmentLengths);


            if (pathLength < shortestPathLength)
            {
                shortestPathLength = pathLength;
                bestWord = word;
                bestPathLengths = pathSegmentLengths;

                std::cout << "pathLength " << pathLength << std::endl;
            }
        }

        return shortestPathLength;
    }

    RSCar ReedsShepp::NormalizeGoalCar(RSCar carStart, RSCar carEnd, double turningRadius)
    {
        // // // Change the position and rotation of the goal car
        // //CVector3d posDiff = carEnd.pos - carStart.pos;

        // // // Turning radius is 1
        // //posDiff = posDiff / turningRadius;


        // CVector3d posDiff((carEnd.pos.x - carStart.pos.x) / turningRadius, (carEnd.pos.y - carStart.pos.y) / turningRadius, 0.0);

        // // Rotate the vector between the cars
        // // Add 90 degrees because of unitys coordinate system
        // //CVector3d newEndPos = Quaternion::Euler(0.0, -carStart->getHeadingInDegrees() + 90.0, 0.0) * posDiff;

        // posDiff.RotateZ(-carStart.getHeadingInDegrees());
        // std::cout << "posDiff.x " << posDiff.x << " posDiff.y " << posDiff.x << " " << posDiff.z << std::endl;

        // CVector3d newEndPos = Quaternion::Euler(0.0, 0.0, -carStart.getHeadingInDegrees()) * posDiff;
        // std::cout << "newEndPos.x " << newEndPos.x << " newEndPos.y " << newEndPos.x << " " << newEndPos.z << std::endl;

        // // Unitys coordinate is not the same as the one used in the pdf so we have to make som strange translations
        // double headingDiff = PathLengthMath::WrapAngleInRadians((2.0 * M_PI) - (carEnd.getHeadingInRad() - carStart.getHeadingInRad()));

        // // get the angle displacement
        // //double headingDiff = mrpt::math::wrapToPi<double>(carEnd.getHeadingInRad() - carStart.getHeadingInRad());
        // std::cout << "carStart.getHeadingInRad() " << carStart.getHeadingInRad() << " carEnd.getHeadingInRad() " << carEnd.getHeadingInRad() << std::endl;
        // std::cout << "headingDiff " << headingDiff << std::endl;

        //RSCar carEndMod(newEndPos, headingDiff);

        //CVector3d posDiff((carEnd.pos.x - carStart.pos.x) / turningRadius, (carEnd.pos.y - carStart.pos.y) / turningRadius, 0.0);

//test2
        CVector3d posDiff = (carEnd.pos - carStart.pos) / turningRadius;
        double diffAgl = carEnd.getHeadingInRad() - carStart.getHeadingInRad();
        double headingDiff = PathLengthMath::WrapAngleInRadians( std::remainder(diffAgl, M_PI / 2) );
        CVector3d newEndPos = posDiff.RotateZ(carStart.getHeadingInRad());
        RSCar carEndMod(newEndPos , headingDiff);

//test3
        // double dx = carEnd.getX() - carStart.getX();
        // double dy = carEnd.getY() - carStart.getY();

        // double theta1 = carStart.getHeadingInRad();
        // double theta2 = carEnd.getHeadingInRad();

        // double new_theta = mrpt::math::wrapToPi<double>(theta2 -  theta1);

        // CVector3d newpos((dx * std::cos(theta1) + dy * std::sin(theta1)) / turningRadius,
        //                 (-dx * std::sin(theta1) + dy * std::cos(theta1)) / turningRadius, 0.0);

        // RSCar carEndMod(newpos, new_theta);

        return carEndMod;
    }

    std::vector<RSCar> ReedsShepp::FindShortestPath(RSCar carStart, RSCar carEnd, RSCar carEndMod, double wpDistance, double turningRadius, bool generateOneWp)
    {
        PathSegmentLengths bestPathLengths;
        PathWords bestWord;

        double shortestPathLength = FindShortestPathLength(carEndMod, bestPathLengths, bestWord);

        // No path could be found
        if (shortestPathLength == std::numeric_limits<double>::max())
        {
            std::cout << "Cant find a Reeds-Shepp path" << std::endl;

            return std::vector<RSCar>();
        }

        // Calculate the waypoints to complete this path
        // Use the car's original start position because we no longer need the normalized version
        std::vector<RSCar> shortestPath = AddWaypoints(bestWord, bestPathLengths, carStart, carEnd, wpDistance, turningRadius, generateOneWp);

        // for (int i = 0; i < shortestPath.size(); i++)
        // {
        //     std::cout << " here --- ptr->pos.x " << shortestPath[i].getX() << " ptr->pos.y " << shortestPath[i].getY() << " ptr->heading " << shortestPath[i].getHeadingInRad() << std::endl;
        // }

        return shortestPath;
    }

    std::vector<RSCar> ReedsShepp::AddWaypoints(PathWords word, PathSegmentLengths pathSegmentLengths, RSCar carStart, RSCar carEnd, double wpDistance, double turningRadius, bool generateOneWp)
    {
        // Find the car settings we need to drive through the path
        std::vector<SegmentSettings> pathSettings = PathSettings::GetSettings(word, pathSegmentLengths);

        if (pathSettings.empty())
        {
            std::cout << "Cant find settings for a path" << std::endl;

            return std::vector<RSCar>();
        }

        // Generate the waypoints

        // Data used when generating the path
        // The pos and heading we will move along the path
        CVector3d pos = carStart.pos;
        double heading = carStart.getHeadingInRad();
        // The distance between each step we take when generating the path, the smaller the better, but is also slower
        double stepDistance = 0.05;
        // To generate waypoints with a certain distance between them we need to know how far we have driven since the last wp
        double driveDistance = 0.0;

        // The waypoints
        std::vector<RSCar> waypoints;

        // Add the first wp
        RSCar tempVar(pos, heading, pathSettings[0].gear, pathSettings[0].steering);
        waypoints.push_back(tempVar);

        // Loop through all path 3-5 path segments
        std::cout << "pathSettings.size() " << pathSettings.size() << std::endl;
        for (int i = 0; i < pathSettings.size(); i++)
        {
            SegmentSettings segmentSettings = pathSettings[i];

            // // subdivide the entire arc length by the grid resolution (stepDistance ~ 1 / turningRadius)
            // How many steps will we take to generate this segment
            // Will always be at least 2 no matter the stepDistance
            int n = static_cast<int>(std::ceil((segmentSettings.length * turningRadius) / stepDistance));

            // How far will we move each step?
            double stepLength = (segmentSettings.length * turningRadius) / n;

            // Change stuff depending on in which direction we are moving
            double steeringWheelPos = -1.0;

            if (segmentSettings.steering == RSCar::Steering::Left)
            {
                steeringWheelPos = 1.0;
            }

            // Invert steering if we are reversing
            if (segmentSettings.gear == RSCar::Gear::Back)
            {
                steeringWheelPos *= -1.0;
            }

            // Drive through this segment in small steps
            for (int j = 0; j < n; j++)
            {
                // Update position
                double dx = stepLength * std::cos(heading);
                double dy = stepLength * std::sin(heading);

                if (segmentSettings.gear == RSCar::Gear::Back)
                {
                    dx = -dx;
                    dy = -dy;
                }

                pos = CVector3d(pos.x + dx, pos.y + dy, pos.z);

                // Update heading if we are turning
                if (segmentSettings.steering != RSCar::Steering::Straight)
                {
                    heading = heading + (stepLength / turningRadius) * steeringWheelPos;
                }

                // Should we generate a new wp?
                driveDistance += stepLength;

                if (driveDistance > wpDistance)
                {
                    RSCar tempVar2(pos, heading, segmentSettings.gear, segmentSettings.steering);
                    waypoints.push_back(tempVar2);

                    driveDistance = driveDistance - wpDistance;

                    if (generateOneWp)
                    {
                        return waypoints;
                    }

                    //std::cout << "pos.x " << pos.x << " pos.y " << pos.y << " heading " << heading << std::endl;
                }
            }

            // We also need to add the last pos of this segment as waypoint or the path will not be the same
            // if we for example are ignoring the waypoint where we change direction
            RSCar tempVar3(pos, heading, segmentSettings.gear, segmentSettings.steering);
            waypoints.push_back(tempVar3);

            //std::cout << "pos.x " << pos.x << " pos.y " << pos.y << " heading " << heading << std::endl;
        }

        // Move the last wp pos to the position of the goal car
        // When we generate waypoints, the accuracy depends on the stepDistance, so is not always hitting the goal exactly
        waypoints[waypoints.size() - 1].pos = carEnd.pos;

        // checking data
        //for (auto ptr: waypoints)
        //    std::cout << " ptr->pos.x " << ptr.getX() << " ptr->pos.y " << ptr.getY() << " ptr->heading " << ptr.getHeadingInRad() << std::endl;
        return waypoints;
    }
}
