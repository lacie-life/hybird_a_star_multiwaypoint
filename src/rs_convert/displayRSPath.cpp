// #include "DisplayReedsShepp.h"
// #include "../SimController.h"
// #include "DisplayShortestPath.h"
// #include "../../Pathfinding/Fixed paths/ReedsShepp/RSCar.h"
// #include "DisplayController.h"

// using namespace UnityEngine;
// using namespace PathfindingForVehicles;
// using namespace PathfindingForVehicles::ReedsSheppPaths;

// void DisplayReedsShepp::Update()
// {
//     // Deactivate all line renderers
//     for (int i = 0; i < lineArray.size(); i++)
//     {
//         lineArray[i]->positionCount = 0;
//     }

//     // The cars we will display the shortest path between
//     Transform *startCarTrans = SimController::current->GetSelfDrivingCarTrans();

//     Transform *goalCarTrans = SimController::current->GetCarMouse();

//     if (goalCarTrans != nullptr && startCarTrans != nullptr)
//     {
//         DisplayShortestPath(startCarTrans, goalCarTrans);
//     }
// }

// void DisplayReedsShepp::DisplayShortestPath(Transform *startCarTrans, Transform *goalCarTrans)
// {
//     Vector3 *startPos = startCarTrans->position;

//     float startHeading = startCarTrans->eulerAngles.y * Mathf::Deg2Rad;

//     Vector3 *goalPos = goalCarTrans->position;

//     float goalHeading = goalCarTrans->eulerAngles.y * Mathf::Deg2Rad;

//     float turningRadius = SimController::current->GetActiveCarData()->carData->turningRadius;

//     // Get the shortest Reeds-Shepp path
//     std::vector<RSCar *> shortestPath = ReedsShepp::GetShortestPath(startPos, startHeading, goalPos, goalHeading, turningRadius, 1.0f, false);

//     // If we found a path
//     if (!shortestPath.empty() && shortestPath.size() > 1)
//     {
//         // Display the path with line renderers
//         DisplayPath(shortestPath);
//     }
// }

// void DisplayReedsShepp::DisplayPath(std::vector<RSCar *> &shortestPath)
// {
//     std::vector<Vector3 *> nodes;

//     // A path needs between 1 and 3 line renderers
//     int lineArrayPos = 0;

//     RSCar::Gear currentGear = shortestPath[0]->gear;

//     for (int i = 0; i < shortestPath.size(); i++)
//     {
//         nodes.push_back(shortestPath[i]->pos);

//         // This means we have finished this segment of the path and should make a line renderer
//         if (shortestPath[i]->gear != currentGear)
//         {
//             bool isReversing = shortestPath[i - 1]->gear == RSCar::Gear::Back ? true : false;

//             AddPositionsToLineRenderer(nodes, lineArray[lineArrayPos], isReversing);

//             // Restart with the next line
//             lineArrayPos += 1;

//             nodes.clear();

//             currentGear = shortestPath[i]->gear;

//             // So the lines connect
//             nodes.push_back(shortestPath[i]->pos);
//         }
//     }

//     // The last segment of the line
//     bool isReversingLast = shortestPath[shortestPath.size() - 1]->gear == RSCar::Gear::Back ? true : false;

//     AddPositionsToLineRenderer(nodes, lineArray[lineArrayPos], isReversingLast);
// }

// void DisplayReedsShepp::AddPositionsToLineRenderer(std::vector<Vector3 *> &nodes, LineRenderer *lineRenderer, bool isReversing)
// {
//     if (nodes.size() > 0)
//     {
//         std::vector<Vector3 *> linePositions;

//         // The height of the line
//         float lineHeight = DisplayController::reedsSheppHeight;

//         for (int i = 0; i < nodes.size(); i++)
//         {
//             Vector3 *pos = new Vector3(nodes[i]->x, lineHeight, nodes[i]->z);

//             linePositions.push_back(pos);

//             // C# TO C++ CONVERTER TODO TASK: A 'delete pos' statement was not added since pos was passed to a method or constructor. Handle memory management manually.
//         }

//         std::vector<Vector3 *> linePositionsArray = linePositions.ToArray();

//         lineRenderer->positionCount = linePositionsArray.size();

//         lineRenderer->SetPositions(linePositionsArray);

//         if (isReversing)
//         {
//             lineRenderer->material = lineReverseMaterial;
//         }
//         else
//         {
//             lineRenderer->material = lineForwardMaterial;
//         }
//     }
// }