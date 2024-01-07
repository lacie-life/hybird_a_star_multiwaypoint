// #pragma once

// #include <vector>

// // C# TO C++ CONVERTER NOTE: Forward class declarations:
// class DisplayShortestPath;

// using namespace UnityEngine;
// using namespace PathfindingForVehicles;
// using namespace PathfindingForVehicles::ReedsSheppPaths;

// // Display the shortest reeds shepp path between the car and the target
// class DisplayReedsShepp : public MonoBehaviour
// {
//     // Line renderers - need maximum of 3 to display the path
// public:
//     std::vector<LineRenderer *> lineArray;
//     // Materials
//     Material *lineForwardMaterial;
//     Material *lineReverseMaterial;

//     virtual ~DisplayReedsShepp()
//     {
//         delete lineForwardMaterial;
//         delete lineReverseMaterial;
//     }

// private:
//     void Update();

//     // Get the shortest Reeds-Shepp path and display it
//     void DisplayShortestPath(Transform *startCarTrans, Transform *goalCarTrans);

//     // Display the Reed Shepp path with line renderers
//     void DisplayPath(std::vector<RSCar *> &shortestPath);

//     // Display path positions with a line renderer
//     void AddPositionsToLineRenderer(std::vector<Vector3 *> &nodes, LineRenderer *lineRenderer, bool isReversing);
// };