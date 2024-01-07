//
// Created by lacie on 15/06/2023.
//

#ifndef HYBRIDASTARFLOW_H
#define HYBRIDASTARFLOW_H

#include "hybrid_a_star/hybrid_a_star.h"
#include <opencv4/opencv2/opencv.hpp>

struct RobotPose {
    double x;
    double y;
    double yaw;
};

struct Point {
    int x;
    int y;
};

struct WayPoint {
    RobotPose pose;
    std::string type;
};

/*
 *      --------- (x2, y2)
 *      |       |
 *      |       |
 *      ---------
 *  (x1, y1)
 */
struct Obstacle {
    int type = 1; // default: rectangle
    std::vector<Point> points;
};

class HybridAStarFlow {

public:
    HybridAStarFlow(std::string save_path = "", double steering_angle = 10, int steering_angle_discrete_num = 1,
                    double segment_length = 1.6,
                    int segment_length_discrete_num = 8, double wheel_base = 1.0, double steering_penalty = 1.05,
                    double reversing_penalty = 2.0, double steering_change_penalty = 1.5, double shot_distance = 5.0);

    HybridAStarFlow(std::string save_path = "", std::string config_path = "");

    ~HybridAStarFlow();

    bool Run();

    void InitMapData(int width, int height, double resolution, double origin_x, double origin_y,
                     const std::vector<Obstacle> &obstacles);

    void InitMapData(double resolution, double origin_x, double origin_y,
                     cv::Mat map, float scale);

    void setStartPose(RobotPose _pose);
    void setGoalPose(RobotPose _pose);

private:
    void PublishPath(const VectorVec3d &path);
    void PublishPath(const std::vector<VectorVec3d> &path);
    void PublishSearchedTree(const VectorVec4d &searched_tree);
    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

    void parseMapConfig(const std::string &configPath);
    void parseMissionConfig(const std::string &configPath);
    void parseVehicleConfig(const std::string &configPath);

private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;

    RobotPose current_init_pose;
    RobotPose current_goal_pose;

    std::vector<WayPoint> mMission;

    std::string m_save_path;

    cv::Mat m_Map;
    cv::Mat m_Map_origin;

    bool mDebug = true;
};


#endif //HYBRIDASTARFLOW_H
