//
// Created by lacie on 15/06/2023.
//

#include "hybrid_a_star/HybridAStarFlow.h"
#include "jsoncpp/json/json.h"

#include <iostream>
#include <fstream>

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow(std::string save_path, double steering_angle, int steering_angle_discrete_num,
                                 double segment_length,
                                 int segment_length_discrete_num, double wheel_base, double steering_penalty,
                                 double reversing_penalty, double steering_change_penalty, double shot_distance)
{

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(steering_angle, steering_angle_discrete_num,
                                                                     segment_length, segment_length_discrete_num,
                                                                     wheel_base, steering_penalty, reversing_penalty,
                                                                     steering_change_penalty, shot_distance);

    m_save_path = save_path;
}

HybridAStarFlow::HybridAStarFlow(std::string save_path, std::string config_path)
{
    parseVehicleConfig(config_path);

    parseMapConfig(config_path);

    parseMissionConfig(config_path);

    m_save_path = save_path;
}

HybridAStarFlow::~HybridAStarFlow()
{
    kinodynamic_astar_searcher_ptr_->Reset();
}

// TODO: need check
void HybridAStarFlow::parseMapConfig(const std::string &configPath)
{
    std::ifstream ifs(configPath + "/map.json");
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root)) {
        std::cout << "Failed to parse " << configPath << std::endl;
        return;
    }

    Json::Value map = root["map"];
    int width = map["width"].asInt();
    int height = map["height"].asInt();
    double resolution = map["resolution"].asDouble();
    double origin_x = map["origin_x"].asDouble();
    double origin_y = map["origin_y"].asDouble();

    std::vector<Obstacle> obstacles;
    Json::Value obstacles_json = root["obstacles"];
    for (int i = 0; i < obstacles_json.size(); ++i) {
        Obstacle obs;
        Json::Value obstacle_json = obstacles_json[i];
        obs.type = obstacle_json["type"].asInt();
        Json::Value points_json = obstacle_json["vertices"];
        for (int j = 0; j < points_json.size(); ++j) {
            Point point;
            Json::Value point_json = points_json[j];
            point.x = point_json["x"].asInt();
            point.y = point_json["y"].asInt();
            obs.points.push_back(point);
        }
        obstacles.push_back(obs);
    }

    // Draw map
    cv::Mat map_img = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < obstacles.size(); ++i) {
        Obstacle obs = obstacles[i];
        std::vector<cv::Point> points;
        for (int j = 0; j < obs.points.size(); ++j) {
            Point point = obs.points[j];
            points.push_back(cv::Point(point.x, point.y));
        }
        cv::fillConvexPoly(map_img, points, cv::Scalar(255));
    }

    cv::Mat binary_image;
    cv::Mat map_final;

    cv::threshold(map_img, binary_image, 100, 255, cv::THRESH_BINARY);
    cv::bitwise_not(binary_image, map_final);

    if(mDebug)
    {
        std::cout << "Map information: " << std::endl;
        std::cout << "width: " << width << std::endl;
        std::cout << "height: " << height << std::endl;
        std::cout << "resolution: " << resolution << std::endl;
        std::cout << "origin_x: " << origin_x << std::endl;
        std::cout << "origin_y: " << origin_y << std::endl;
        std::cout << "Number obstacles: " << obstacles.size() << std::endl << std::endl;
    }

    InitMapData(resolution, origin_x, origin_y, map_final, 1);
}


// TODO: Need to be check
void HybridAStarFlow::parseVehicleConfig(const std::string &configPath) {
    std::ifstream ifs(configPath + "/vehicle.json");
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root)) {
        std::cout << "Failed to parse " << configPath << std::endl;
        return;
    }

    Json::Value vehicle = root["vehicle"];
    double wheel_base = vehicle["wheel_base"].asDouble();
    double steering_angle = vehicle["steering_angle"].asDouble();
    int steering_angle_discrete_num = vehicle["steering_angle_discrete_num"].asInt();
    double segment_length = vehicle["segment_length"].asDouble();
    int segment_length_discrete_num = vehicle["segment_length_discrete_num"].asInt();
    double steering_penalty = vehicle["steering_penalty"].asDouble();
    double reversing_penalty = vehicle["reversing_penalty"].asDouble();
    double steering_change_penalty = vehicle["steering_change_penalty"].asDouble();
    double shot_distance = vehicle["shot_distance"].asDouble();

    if(mDebug)
    {
        std::cout << "Vehicle information: " << std::endl;
        std::cout << "wheel_base: " << wheel_base << std::endl;
        std::cout << "steering_angle: " << steering_angle << std::endl;
        std::cout << "steering_angle_discrete_num: " << steering_angle_discrete_num << std::endl;
        std::cout << "segment_length: " << segment_length << std::endl;
        std::cout << "segment_length_discrete_num: " << segment_length_discrete_num << std::endl;
        std::cout << "steering_penalty: " << steering_penalty << std::endl;
        std::cout << "reversing_penalty: " << reversing_penalty << std::endl;
        std::cout << "steering_change_penalty: " << steering_change_penalty << std::endl;
        std::cout << "shot_distance: " << shot_distance << std::endl  << std::endl;
    }

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(steering_angle, steering_angle_discrete_num,
                                                                     segment_length, segment_length_discrete_num,
                                                                     wheel_base, steering_penalty, reversing_penalty,
                                                                     steering_change_penalty, shot_distance);
}

// TODO: Need check
void HybridAStarFlow::parseMissionConfig(const std::string &configPath) {
    std::ifstream ifs(configPath + "/mission.json");
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root)) {
        std::cout << "Failed to parse " << configPath << std::endl;
        return;
    }

    Json::Value waypoint = root["vehicle_states"];
    mMission.clear();

    for(int i = 0; i < waypoint.size(); ++i) {
        Json::Value tempWP = waypoint[i];

        Json::Value pose = tempWP["rear_wheel_position"];

        WayPoint vehicle_state;

        vehicle_state.pose.x = pose[0].asDouble();
        vehicle_state.pose.y = pose[1].asDouble();
        vehicle_state.pose.yaw = pose[2].asDouble();
        vehicle_state.type = tempWP["type"].asString();

        mMission.push_back(vehicle_state);
    }

    if(mDebug)
    {
        std::cout << "Mission information: " << std::endl;
        std::cout << "Number of waypoints: " << mMission.size() << std::endl << std::endl;
    }
}

bool HybridAStarFlow::Run()
{
    if(mMission.size() ==  2)
    {
        setStartPose(mMission[0].pose);
        setGoalPose(mMission[1].pose);

        // TODO: Import Start and Goal Pose
        Vec3d start_state = Vec3d(
                current_init_pose.x,
                current_init_pose.y,
                current_init_pose.yaw
        );
        Vec3d goal_state = Vec3d(
                current_goal_pose.x,
                current_goal_pose.y,
                current_goal_pose.yaw
        );
        // TODO: Export and save results
        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            PublishPath(path);
            PublishVehiclePath(path, 4.0, 2.0, 5u);
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());

            return true;
        }

        kinodynamic_astar_searcher_ptr_->Reset();

        return false;
    }
    else
    {
        std::cout << "Mission Input" << std::endl;

        std::vector<VectorVec3d> totalPath;

        for(int i = 0; i < mMission.size() - 1; i++)
        {
            current_init_pose = mMission[i].pose;
            current_goal_pose = mMission[i + 1].pose;

            Vec3d start_state = Vec3d(
                    current_init_pose.x,
                    current_init_pose.y,
                    current_init_pose.yaw
            );
            Vec3d goal_state = Vec3d(
                    current_goal_pose.x,
                    current_goal_pose.y,
                    current_goal_pose.yaw
            );

            if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
                auto path = kinodynamic_astar_searcher_ptr_->GetPath();
                totalPath.push_back(path);
            } else {
                std::cout << "Failed to find path" << std::endl;
            }
            kinodynamic_astar_searcher_ptr_->Reset();
        }

        PublishPath(totalPath);
    }

}

void HybridAStarFlow::InitMapData(int width, int height, double resolution, double origin_x, double origin_y,
                                  const std::vector<Obstacle> &obstacles)
{
    // TODO: Create a map with obstacles
    const double map_resolution = 1;

    // Full map
    kinodynamic_astar_searcher_ptr_->Init(
            origin_x,
            1.0 * width * resolution,
            origin_y,
            1.0 * height * resolution,
            resolution,
            map_resolution
    );

    // Obstacle map (Rectangle only) => Need check?
    for(auto obs : obstacles)
    {
        for (unsigned int i = obs.points.at(0).x / map_resolution; i < obs.points.at(1).x / map_resolution; ++i) {
            for (unsigned int j = obs.points.at(0).y / map_resolution; j < obs.points.at(1).y / map_resolution; ++j) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(i, j);
                    std::cout << "Set Obstacle: " << i << ", " << j << std::endl;
            }
        }
    }
}

void HybridAStarFlow::InitMapData(double resolution, double origin_x, double origin_y,
                                  cv::Mat map,float scale)
{
    // Read image and convert to map in searcher
    const double map_resolution = 1;
    cv::Mat map_resize;
    cv::resize(map, map_resize, cv::Size(), scale, scale, cv::INTER_NEAREST);

    if(mDebug)
    {
        cv::imshow("map", map_resize);
        cv::waitKey(0);
    }

    m_Map_origin = map_resize.clone();
    m_Map = map_resize.clone();

    kinodynamic_astar_searcher_ptr_->Init(
        origin_x,
        map_resize.cols * resolution,
        origin_y,
        map_resize.rows * resolution,
        resolution,
        map_resolution
            );
    int count = 0;
    for (unsigned int i = 0; i < map_resize.cols; ++i) {
        for (unsigned int j = 0; j < map_resize.rows; ++j) {
            if (map_resize.at<uchar>(j, i) == 0) {
                kinodynamic_astar_searcher_ptr_->SetObstacle(i, j);
                count++;
            }
        }
    }
    std::cout << "Obstacle count: " << count << std::endl;
}


void HybridAStarFlow::setStartPose(RobotPose _pose)
{
    current_init_pose = _pose;
}

void HybridAStarFlow::setGoalPose(RobotPose _pose)
{
    current_goal_pose = _pose;
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path)
{
    std::ofstream out(m_save_path + "/path.json");
    Json::FastWriter writer;
    Json::Value data;

    for (const auto &point : path) {
        Json::Value point_data;
        point_data["x"] = point.x();
        point_data["y"] = point.y();
        point_data["yaw"] = point.z();

        out << writer.write(point_data);

        if (mDebug)
        {
            cv::circle(m_Map, cv::Point(point.x(), point.y()), 1, cv::Scalar(0, 0, 255), 1);
        }
    }
    out.close();

    if(mDebug)
    {
        cv::imwrite(m_save_path + "/path.png", m_Map);
    }
}

void HybridAStarFlow::PublishPath(const std::vector<VectorVec3d> &path) {
    std::ofstream out(m_save_path + "/path.json");
    Json::FastWriter writer;
    Json::Value data;

    for (const auto &path_i : path) {
        for (const auto &point : path_i) {
            Json::Value point_data;
            point_data["x"] = point.x();
            point_data["y"] = point.y();
            point_data["yaw"] = point.z();

            out << writer.write(point_data);

            if (mDebug)
            {
                cv::circle(m_Map, cv::Point(point.x(), point.y()), 1, cv::Scalar(0, 0, 255), 1);
            }
        }
    }
    out.close();

    if(mDebug)
    {
        cv::Mat result;
        cv::cvtColor(m_Map, result, cv::COLOR_GRAY2BGR);

        for (auto wp : mMission)
        {
            cv::circle(result, cv::Point(wp.pose.x, wp.pose.y), 3, cv::Scalar(0, 255, 255), 3);
        }
        cv::imwrite(m_save_path + "/path.png", result);
    }
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree)
{
    std::ofstream out(m_save_path + "/searchedTree.json");
    Json::FastWriter writer;
    Json::Value data;

    for (const auto &point : searched_tree) {
        Json::Value point_data;
        point_data["x"] = point.x();
        point_data["y"] = point.y();
        point_data["yaw"] = point.z();
        point_data["direction"] = point.w();

        out << writer.write(point_data);
    }
    out.close();
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width, double length,
                                         unsigned int vehicle_interval)
{
    std::ofstream out(m_save_path + "/vehiclePath.json");
    Json::FastWriter writer;
    Json::Value data;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        Json::Value point_data;
        point_data["x"] = path[i].x();
        point_data["y"] = path[i].y();
        point_data["yaw"] = path[i].z();
        point_data["width"] = width;
        point_data["length"] = length;

        out << writer.write(point_data);
    }
    out.close();
}

