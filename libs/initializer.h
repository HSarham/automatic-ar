#ifndef INITIALIZER_H
#define INITIALIZER_H
#include <opencv2/core.hpp>
#include <map>
#include <set>
#include <aruco/aruco.h>
#include "cam_config.h"

class Initializer
{
public:
    Initializer(double marker_s, std::vector<CamConfig> &cam_c, const std::set<int> &excluded_cs=std::set<int>());
    Initializer(std::vector<std::vector<std::vector<aruco::Marker>>> &dts, double marker_s, std::vector<CamConfig> &cam_c, const std::set<int> &excluded_cs=std::set<int>());
    static std::vector<std::vector<std::vector<aruco::Marker>>> read_detections_file(std::string path, const std::vector<int> &subseqs=std::vector<int>());
    struct Config{
        bool init_cams=true;
        bool init_markers=true;
        bool init_relative_poses=true;
    };
    std::set<int> get_marker_ids();
    std::set<int> get_cam_ids();
    int get_root_cam();
    int get_root_marker();
    std::map<int,cv::Mat> get_transforms_to_root_cam();
    std::map<int,cv::Mat> get_transforms_to_root_marker();
    void set_transforms_to_root_cam(std::map<int,cv::Mat> &ttrc);
    void set_transforms_to_root_marker(std::map<int,cv::Mat> &ttrm);
    void set_detections(std::vector<std::vector<std::vector<aruco::Marker>>> &dts);
    void obtain_pose_estimations();
    void init_object_transforms();
    std::map<int,cv::Mat> get_object_transforms();
    double get_marker_size();
    std::map<int, std::map<int, std::vector<aruco::Marker>>> get_frame_cam_markers();
    std::vector<CamConfig> get_cam_configs();

private:

    Config config;
    std::vector<std::vector<std::vector<aruco::Marker>>> detections;
    enum transform_type{camera,marker};

    std::map<int, std::map<int, std::vector<std::tuple<cv::Mat,cv::Mat,cv::Mat,double>>>> transformation_sets_cam, transformation_sets_marker;

    struct Node{
        int id;
        mutable double distance;
        mutable int parent;
        bool operator < (const Node &n) const{
            return this->id<n.id;
        }
    };

    int root_cam,root_marker,min_detections = 2;
    double marker_size,threshold = 2.0;
    std::set<int> marker_ids,cam_ids,excluded_cams;
    std::map<int, std::map<int, std::map<int, std::vector<std::pair<cv::Mat,double>>>>> frame_poses_cam,frame_poses_marker;
    std::map<int, std::map<int, std::vector<aruco::Marker>>> frame_cam_markers;
    std::vector<CamConfig> cam_configs;
    std::map<int,cv::Mat> transforms_to_root_cam,transforms_to_root_marker,object_transforms;

    void fill_transformation_set(const std::map<int, std::map<int, std::vector<std::pair<cv::Mat,double>>>> &pose_estimations, const std::map<int, cv::Mat>& transforms_to_root_cam, const std::map<int, cv::Mat>& transforms_to_root_marker, std::vector<std::tuple<cv::Mat,cv::Mat,cv::Mat,double>> &transformation_set);
    void fill_transformation_sets(transform_type tt, const std::map<int, std::map<int, std::vector<std::pair<cv::Mat,double>>>> &pose_estimations, std::map<int, std::map<int, std::vector<std::tuple<cv::Mat,cv::Mat,cv::Mat,double>>>> &transformation_sets);
    int find_best_transformation_min(double marker_size, const std::vector<std::tuple<cv::Mat,cv::Mat,cv::Mat,double>>& solutions, double& min_error);
    int find_best_transformation(double marker_size, const std::vector<std::tuple<cv::Mat,cv::Mat,cv::Mat,double>>& solutions, double& weight);
    void find_best_transformations(double marker_size, const std::map<int, std::map<int, std::vector<std::tuple<cv::Mat,cv::Mat,cv::Mat,double>>>> &transformation_sets, std::map<int, std::map<int, std::pair<cv::Mat,double>>> &best_transformations);
//    double get_reprojection_error(double marker_size, aruco::Marker marker, cv::Mat r, cv::Mat t, cv::Mat cam_mat, cv::Mat dist_coeffs);
    void make_mst(int starting_node, std::set<int> node_ids, const std::map<int, std::map<int, std::pair<cv::Mat,double>>>& adjacency, std::map<int, std::set<int>> &children);
    void find_transforms_to_root(int root_node, const std::map<int,std::set<int>> &children, const std::map<int, std::map<int, std::pair<cv::Mat,double>>> &best_transforms, std::map<int, cv::Mat> &transforms_to_root);
    void init_transforms_cam();
    void init_transforms_marker();
    void init_transforms();

};

#endif // INITIALIZER_H
