#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <set>
#include <aruco/aruco.h>
#include "sparselevmarq.h"
#include "cam_config.h"
#include "initializer.h"
#include "pointcloud_drawer.h"

class MultiCamMapper
{
public:
    MultiCamMapper(size_t root_c, const std::map<int, cv::Mat> &T_to_root_cam, size_t root_m, const std::map<int, cv::Mat> &T_to_root_marker, const std::map<int, cv::Mat> &obj_transforms, const std::map<int, std::map<int, std::vector<aruco::Marker>>> &fcm, float m_size, std::vector<CamConfig> &cam_confs);
    MultiCamMapper(Initializer &);
    MultiCamMapper();
    void init(size_t root_c, const std::map<int, cv::Mat> &T_to_root_cam, size_t root_m, const std::map<int, cv::Mat> &T_to_root_marker, const std::map<int, cv::Mat> &object_poses, const std::map<int, std::map<int, std::vector<aruco::Marker> > > &fcm, float m_size, const std::vector<CamConfig> &cam_confs);
    void init(const std::map<int, cv::Mat> &object_poses, const std::map<int, std::map<int, std::vector<aruco::Marker>>> &fcm);
    void error_function( const typename ucoslam::SparseLevMarq<double>::eVector &input, typename ucoslam::SparseLevMarq<double>::eVector &error);
    void error_function_cams( const typename ucoslam::SparseLevMarq<double>::eVector &input, typename ucoslam::SparseLevMarq<double>::eVector &error);
    void error_function_markers( const typename ucoslam::SparseLevMarq<double>::eVector &input, typename ucoslam::SparseLevMarq<double>::eVector &error);
    void error_function_object_poses( const typename ucoslam::SparseLevMarq<double>::eVector &input, typename ucoslam::SparseLevMarq<double>::eVector &error);
    void error_function_tracking(const ucoslam::SparseLevMarq<double>::eVector &input, ucoslam::SparseLevMarq<double>::eVector &error);

    void serialize_frame_cam_markers(std::ofstream &ofile);
    void deserialize_frame_cam_markers(std::ifstream &ifile);

    void overlay_coords(cv::Mat img, cv::Mat to_ref_cam, float size, int cam_id);
    void overlay_markers(cv::Mat &img, int frame_id, int camera_id);

    static std::vector<int> read_subseqs(std::string path);

    void set_optmize_flag_cam_poses(bool);
    void set_optmize_flag_marker_poses(bool);
    void set_optmize_flag_object_poses(bool);
    void set_optmize_flag_cam_intrinsics(bool);
    void set_with_huber(bool);
    void  optCallBack(const  ucoslam::SparseLevMarq<double>::eVector  &v);
    float hubberDelta=2.5;
    void solve();
    void track();
    bool write_solution_file(std::string path);
    bool read_solution_file(std::string path);
    typename ucoslam::SparseLevMarq<double>::eVector io_vec;

    void write_text_solution_file(std::string text_path);

    void visualize_sequence(std::string path="", size_t num_total_frames=0);

    static void draw_3d_line_points(cv::Mat start, cv::Mat end, int steps, cv::Scalar color, pointcloud_t &point_cloud, cv::Scalar end_color=cv::Scalar(-1,-1,-1));
    static void visualize_camera(int cam_num, const cv::Mat cam_mat, cv::Mat T, const cv::Size im_size,double Z, pointcloud_t &point_cloud);
    static void visualize_cameras(const std::vector<CamConfig> &cam_confs, const std::vector<cv::Mat> &transforms_to_root_cam, const std::vector<int> &index2id, float Z, pointcloud_t &point_cloud_cameras);
    static void visualize_marker(int marker_id,const cv::Mat T,float marker_size,pointcloud_t& point_cloud);
    static void visualize_markers(const std::vector<cv::Mat> &transforms_to_root_marker, const std::vector<int> index2id, float marker_size, pointcloud_t &point_cloud_markers, cv::Mat T);

    void visualize_cameras(pointcloud_t &point_cloud_cameras, float Z=0.1);
    void visualize_markers(pointcloud_t &point_cloud_markers, const cv::Mat T=cv::Mat::eye(4,4,CV_64FC1));

    static int read_stereo_calib(std::string path, std::map<int,std::map<int,cv::Mat>> &transforms);
    static void write_stereo_calib(std::string path, const std::map<int,std::map<int,cv::Mat>> &transforms, int root_cam_id);

    static void read_ground_truth(std::string path, std::map<size_t,cv::Mat>& poses);
    static void write_ground_truth(std::string path, const std::map<size_t,cv::Mat>& poses);

    static void write_detections_file(std::string path, std::vector<std::vector<std::vector<aruco::Marker>>> &seq);

    size_t get_root_cam();
    size_t get_root_marker();
    double get_marker_size();
    std::vector<cv::Size> get_image_sizes();

    struct Config{
        bool optimize_cam_poses=true;
        bool optimize_object_poses=true;
        bool optimize_marker_poses=true;
        bool optimize_cam_intrinsics=true;
        Config(){}
    };

    void set_config(Config &conf);
    size_t get_num_vars(const Config& conf);

    struct MatArray{
        std::vector<cv::Mat> v;
        std::map <int,int> m;
        std::vector<int> id;
        void resize(size_t new_size){
            v.resize(new_size);
            id.resize(new_size);
        }

        MatArray &operator=(const std::map<int,cv::Mat> &ma){
            v.resize(ma.size());
            id.resize(ma.size());
            size_t index=0;
            for(auto it=ma.begin();it != ma.end(); it++){
                m[it->first]=index;
                id[index]=it->first;
                it->second.convertTo(v[index++],CV_64FC1);
            }
            return *this;
        }

        MatArray &operator=(const MatArray &ma){
            m=ma.m;
            id=ma.id;
            v.resize(ma.v.size());
            for(size_t i=0;i<ma.v.size();i++)
                ma.v[i].convertTo(v[i],CV_64FC1);
            return *this;
        }
        cv::Mat &operator[](size_t i){
            return v[m[i]];
        }
        const cv::Mat &operator[](size_t i) const{
            return v[m.at(i)];
        }

        void init(MatArray &ma){
            m=ma.m;
            id=ma.id;
            v.resize(ma.v.size());
        }

        MatArray(MatArray &ma){
            m=ma.m;
            id=ma.id;
            v.resize(ma.v.size());
            for(size_t i=0;i<ma.v.size();i++)
                ma.v[i].convertTo(v[i],CV_64FC1);
        }
        MatArray(){}
    };

    struct MatArrays{
        MatArray object_to_global;
        MatArray transforms_to_root_marker;
        MatArray transforms_to_root_cam;
        MatArray transforms_to_local_cam;
        MatArray cam_mats;
        MatArray dist_coeffs;
        void init(MatArrays &ma, Config &conf){
            if(conf.optimize_cam_poses){
                transforms_to_root_cam.init(ma.transforms_to_root_cam);
                transforms_to_local_cam.init(ma.transforms_to_local_cam);
            }

            if(conf.optimize_marker_poses)
                transforms_to_root_marker.init(ma.transforms_to_root_marker);

            if(conf.optimize_object_poses)
                object_to_global.init(ma.object_to_global);

            if(conf.optimize_cam_intrinsics){
                cam_mats.init(ma.cam_mats);
                dist_coeffs.init(ma.dist_coeffs);
            }
        }

        MatArrays(const MatArrays &ma, const Config &conf=Config()){
            if(conf.optimize_cam_poses){
                transforms_to_root_cam=ma.transforms_to_root_cam;
                transforms_to_local_cam=ma.transforms_to_local_cam;
            }
            if(conf.optimize_marker_poses)
                transforms_to_root_marker=ma.transforms_to_root_marker;
            if(conf.optimize_object_poses)
                object_to_global=ma.object_to_global;
            if(conf.optimize_cam_intrinsics){
                cam_mats=ma.cam_mats;
                dist_coeffs=ma.dist_coeffs;
            }
        }

        MatArrays(){}
    };

    MatArrays get_mat_arrays();

private:
    bool with_huber=false;
    MatArrays mat_arrays;
    enum param_type{camera, marker, object, intrinsics};
    Config config;

    double J_delta=0.001;
    double marker_size;
    std::set<int> cam_ids;
    size_t root_cam, root_marker, num_cameras, num_markers, num_frames, num_point_xys, num_vars=0;
    std::map<int, int> marker_id2index, marker_index2id;

    std::vector<cv::Point3f> marker_points_3d;
    std::map<int,cv::Mat> marker_object_points_3d_mats;
    cv::Mat marker_points_3d_mat;
    std::vector<double> point_projection_deviation;
    std::map<int, std::map<int, std::vector<aruco::Marker>>> frame_cam_markers;
    std::map<int, std::map<int, std::map<int, std::pair<aruco::Marker,size_t>>>> cam_marker_frame,marker_cam_frame,frame_cam_marker;
    std::vector<cv::Size> image_sizes;
    std::vector<CamConfig> cam_configs;

    void init_marker_points_3d();
    void remove_distortions();
    void transformation_mat2vec(const cv::Mat mat, size_t &vec_index);
    void vec2transformation_mat(size_t &vec_index, const typename ucoslam::SparseLevMarq<double>::eVector &vec, cv::Mat& mat);
    size_t fill_io_vec_cams(size_t vec_index=0);
    size_t fill_io_vec_markers(size_t vec_index=0);
    size_t fill_io_vec_object_poses(size_t vec_index=0);
    size_t fill_io_vec_cam_intrinsics(size_t vec_index=0);
    size_t intrinsics_vec2mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma, size_t vec_index=0);
    size_t object_poses_vec2mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays& ma, size_t vec_index=0);
    size_t cams_vec2mats(const typename ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma, size_t vec_index=0);
    size_t markers_vec2mats(const typename ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma, size_t vec_index=0);
    void eval_curr_solution(MatArrays &ma, typename ucoslam::SparseLevMarq<double>::eVector &error);

    void init_io_vec_object_poses();
    void fill_iteration_arrays();
    void project_marker(const MatArrays &ma, size_t frame_num, size_t marker_index, size_t cam_index, std::vector<cv::Point2f> &points_2d);
    void eVec2Mats(const typename ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma);
    void mats2eVec(const Config &conf);
    void mats2eVec();
    void project_points();
    void run();

    void obtain_marker_derivs(const MatArrays &ma_a, const MatArrays &ma_s, const aruco::Marker& marker, Eigen::SparseMatrix<double> &J, size_t frame_index, size_t marker_index, size_t cam_index, size_t error_vec_ind, size_t param_index, std::vector<Eigen::Triplet<double>> &elems);
    void obtain_transformation_derivs(const MatArrays &ma, const ucoslam::SparseLevMarq<double>::eVector &input, param_type transform_type, size_t transform_index, size_t param_offset, Eigen::SparseMatrix<double> &J, std::vector<Eigen::Triplet<double>> &elems);
    void jacobian_function(const ucoslam::SparseLevMarq<double>::eVector &input, Eigen::SparseMatrix<double> &J);

    ucoslam::SparseLevMarq<double> solver;



};

#endif // OPTIMIZER_H
