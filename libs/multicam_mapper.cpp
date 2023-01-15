#include "multicam_mapper.h"
#include "aruco_serdes.h"
#include "sgl.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <thread>

using namespace std;

inline double hubberMono(double e,float delta=5){
    float deltaSq=delta*delta;
    float delta2=2*delta;

    if (e <= deltaSq) { // inlier
      return  e;
    } else  // outlier
       return  delta2*sqrt(e) - deltaSq; // rho(e)   = 2 * delta * e^(1/2) - delta^2
}

inline double getHubberMonoWeight(double SqErr ,float delta){
    if (SqErr==0) return 1;
     return sqrt(hubberMono(  SqErr,delta)/ SqErr);
}

std::vector<cv::Size> MultiCamMapper::get_image_sizes(){
    return image_sizes;
}

void MultiCamMapper::set_with_huber(bool wh){
    with_huber=wh;
}

size_t MultiCamMapper::get_root_cam(){
    return root_cam;
}

size_t MultiCamMapper::get_root_marker(){
    return root_marker;
}

double MultiCamMapper::get_marker_size(){
    return marker_size;
}

vector<int> MultiCamMapper::read_subseqs(string path){
    ifstream ifile(path);
    if(!ifile.is_open())
        throw runtime_error("Could not open subsequences file at: "+path);
    int num;
    vector<int> result;
    while(ifile>>num)
        result.push_back(num);
    return result;
}

MultiCamMapper::MatArrays MultiCamMapper::get_mat_arrays(){
    return mat_arrays;
}

void MultiCamMapper::set_optmize_flag_cam_poses(bool flag){
    config.optimize_cam_poses=flag;
    num_vars = get_num_vars(config);
}

void MultiCamMapper::set_optmize_flag_marker_poses(bool flag){
    config.optimize_marker_poses=flag;
    num_vars = get_num_vars(config);
}

void MultiCamMapper::set_optmize_flag_object_poses(bool flag){
    config.optimize_object_poses=flag;
    num_vars = get_num_vars(config);
}

void MultiCamMapper::set_optmize_flag_cam_intrinsics(bool flag){
    config.optimize_cam_intrinsics=flag;
    num_vars = get_num_vars(config);
}

void MultiCamMapper::set_config(Config &conf){
    config=conf;
    num_vars = get_num_vars(config);
}

int MultiCamMapper::read_stereo_calib(string path, map<int,map<int,cv::Mat>> &transforms){
    ifstream ifile(path,ios_base::binary);
    if(!ifile.is_open())
        throw std::runtime_error("Could not open stereo calib file to read at: "+path);
    size_t num_first_nodes;
    ifile.read((char*)&num_first_nodes,sizeof num_first_nodes);//read the number of first nodes
    for(size_t i=0;i<num_first_nodes;i++){
        int node1;
        ifile.read((char*)&node1,sizeof node1);//read the first node number

        size_t num_second_nodes;
        ifile.read((char*)&num_second_nodes,sizeof num_second_nodes);//read the number of second nodes

        for(size_t j=0;j<num_second_nodes;j++){
            int node2;
            ifile.read((char*)&node2,sizeof node2);//read the second node number
            cv::Mat T=cv::Mat::eye(4,4,CV_64FC1),r(3,1,CV_64FC1);

            for(int i=0;i<3;i++)//read the rotation vector
                ifile.read((char*)&r.at<double>(i),sizeof r.at<double>(i));
            for(int i=0;i<3;i++)//read the translation vector
                ifile.read((char*)&T.at<double>(i,3),sizeof T.at<double>(i,3));

            Rodrigues(r,T(cv::Range(0,3),cv::Range(0,3)));
            transforms[node1][node2]=T;
            transforms[node2][node1]=T.inv();
        }
    }
    int root_cam_id;
    ifile.read((char*)&root_cam_id,sizeof root_cam_id);
    return root_cam_id;
}

void MultiCamMapper::write_stereo_calib(string path, const map<int,map<int,cv::Mat>> &transforms, int root_cam_id){
    ofstream ofile(path,ios_base::binary);
    if(!ofile.is_open())
        throw std::runtime_error("Could not open stereo calib file to write at: "+path);
    auto num_first_nodes=transforms.size();
    ofile.write((char*)&num_first_nodes,sizeof num_first_nodes);//write the number of first nodes
    for(auto it1=transforms.begin();it1!=transforms.end();it1++){
        ofile.write((char*)&it1->first,sizeof it1->first);//write the first node number
        const auto &second_nodes=it1->second;
        auto num_second_nodes=second_nodes.size();
        ofile.write((char*)&num_second_nodes,sizeof num_second_nodes);//write the number of second nodes
        for(auto it2=second_nodes.begin();it2!=second_nodes.end();it2++){
            ofile.write((char*)&it2->first,sizeof it2->first);//write the second node number
            cv::Mat T=it2->second,r;
            Rodrigues(T(cv::Range(0,3),cv::Range(0,3)),r);
            for(int i=0;i<3;i++)//write the rotation vector
                ofile.write((char*)&r.at<double>(i),sizeof r.at<double>(i));
            for(int i=0;i<3;i++)//write the translation vector
                ofile.write((char*)&T.at<double>(i,3),sizeof T.at<double>(i,3));
        }
    }
    ofile.write((char*)&root_cam_id,sizeof root_cam_id);
}

void MultiCamMapper::read_ground_truth(string path, map<size_t,cv::Mat>& poses){
    ifstream in_file(path,ios_base::binary);
    if(!in_file.is_open())
        throw std::runtime_error("Could not open ground truth file to read at: "+path);

    size_t frame_num;
    in_file.read((char*)&frame_num,sizeof frame_num);
    while(in_file.gcount()==sizeof frame_num){
        cv::Mat r(3,1,CV_64FC1),P=cv::Mat::eye(4,4,CV_64FC1);
        for(int i=0;i<3;i++){
            in_file.read((char*)&r.at<double>(i),sizeof r.at<double>(i));
            if(in_file.gcount()!=sizeof r.at<double>(i))
                throw runtime_error("Unexpected end of input ground truth file : "+path);
        }
        for(int i=0;i<3;i++){
            in_file.read((char*)&P.at<double>(i,3),sizeof P.at<double>(i,3));
            if(in_file.gcount()!=sizeof P.at<double>(i,3))
                throw runtime_error("Unexpected end of input ground truth file : "+path);
        }
        cv::Rodrigues(r,P(cv::Range(0,3),cv::Range(0,3)));
        poses[frame_num]=P;
        in_file.read((char*)&frame_num,sizeof frame_num);
    }
}

void MultiCamMapper::write_ground_truth(string path, const map<size_t,cv::Mat>& poses){
    ofstream out_file(path,ios_base::binary);
    if(!out_file.is_open())
        throw std::runtime_error("Could not open ground truth file to write at: "+path);

    for(auto it=poses.begin();it!=poses.end();it++){
        size_t frame_num=it->first;
        out_file.write((char*)&frame_num,sizeof frame_num);
        cv::Mat pose=it->second;
        cv::Mat r;
        cv::Rodrigues(pose(cv::Range(0,3),cv::Range(0,3)),r);
        for(int i=0;i<3;i++)
            out_file.write((char*)&r.at<double>(i),sizeof r.at<double>(i));
        for(int i=0;i<3;i++)
            out_file.write((char*)&pose.at<double>(i,3),sizeof pose.at<double>(i,3));
    }
}

void MultiCamMapper::overlay_coords(cv::Mat img, cv::Mat to_ref_cam, float size, int cam_id){
    to_ref_cam.convertTo(to_ref_cam,CV_64F);
    cv::Mat points=cv::Mat::zeros(4,4,CV_64F);

    for(int i=0;i<3;i++)
        points.at<double>(i,i)=size;
    for(int i=0;i<4;i++)
        points.at<double>(3,i)=1;

    points=to_ref_cam*points;
    vector<cv::Point3f> points_3d(4);
    for(int i=0;i<points_3d.size();i++){
        points_3d[i].x=points.at<double>(0,i);
        points_3d[i].y=points.at<double>(1,i);
        points_3d[i].z=points.at<double>(2,i);
    }
    cv::Mat r,T=mat_arrays.transforms_to_root_cam[cam_id].inv();
    cv::Rodrigues(T(cv::Range(0,3),cv::Range(0,3)),r);

    vector<cv::Point2f> projected_points;
    cv::projectPoints(points_3d,r,T(cv::Range(0,3),cv::Range(3,4)),mat_arrays.cam_mats[cam_id],mat_arrays.dist_coeffs[cam_id],projected_points);

    for(int i=0;i<3;i++){
        cv::Scalar color(0,0,0);
        color[i]=255;

        cv::line(img,projected_points[3],projected_points[i],color,5);
    }
}

void MultiCamMapper::write_detections_file(string path, vector<vector<vector<aruco::Marker>>> &input){
    ofstream detections_file(path,ios_base::binary);

    if(!detections_file.is_open())
        throw runtime_error("Could not open to write the detection file at: "+path);

    size_t num_cams=input[0].size();
    detections_file.write((char*)&num_cams,sizeof num_cams);

    for(size_t frame_num=0;frame_num<input.size();frame_num++){
        auto &frame_markers=input[frame_num];

        for(size_t cam=0;cam<frame_markers.size();cam++){//loop over all possible cameras
            size_t num_cam_markers=frame_markers[cam].size();
            detections_file.write((char*)&num_cam_markers,sizeof num_cam_markers);//read the number of markers for the first camera

            for(int m=0;m<num_cam_markers;m++)
                ArucoSerdes::serialize_marker(frame_markers[cam][m],detections_file);

        }
    }
}

size_t MultiCamMapper::get_num_vars(const Config& conf){
    size_t num_conf_vars=0;
    if(conf.optimize_cam_poses)
        num_conf_vars+=(num_cameras-1)*6;
    if(conf.optimize_marker_poses)
        num_conf_vars+=(num_markers-1)*6;
    if(conf.optimize_object_poses)
        num_conf_vars+=num_frames*6;
    if(conf.optimize_cam_intrinsics)
        num_conf_vars+=num_cameras*9;
    return num_conf_vars;
}

MultiCamMapper::MultiCamMapper(Initializer &initializer){
    init(initializer.get_root_cam(), initializer.get_transforms_to_root_cam(), initializer.get_root_marker(), initializer.get_transforms_to_root_marker(), initializer.get_object_transforms(), initializer.get_frame_cam_markers(), initializer.get_marker_size(),initializer.get_cam_configs());
}

MultiCamMapper::MultiCamMapper(size_t root_c, const std::map<int, cv::Mat> &T_to_root_cam, size_t root_m, const std::map<int, cv::Mat> &T_to_root_marker, const std::map<int, cv::Mat> &object_poses, const std::map<int, std::map<int, std::vector<aruco::Marker>>> &fcm, float m_size, vector<CamConfig> &cam_confs)
{
    init(root_c, T_to_root_cam,  root_m, T_to_root_marker, object_poses, fcm, m_size,cam_confs);
}

void MultiCamMapper::init_marker_points_3d(){
    marker_points_3d=aruco::Marker::get3DPoints(marker_size);
    marker_points_3d_mat=cv::Mat(4,marker_points_3d.size(),CV_64FC1);
    for(size_t i=0;i<marker_points_3d.size();i++){
        marker_points_3d_mat.at<double>(0,i)=marker_points_3d[i].x;
        marker_points_3d_mat.at<double>(1,i)=marker_points_3d[i].y;
        marker_points_3d_mat.at<double>(2,i)=marker_points_3d[i].z;
        marker_points_3d_mat.at<double>(3,i)=1;
    }
}

void MultiCamMapper::init(const std::map<int, cv::Mat> &object_poses, const std::map<int, std::map<int, std::vector<aruco::Marker>>> &fcm){
    num_frames=object_poses.size();
    mat_arrays.object_to_global=object_poses;
    frame_cam_markers=fcm;
    num_vars=get_num_vars(config);
    fill_iteration_arrays();
    remove_distortions();
}

void MultiCamMapper::init(size_t root_c, const std::map<int, cv::Mat> &T_to_root_cam, size_t root_m, const std::map<int, cv::Mat> &T_to_root_marker, const std::map<int, cv::Mat> &object_poses, const std::map<int, std::map<int, std::vector<aruco::Marker>>> &fcm, float m_size, const vector<CamConfig> &cam_confs){
    root_cam=root_c;
    root_marker=root_m;
    frame_cam_markers=fcm;
//    marker_id2index=m_id2index;
//    marker_index2id=m_index2id;

    //convert the transformations
    num_cameras=T_to_root_cam.size();
    mat_arrays.transforms_to_root_cam=T_to_root_cam;

    mat_arrays.transforms_to_local_cam.init(mat_arrays.transforms_to_root_cam);
    for(int i=0;i<num_cameras;i++)
        mat_arrays.transforms_to_local_cam.v[i]=mat_arrays.transforms_to_root_cam.v[i].inv();

    num_markers=T_to_root_marker.size();
    mat_arrays.transforms_to_root_marker=T_to_root_marker;

    num_frames=object_poses.size();
    mat_arrays.object_to_global=object_poses;

    num_vars = (num_frames+num_cameras-1+num_markers-1)*6 + num_cameras*9;

    fill_iteration_arrays();

    cam_configs=cam_confs;
    mat_arrays.cam_mats.resize(num_cameras);
    mat_arrays.dist_coeffs.resize(num_cameras);
    image_sizes.resize(num_cameras);
    size_t cam_index=0;

    for(auto it=T_to_root_cam.begin();it!=T_to_root_cam.end();it++,cam_index++){
        int cam_id=it->first;
        cam_configs[cam_id].getCamMat().convertTo(mat_arrays.cam_mats.v[cam_index],CV_64FC1);
        cam_configs[cam_id].getDistCoeffs().convertTo(mat_arrays.dist_coeffs.v[cam_index],CV_64FC1);
        image_sizes[cam_index]=cam_configs[cam_id].getImageSize();
        mat_arrays.cam_mats.id[cam_index]=cam_id;
        mat_arrays.cam_mats.m[cam_id]=cam_index;
        mat_arrays.dist_coeffs.id[cam_index]=cam_id;
        mat_arrays.dist_coeffs.m[cam_id]=cam_index;
    }
    remove_distortions();
    marker_size=m_size;
    init_marker_points_3d();

    ucoslam::SparseLevMarq<double>::Params p;
    p.verbose=true;
    p.maxIters=10000;
    p.min_average_step_error_diff=1e-4;
    solver.setParams(p);
    ucoslam::SparseLevMarq<double>::eVector error;
    eval_curr_solution(mat_arrays,error);
    cout<<"the very initial error: "<<error.dot(error)<<endl;

}

MultiCamMapper::MultiCamMapper(){
    ucoslam::SparseLevMarq<double>::Params p;
    //p.verbose=true;
    p.maxIters=10000;
    p.min_average_step_error_diff=1e-4;
    solver.setParams(p);
}

void MultiCamMapper::fill_iteration_arrays(){
    num_point_xys=0;
    cam_marker_frame.clear();
    marker_cam_frame.clear();
    frame_cam_marker.clear();
    for(auto frame_it=frame_cam_markers.begin();frame_it!=frame_cam_markers.end();frame_it++){
        int frame_id=frame_it->first;
        auto &cam_markers=frame_it->second;
        for(auto cam_it=cam_markers.begin();cam_it!=cam_markers.end();){
            int cam_id = cam_it->first;
            //erase the camera if its transformation to root camera is not known
            if(mat_arrays.transforms_to_root_cam.m.find(cam_id)==mat_arrays.transforms_to_root_cam.m.end()){
                cam_it=cam_markers.erase(cam_it);
                continue;
            }
            auto &markers = cam_it->second;
            for(auto marker_it=markers.begin();marker_it!=markers.end();){
                int marker_id=marker_it->id;
                //erase the marker if its transformation to root marker is not known
                if(mat_arrays.transforms_to_root_marker.m.find(marker_id)==mat_arrays.transforms_to_root_marker.m.end()){
                    marker_it=markers.erase(marker_it);
                    continue;
                }
                cam_marker_frame[cam_id][marker_id][frame_id]=make_pair(*marker_it,num_point_xys);
                marker_cam_frame[marker_id][cam_id][frame_id]=make_pair(*marker_it,num_point_xys);
                frame_cam_marker[frame_id][cam_id][marker_id]=make_pair(*marker_it,num_point_xys);
                num_point_xys += 8;
                marker_it++;
            }
            cam_it++;
        }
    }
}

//void MultiCamMapper::solve_for_cams(){
//    io_vec.resize((num_cameras-1)*6);
//    fill_io_vec_cams();
//    ucoslam::SparseLevMarq<double>::eVector error;
//    error_function_cams(io_vec,error);
//    cout<<"initial_error: "<<error.dot(error)<<"error size: "<<error.size()<<endl;
//    solver.solve(io_vec,bind(&MultiCamMapper::error_function_cams,this,placeholders::_1,placeholders::_2));
//    cout<<"optimzation result: "<<endl;
//    for(int i=0;i<io_vec.size();i++)
//        cout<<io_vec(i)<<" ";
//    cout<<endl;
//    cams_vec2mats(io_vec,mat_arrays);
//}

//void MultiCamMapper::solve_for_markers(){
//    io_vec.resize((num_markers-1)*6);
//    fill_io_vec_markers();
//    ucoslam::SparseLevMarq<double>::eVector error;
//    error_function_markers(io_vec,error);
//    cout<<"initial_error: "<<error.dot(error)<<"error size: "<<error.size()<<endl;
//    solver.solve(io_vec,bind(&MultiCamMapper::error_function_markers,this,placeholders::_1,placeholders::_2));
//    markers_vec2mats(io_vec,mat_arrays);
//}

//void MultiCamMapper::solve_for_object_poses(){
//    io_vec.resize(num_frames*6);
//    fill_io_vec_object_poses();
//    ucoslam::SparseLevMarq<double>::eVector error;
//    error_function_object_poses(io_vec,error);
//    cout<<"initial_error: "<<error.dot(error)<<"error size: "<<error.size()<<endl;
//    solver.solve(io_vec,bind(&MultiCamMapper::error_function_object_poses,this,placeholders::_1,placeholders::_2));
//    object_poses_vec2mats(io_vec,mat_arrays);
//}
void MultiCamMapper::optCallBack(const  ucoslam::SparseLevMarq<double>::eVector  &v){
    if (hubberDelta>2.5){
        hubberDelta-= 7.5/500;
    }
    //cerr<<"hubberDelta="<<hubberDelta<<endl;
}

void MultiCamMapper::solve(){
    mats2eVec();
    ucoslam::SparseLevMarq<double>::eVector error;
    solver.setStepCallBackFunc(bind(&MultiCamMapper::optCallBack,this,placeholders::_1));
    error_function(io_vec,error);
    cout<<"initial_error: "<<error.dot(error)<<"error size: "<<error.size()<<endl;
    hubberDelta=10;
    solver.solve(io_vec,bind(&MultiCamMapper::error_function,this,placeholders::_1,placeholders::_2),bind(&MultiCamMapper::jacobian_function,this,placeholders::_1,placeholders::_2));
    eVec2Mats(io_vec,mat_arrays);
}

void MultiCamMapper::track(){
    mats2eVec();

    for(pair<int,int> id_index:mat_arrays.transforms_to_root_marker.m)
        marker_object_points_3d_mats[id_index.first]=mat_arrays.transforms_to_root_marker.v[id_index.second]*marker_points_3d_mat;

    ucoslam::SparseLevMarq<double>::eVector error;
//    error_function_tracking(io_vec,error);
//    cout<<"initial_error: "<<error.dot(error)<<"error size: "<<error.size()<<endl;
    hubberDelta=10;
    //solver._params.use_omp=false;
    solver.solve(io_vec,bind(&MultiCamMapper::error_function_tracking,this,placeholders::_1,placeholders::_2));
    eVec2Mats(io_vec,mat_arrays);
}

void MultiCamMapper::mats2eVec(){
    mats2eVec(config);
}

void MultiCamMapper::mats2eVec(const Config &conf){
    io_vec.resize(get_num_vars(conf));

    size_t vec_index=0;
    if(conf.optimize_cam_poses)
        vec_index=fill_io_vec_cams(vec_index);
    if(conf.optimize_marker_poses)
        vec_index=fill_io_vec_markers(vec_index);
    if(conf.optimize_object_poses)
        vec_index=fill_io_vec_object_poses(vec_index);
    if(conf.optimize_cam_intrinsics)
        vec_index=fill_io_vec_cam_intrinsics(vec_index);
}

void MultiCamMapper::vec2transformation_mat(size_t &vec_index, const ucoslam::SparseLevMarq<double>::eVector& vec, cv::Mat &mat){
    cv::Mat rot_vec(3,1,CV_64FC1);

    for(int i=0;i<3;i++){
        rot_vec.at<double>(i)=vec[vec_index+i];
        mat.at<double>(i,3)=vec[vec_index+3+i];
    }
    cv::Rodrigues(rot_vec,mat(cv::Range(0,3),cv::Range(0,3)));

    vec_index += 6;
}

void MultiCamMapper::transformation_mat2vec(const cv::Mat mat, size_t &vec_index){

    cv::Mat rot_vec;
    cv::Rodrigues(mat(cv::Range(0,3),cv::Range(0,3)),rot_vec);

    for(int i=0;i<3;i++){
        io_vec[vec_index+i]=rot_vec.at<double>(i);
        io_vec[vec_index+3+i]=mat.at<double>(i,3);
    }

    vec_index += 6;
}

size_t MultiCamMapper::fill_io_vec_cam_intrinsics(size_t vec_index){
    for(size_t i=0;i<num_cameras;i++){
        io_vec[vec_index++]=mat_arrays.cam_mats.v[i].at<double>(0,0);//fx
        io_vec[vec_index++]=mat_arrays.cam_mats.v[i].at<double>(0,2);//cx
        io_vec[vec_index++]=mat_arrays.cam_mats.v[i].at<double>(1,1);//fy
        io_vec[vec_index++]=mat_arrays.cam_mats.v[i].at<double>(1,2);//cy
        for(int j=0;j<5;j++)
            io_vec[vec_index++]=mat_arrays.dist_coeffs.v[i].at<double>(j);
    }
    return vec_index;
}

size_t MultiCamMapper::fill_io_vec_cams(size_t vec_index){
    for(size_t i=0;i<num_cameras;i++){
        if(i==mat_arrays.transforms_to_root_cam.m[root_cam])
            continue;
        transformation_mat2vec(mat_arrays.transforms_to_root_cam.v[i],vec_index);
    }
    return vec_index;
}

size_t MultiCamMapper::fill_io_vec_markers(size_t vec_index){
    for(size_t i=0;i<num_markers;i++){
        if(i==mat_arrays.transforms_to_root_marker.m[root_marker])
            continue;
        transformation_mat2vec(mat_arrays.transforms_to_root_marker.v[i],vec_index);
    }
    return vec_index;
}

size_t MultiCamMapper::fill_io_vec_object_poses(size_t vec_index){
    for(size_t i=0;i<num_frames;i++)
        transformation_mat2vec(mat_arrays.object_to_global.v[i],vec_index);
    return vec_index;
}

size_t MultiCamMapper::object_poses_vec2mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays& ma, size_t vec_index){
    for(size_t i=0;i<num_frames;i++){
        ma.object_to_global.v[i]=cv::Mat::eye(4,4,CV_64FC1);
        vec2transformation_mat(vec_index,eVec,ma.object_to_global.v[i]);
    }
    return vec_index;
}

size_t MultiCamMapper::cams_vec2mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays& ma, size_t vec_index){
    for(size_t i=0;i<num_cameras;i++){
        ma.transforms_to_root_cam.v[i]=cv::Mat::eye(4,4,CV_64FC1);
        ma.transforms_to_local_cam.v[i]=cv::Mat::eye(4,4,CV_64FC1);
        if(i==ma.transforms_to_root_cam.m[root_cam])
            continue;
        vec2transformation_mat(vec_index,eVec,ma.transforms_to_root_cam.v[i]);
        ma.transforms_to_local_cam.v[i]=ma.transforms_to_root_cam.v[i].inv();
    }
    return vec_index;
}

size_t MultiCamMapper::markers_vec2mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma, size_t vec_index){
    for(size_t i=0;i<num_markers;i++){
        ma.transforms_to_root_marker.v[i]=cv::Mat::eye(4,4,CV_64FC1);
        if(i==ma.transforms_to_root_marker.m[root_marker])
            continue;
        vec2transformation_mat(vec_index,eVec,ma.transforms_to_root_marker.v[i]);
    }
    return vec_index;
}

void MultiCamMapper::remove_distortions(){

    for(pair<const int,map<int,vector<aruco::Marker>>> &f_cam_markers:frame_cam_markers){
        for(pair<const int,vector<aruco::Marker>> &c_markers:f_cam_markers.second){
            int cam_id=c_markers.first;
            vector<aruco::Marker>& markers=c_markers.second;

            cv::Mat points(4*markers.size(),1,CV_32FC2),undistorted_points;

            for(size_t i=0;i<markers.size();i++)//copy to matrix
                for(size_t j=0;j<markers[i].size();j++)
                    points.at<cv::Vec2f>(i*4+j)=markers[i][j];

            cv::Mat cam_mat=mat_arrays.cam_mats[cam_id];
            cv::Mat dist_coeffs=mat_arrays.dist_coeffs[cam_id];

            cv::undistortPoints(points,undistorted_points,cam_mat,dist_coeffs,cv::noArray(),cam_mat);

            for(size_t i=0;i<markers.size();i++)//copy back from matrix
                for(size_t j=0;j<markers[i].size();j++)
                    markers[i][j]=undistorted_points.at<cv::Vec2f>(i*4+j);
        }
    }

}

size_t MultiCamMapper::intrinsics_vec2mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma, size_t vec_index){
    for(size_t i=0;i<num_cameras;i++){
        ma.cam_mats.v[i]=cv::Mat::eye(3,3,CV_64FC1);
        ma.cam_mats.v[i].at<double>(0,0)=eVec(vec_index++);//fx
        ma.cam_mats.v[i].at<double>(0,2)=eVec(vec_index++);//cx
        ma.cam_mats.v[i].at<double>(1,1)=eVec(vec_index++);//fy
        ma.cam_mats.v[i].at<double>(1,2)=eVec(vec_index++);//cy

        ma.dist_coeffs.v[i].create(5,1,CV_64FC1);
        for(int j=0;j<5;j++)
            ma.dist_coeffs.v[i].at<double>(j)=eVec(vec_index++);
    }
    return vec_index;
}

void MultiCamMapper::eVec2Mats(const ucoslam::SparseLevMarq<double>::eVector &eVec, MatArrays &ma){
    size_t vec_index=0;
    if(config.optimize_cam_poses)
        vec_index=cams_vec2mats(eVec,ma,vec_index);
    if(config.optimize_marker_poses)
        vec_index=markers_vec2mats(eVec,ma,vec_index);
    if(config.optimize_object_poses)
        vec_index=object_poses_vec2mats(eVec,ma,vec_index);
    if(config.optimize_cam_intrinsics)
        vec_index=intrinsics_vec2mats(eVec,ma,vec_index);

}

void MultiCamMapper::project_marker(const MatArrays &ma, size_t frame_id, size_t marker_id, size_t cam_id, vector<cv::Point2f> &points_2d){
    //obtain the global coordinates of the marker
    cv::Mat transform, cam_mat, dist_coeffs;

    if(config.optimize_object_poses)
        ma.object_to_global[frame_id].copyTo(transform);
    else
        mat_arrays.object_to_global[frame_id].copyTo(transform);

    if(cam_id != root_cam)
        if(config.optimize_cam_poses)
            transform=ma.transforms_to_root_cam[cam_id].inv()*transform;
        else
            transform=mat_arrays.transforms_to_root_cam[cam_id].inv()*transform;


    if(marker_id != root_marker)
        if(config.optimize_marker_poses)
            transform=transform*ma.transforms_to_root_marker[marker_id];
        else
            transform=transform*mat_arrays.transforms_to_root_marker[marker_id];

    if(config.optimize_cam_intrinsics){
        cam_mat=ma.cam_mats[cam_id];
        dist_coeffs=ma.dist_coeffs[cam_id];
    }
    else{
        cam_mat=mat_arrays.cam_mats[cam_id];
        dist_coeffs=mat_arrays.dist_coeffs[cam_id];
    }

    //obtain the coordinates in the specific camera
    cv::Mat points_3d_mat=cam_mat*transform.rowRange(0,3)*marker_points_3d_mat;
    double* xs=points_3d_mat.ptr<double>(0);
    double* ys=points_3d_mat.ptr<double>(1);
    double* zs=points_3d_mat.ptr<double>(2);
    points_2d.resize(points_3d_mat.cols);
    for(int c=0;c<points_3d_mat.cols;c++){
        points_2d[c].x=xs[c]/zs[c];
        points_2d[c].y=ys[c]/zs[c];
    }
}

void MultiCamMapper::error_function_cams(const ucoslam::SparseLevMarq<double>::eVector &input, ucoslam::SparseLevMarq<double>::eVector &error){
    MatArrays ma(mat_arrays);
    cams_vec2mats(input,ma);
    eval_curr_solution(ma,error);
//    cout<<"error function input:";
//    for(size_t i=0;i<input.size();i++)
//        cout<<input(i)-io_vec(i)<<" ";
//    cout<<endl;
//    cout<<"error function error:";
//    for(size_t i=0;i<error.size();i++)
//        cout<<error(i)<<" ";
//    cout<<endl;
//    cout.flush();
}

void MultiCamMapper::error_function_markers(const ucoslam::SparseLevMarq<double>::eVector &input, ucoslam::SparseLevMarq<double>::eVector &error){
    MatArrays ma(mat_arrays);
    markers_vec2mats(input,ma);
    eval_curr_solution(ma,error);
}

void MultiCamMapper::error_function_object_poses(const ucoslam::SparseLevMarq<double>::eVector &input, ucoslam::SparseLevMarq<double>::eVector &error){
    MatArrays ma(mat_arrays);
    object_poses_vec2mats(input,ma);
    eval_curr_solution(ma,error);
}

void MultiCamMapper::error_function_tracking(const ucoslam::SparseLevMarq<double>::eVector &input, ucoslam::SparseLevMarq<double>::eVector &error){

//    cv::Matx33d R;
//    cv::Vec3d t,r;
//    for(int i=0;i<3;i++){
//        r[i]=input[i];
//        t[i]=input[i+3];
//    }
//    cv::Rodrigues(r,R);

    cv::Mat object_to_global=cv::Mat::eye(4,4,CV_64FC1),r(3,1,CV_64FC1);
    for(int i=0;i<3;i++){
        r.at<double>(i)=input[i];
        object_to_global.at<double>(i,3)=input[i+3];
    }
    cv::Rodrigues(r,object_to_global(cv::Range(0,3),cv::Range(0,3)));

    error.resize(num_point_xys);

    size_t index=0;
    int frame_id=frame_cam_markers.begin()->first;
    map<int, vector<aruco::Marker>> &cams_markers=frame_cam_markers.begin()->second;
    for(pair<const int, vector<aruco::Marker>> &cam_markers:cams_markers){
        size_t cam_id=cam_markers.first;
        vector<aruco::Marker> &markers=cam_markers.second;
        for(size_t m=0;m<markers.size();m++){
            cv::Mat transformed_points=mat_arrays.transforms_to_root_cam[cam_id].inv()*object_to_global*marker_object_points_3d_mats[markers[m].id];
            transformed_points=mat_arrays.cam_mats[cam_id]*transformed_points.rowRange(0,3);
            cv::divide(transformed_points.row(0),transformed_points.row(2),transformed_points.row(0));
            cv::divide(transformed_points.row(1),transformed_points.row(2),transformed_points.row(1));

            double *projected_points_xs=transformed_points.ptr<double>(0);
            double *projected_points_ys=transformed_points.ptr<double>(1);
            for(int i=0;i<marker_points_3d.size();i++){
                //R*marker_points_3d[i]+t;

                double ex=markers[m][i].x-projected_points_xs[i];
                double ey=markers[m][i].y-projected_points_ys[i];
                if(with_huber){
                    double e=ex*ex+ey*ey;
                    double w=getHubberMonoWeight(e,hubberDelta);
                    error(index++)=w*ex;
                    error(index++)=w*ey;
                }
                else{
                    error(index++)=ex;
                    error(index++)=ey;
                }
            }
        }
    }
}

void MultiCamMapper::error_function(const ucoslam::SparseLevMarq<double>::eVector &input, ucoslam::SparseLevMarq<double>::eVector &error){
    MatArrays ma;
    ma.init(mat_arrays,config);
    eVec2Mats(input,ma);
    eval_curr_solution(ma,error);

}

void MultiCamMapper::jacobian_function(const ucoslam::SparseLevMarq<double>::eVector &input, Eigen::SparseMatrix<double> &J){
    MatArrays ma;
    ma.init(mat_arrays,config);
    eVec2Mats(input,ma);
    J.resize(num_point_xys,num_vars);
    vector<vector<Eigen::Triplet<double>>> elems;
    vector<Eigen::Triplet<double>> all_elems;

    size_t param_offset=0;
    if(config.optimize_cam_poses){
        elems.resize(num_cameras);
#pragma omp parallel for
        for(long long cam_index=0;cam_index<num_cameras;cam_index++){
            if(ma.transforms_to_root_cam.id[cam_index]==root_cam)
                continue;
            obtain_transformation_derivs(ma,input,param_type::camera,cam_index,param_offset,J,elems[cam_index]);
        }
        for(size_t i=0;i<elems.size();i++)
            all_elems.insert(all_elems.end(),elems[i].begin(),elems[i].end());
        elems.clear();
        param_offset += (num_cameras-1)*6;
    }

    if(config.optimize_marker_poses){
        elems.resize(num_markers);
#pragma omp parallel for
        for(long long marker_index=0;marker_index<num_markers;marker_index++){
            if(ma.transforms_to_root_marker.id[marker_index]==root_marker)
                continue;
            obtain_transformation_derivs(ma,input,param_type::marker,marker_index,param_offset,J,elems[marker_index]);
        }
        for(size_t i=0;i<elems.size();i++)
            all_elems.insert(all_elems.end(),elems[i].begin(),elems[i].end());
        elems.clear();
        param_offset += (num_markers-1)*6;
    }

    if(config.optimize_object_poses){
        elems.resize(num_frames);
#pragma omp parallel for
        for(long long frame_index=0;frame_index<num_frames;frame_index++){
            obtain_transformation_derivs(ma,input,param_type::object,frame_index,param_offset,J,elems[frame_index]);
        }
        for(size_t i=0;i<elems.size();i++)
            all_elems.insert(all_elems.end(),elems[i].begin(),elems[i].end());
        elems.clear();
        param_offset += num_frames*6;
    }

    if(config.optimize_cam_intrinsics){
        elems.resize(num_cameras);
#pragma omp parallel for
        for(long long cam_index=0;cam_index<num_cameras;cam_index++){
            obtain_transformation_derivs(ma,input,param_type::intrinsics,cam_index,param_offset,J,elems[cam_index]);
        }
        for(size_t i=0;i<elems.size();i++)
            all_elems.insert(all_elems.end(),elems[i].begin(),elems[i].end());
        elems.clear();
        param_offset += num_cameras*9;
    }

    J.setFromTriplets(all_elems.begin(),all_elems.end());
}

void MultiCamMapper::obtain_transformation_derivs(const MatArrays &ma, const ucoslam::SparseLevMarq<double>::eVector &input, param_type parameter_type, size_t transform_index, size_t param_offset, Eigen::SparseMatrix<double> &J, vector<Eigen::Triplet<double>> &elems){
    MatArrays ma_a, ma_s;
    ma_a=ma;
    ma_s=ma;

    cv::Mat T_a,T_s,T;
    size_t transform_param_index;
    if(parameter_type==camera){
        T = ma.transforms_to_root_cam.v[transform_index];
        T_a = ma_a.transforms_to_root_cam.v[transform_index] = T.clone();//to have a separate copy to be able to modify it
        T_s = ma_s.transforms_to_root_cam.v[transform_index] = T.clone();

        transform_param_index = param_offset + transform_index*6;
        if(transform_index>ma.transforms_to_root_cam.m.at(root_cam))
            transform_param_index -= 6;
    }
    else if(parameter_type==marker){
        T = ma.transforms_to_root_marker.v[transform_index];
        T_a = ma_a.transforms_to_root_marker.v[transform_index] = T.clone();
        T_s = ma_s.transforms_to_root_marker.v[transform_index] = T.clone();

        transform_param_index = param_offset + transform_index*6;
        if(transform_index>ma.transforms_to_root_marker.m.at(root_marker))
            transform_param_index -= 6;
    }
    else if(parameter_type==object){
        T = ma.object_to_global.v[transform_index];
        T_a = ma_a.object_to_global.v[transform_index] = T.clone();
        T_s = ma_s.object_to_global.v[transform_index] = T.clone();

        transform_param_index = param_offset + transform_index*6;
    }
    else if(parameter_type==intrinsics){
        cv::Mat K = ma.cam_mats.v[transform_index];
        cv::Mat &K_a = ma_a.cam_mats.v[transform_index];
        cv::Mat &K_s = ma_s.cam_mats.v[transform_index];

        cv::Mat d = ma.dist_coeffs.v[transform_index];
        cv::Mat &d_a = ma_a.dist_coeffs.v[transform_index];
        cv::Mat &d_s = ma_s.dist_coeffs.v[transform_index];

        param_offset += transform_index*9;

        for(int i=0;i<9;i++){
            switch (i){
            case 0://fx
                K_a.at<double>(0,0) += J_delta;
                K_s.at<double>(0,0) -= J_delta;
                break;
            case 1://cx
                K_a.at<double>(0,2) += J_delta;
                K_s.at<double>(0,2) -= J_delta;
                break;
            case 2://fy
                K_a.at<double>(1,1) += J_delta;
                K_s.at<double>(1,1) -= J_delta;
                break;
            case 3://cy
                K_a.at<double>(1,2) += J_delta;
                K_s.at<double>(1,2) -= J_delta;
                break;
            default:
                d_a.at<double>(i-4) += J_delta;
                d_s.at<double>(i-4) -= J_delta;
            }


            size_t cam_id = ma.cam_mats.id[transform_index];
            auto marker_frame = cam_marker_frame[cam_id];
            for(auto marker_it = marker_frame.begin();marker_it!=marker_frame.end();marker_it++){
                int marker_id = marker_it->first;
                auto frame = marker_it->second;
                for(auto frame_it=frame.begin();frame_it!=frame.end();frame_it++){
                    int frame_id = frame_it->first;
                    auto marker_outputIndex = frame_it->second;
                    aruco::Marker marker = marker_outputIndex.first;
                    size_t output_index = marker_outputIndex.second;
                    obtain_marker_derivs(ma_a, ma_s, marker, J, frame_id, marker_id, cam_id, output_index, param_offset+i, elems);
                }
            }


            K_a = K.clone();
            K_s = K.clone();
            d_a = d.clone();
            d_s = d.clone();
        }


        return;
    }
    else{
        cout<<"Invalid transform type. Exiting function..."<<endl;
        return;
    }

    cv::Vec3d rot_vec;//get a copy of the Rodrigues representation of the rotation
    for(int i=0;i<3;i++)
        rot_vec[i]=input(transform_param_index+i);

    for(int i=0;i<6;i++){
        //modify parameters
        if(i<3){
            cv::Vec3d rv_a,rv_s;
            rv_s=rv_a=rot_vec;
            rv_a[i] += J_delta;
            rv_s[i] -= J_delta;
            cv::Rodrigues(rv_a,T_a(cv::Range(0,3),cv::Range(0,3)));
            cv::Rodrigues(rv_s,T_s(cv::Range(0,3),cv::Range(0,3)));
        }
        else{
            T_a.at<double>(i-3,3) += J_delta;
            T_s.at<double>(i-3,3) -= J_delta;
        }
        //get the corresponding derives
        if(parameter_type==camera){
            size_t cam_id = ma.transforms_to_root_cam.id[transform_index];
            auto marker_frame = cam_marker_frame[cam_id];
            for(auto marker_it = marker_frame.begin(); marker_it != marker_frame.end(); marker_it++){
                int marker_id=marker_it->first;
                auto frames = marker_it->second;
                for(auto frame_it=frames.begin();frame_it!=frames.end();frame_it++){
                    int frame_id = frame_it->first;
                    auto marker_outputindex = frame_it->second;
                    aruco::Marker marker = marker_outputindex.first;
                    size_t output_index = marker_outputindex.second;
                    obtain_marker_derivs(ma_a, ma_s, marker, J, frame_id, marker_id, cam_id, output_index, transform_param_index+i, elems);
                }
            }
        }
        else if(parameter_type==marker){
            size_t marker_id = ma.transforms_to_root_marker.id[transform_index];
            auto cam_frame = marker_cam_frame[marker_id];
            for(auto cam_it = cam_frame.begin(); cam_it != cam_frame.end(); cam_it++){
                size_t cam_id = cam_it->first;
                auto frames = cam_it->second;
                for(auto frame_it = frames.begin(); frame_it != frames.end(); frame_it++){
                    size_t frame_id = frame_it->first;
                    auto marker_outputindex = frame_it->second;
                    aruco::Marker marker = marker_outputindex.first;
                    size_t output_index = marker_outputindex.second;
                    obtain_marker_derivs(ma_a, ma_s, marker, J, frame_id, marker_id, cam_id, output_index, transform_param_index+i, elems);
                }
            }
        }
        else if(parameter_type==object){
            size_t frame_id = ma.object_to_global.id[transform_index];
            auto cam_marker = frame_cam_marker[frame_id];
            for(auto cam_it = cam_marker.begin(); cam_it != cam_marker.end(); cam_it++){
                size_t cam_id = cam_it->first;
                auto markers = cam_it->second;
                for(auto marker_it=markers.begin(); marker_it!=markers.end(); marker_it++){
                    size_t marker_id = marker_it->first;
                    auto marker_outputindex = marker_it->second;
                    aruco::Marker marker = marker_outputindex.first;
                    size_t output_index = marker_outputindex.second;
                    obtain_marker_derivs(ma_a, ma_s, marker, J, frame_id, marker_id, cam_id, output_index, transform_param_index+i, elems);
                }
            }
        }

        //restore the modified values
        if(i<3){
            T(cv::Range(0,3),cv::Range(0,3)).copyTo(T_a(cv::Range(0,3),cv::Range(0,3)));
            T(cv::Range(0,3),cv::Range(0,3)).copyTo(T_s(cv::Range(0,3),cv::Range(0,3)));
        }
        else{
            T(cv::Range(0,3),cv::Range(3,4)).copyTo(T_a(cv::Range(0,3),cv::Range(3,4)));
            T(cv::Range(0,3),cv::Range(3,4)).copyTo(T_s(cv::Range(0,3),cv::Range(3,4)));
        }
    }
}

void MultiCamMapper::obtain_marker_derivs(const MatArrays &ma_a, const MatArrays &ma_s, const aruco::Marker& marker, Eigen::SparseMatrix<double> &J, size_t frame_id, size_t marker_id, size_t cam_id, size_t error_vec_ind, size_t param_index, vector<Eigen::Triplet<double>> &elems){
    vector<cv::Point2f> projected_points_a,projected_points_s;
    project_marker(ma_a,frame_id,marker_id,cam_id,projected_points_a);
    project_marker(ma_s,frame_id,marker_id,cam_id,projected_points_s);
    for(int p=0;p<4;p++){
        double error_x_a=marker[p].x-projected_points_a[p].x;
        double error_x_s=marker[p].x-projected_points_s[p].x;
        double error_x_deriv=(error_x_a-error_x_s)/(2*J_delta);

        elems.push_back(Eigen::Triplet<double>(error_vec_ind+p*2,param_index,error_x_deriv));

        double error_y_a=marker[p].y-projected_points_a[p].y;
        double error_y_s=marker[p].y-projected_points_s[p].y;
        double error_y_deriv=(error_y_a-error_y_s)/(2*J_delta);

        elems.push_back(Eigen::Triplet<double>(error_vec_ind+p*2+1,param_index,error_y_deriv));

    }
}

void MultiCamMapper::eval_curr_solution(MatArrays& ma, ucoslam::SparseLevMarq<double>::eVector &error){

    error.resize(num_point_xys);

    size_t index=0;
    for(auto frame_it=frame_cam_markers.begin();frame_it!=frame_cam_markers.end();frame_it++){
        int frame_id = frame_it -> first;
        map<int, vector<aruco::Marker>> &cam_markers=frame_it->second;
        for(auto it=cam_markers.begin();it!=cam_markers.end();it++){
            size_t cam_id=it->first;
            vector<aruco::Marker> &markers=it->second;
            for(size_t m=0;m<markers.size();m++){
                vector<cv::Point2f> projected_points;
                project_marker(ma,frame_id,markers[m].id,cam_id,projected_points);

                for(int i=0;i<4;i++){
                    double ex=markers[m][i].x-projected_points[i].x;
                    double ey=markers[m][i].y-projected_points[i].y;
                    if(with_huber){
                        double e=ex*ex+ey*ey;
                        double w=getHubberMonoWeight(e,hubberDelta);
                        error(index++)=w*ex;
                        error(index++)=w*ey;
                    }
                    else{
                        error(index++)=ex;
                        error(index++)=ey;
                    }
                }
            }
        }
    }
}

void MultiCamMapper::serialize_frame_cam_markers(ofstream &ofile){
    size_t num_f=frame_cam_markers.size();
    ofile.write((char*)&num_f,sizeof num_f);
    for(auto frame_it=frame_cam_markers.begin();frame_it!=frame_cam_markers.end();frame_it++){
        int frame_id=frame_it->first;
        ofile.write((char*)&frame_id,sizeof frame_id);

        auto &cam_markers=frame_it->second;
        size_t num_c=cam_markers.size();
        ofile.write((char*)&num_c,sizeof num_c);
        for(auto cam_it=cam_markers.begin();cam_it!=cam_markers.end();cam_it++){
            int cam_id=cam_it->first;
            ofile.write((char*)&cam_id,sizeof cam_id);

            vector<aruco::Marker> &markers=cam_it->second;
            size_t num_m=markers.size();
            ofile.write((char*)&num_m,sizeof num_m);
            for(size_t m=0;m<num_m;m++)
                ArucoSerdes::serialize_marker(markers[m],ofile);
        }
    }
}

bool MultiCamMapper::write_solution_file(string path){
    mats2eVec();
    ofstream ofile(path,ios_base::binary);
    if(!ofile.is_open()){
        cout<<"Could not open a file in: "<<path<<" for writing."<<endl;
        return false;
    }
    ofile.write((char*)&num_cameras,sizeof num_cameras);
    //write the ids of the indices
    for(size_t i=0;i<mat_arrays.cam_mats.id.size();i++)
        ofile.write((char*)&mat_arrays.cam_mats.id[i],sizeof mat_arrays.cam_mats.id[i]);

    ofile.write((char*)&root_cam,sizeof root_cam);

    for(size_t i=0;i<num_cameras;i++){
        ofile.write((char*)&image_sizes[i].width,sizeof image_sizes[i].width);
        ofile.write((char*)&image_sizes[i].height,sizeof image_sizes[i].height);
    }

    ofile.write((char*)&num_markers,sizeof num_markers);
    //write the ids of the indices
    for(size_t i=0;i<mat_arrays.transforms_to_root_marker.id.size();i++)
        ofile.write((char*)&mat_arrays.transforms_to_root_marker.id[i],sizeof mat_arrays.transforms_to_root_marker.id[i]);

    ofile.write((char*)&root_marker,sizeof root_marker);

    ofile.write((char*)&marker_size,sizeof marker_size);
    ofile.write((char*)&num_frames,sizeof num_frames);
    //write the ids of the indices
    for(size_t i=0;i<mat_arrays.object_to_global.id.size();i++)
        ofile.write((char*)&mat_arrays.object_to_global.id[i],sizeof mat_arrays.object_to_global.id[i]);
    //get a new conf to have all parameters written to vector regardless of them having been optimized or not
    Config conf;
    mats2eVec(conf);
    //write the vector
    for(size_t i=0;i<io_vec.size();i++)
        ofile.write((char*)&io_vec(i),sizeof(double));
    //frame_camera_marker
    serialize_frame_cam_markers(ofile);
    //configuration
    ofile.write((char*)&config.optimize_cam_poses,sizeof config.optimize_cam_poses);
    ofile.write((char*)&config.optimize_marker_poses,sizeof config.optimize_marker_poses);
    ofile.write((char*)&config.optimize_object_poses,sizeof config.optimize_object_poses);
    ofile.write((char*)&config.optimize_cam_intrinsics,sizeof config.optimize_cam_intrinsics);

    return true;
}

void MultiCamMapper::deserialize_frame_cam_markers(ifstream &ifile){
    frame_cam_markers.clear();
    size_t num_f;
    ifile.read((char*)&num_f,sizeof num_f);
    for(size_t f=0;f<num_f;f++){
        int frame_id;
        ifile.read((char*)&frame_id,sizeof frame_id);

        size_t num_c;
        ifile.read((char*)&num_c,sizeof num_c);
        for(size_t c=0;c<num_c;c++){
            int cam_id;
            ifile.read((char*)&cam_id,sizeof cam_id);

            size_t num_m;
            ifile.read((char*)&num_m,sizeof num_m);
            frame_cam_markers[f][c].resize(num_m);
            for(size_t m=0;m<num_m;m++)
                ArucoSerdes::deserialize_marker(ifile,frame_cam_markers[f][c][m]);
        }
    }
}

bool MultiCamMapper::read_solution_file(string path){
    ifstream ifile(path,ios_base::binary);
    if(!ifile.is_open()){
        cout<<"Could not open a file in: "<<path<<" for reading."<<endl;
        return false;
    }

    //Cameras
    ifile.read((char*)&num_cameras,sizeof num_cameras);
    mat_arrays.cam_mats.resize(num_cameras);

    //read the ids of the indices

    for(size_t i=0;i<mat_arrays.cam_mats.id.size();i++){
        ifile.read((char*)&mat_arrays.cam_mats.id[i],sizeof mat_arrays.cam_mats.id[i]);
        mat_arrays.cam_mats.m[mat_arrays.cam_mats.id[i]]=i;
    }
    //prepare the dist coeffs
    mat_arrays.dist_coeffs.resize(num_cameras);
    mat_arrays.dist_coeffs.id=mat_arrays.cam_mats.id;
    mat_arrays.dist_coeffs.m=mat_arrays.cam_mats.m;

    //prepare the transforms to root and local cams
    mat_arrays.transforms_to_root_cam.resize(num_cameras);
    mat_arrays.transforms_to_root_cam.id=mat_arrays.cam_mats.id;
    mat_arrays.transforms_to_root_cam.m=mat_arrays.cam_mats.m;

    mat_arrays.transforms_to_local_cam.resize(num_cameras);
    mat_arrays.transforms_to_local_cam.id=mat_arrays.cam_mats.id;
    mat_arrays.transforms_to_local_cam.m=mat_arrays.cam_mats.m;

    ifile.read((char*)&root_cam,sizeof root_cam);

    image_sizes.resize(num_cameras);
    for(size_t i=0;i<num_cameras;i++){
        ifile.read((char*)&image_sizes[i].width,sizeof image_sizes[i].width);
        ifile.read((char*)&image_sizes[i].height,sizeof image_sizes[i].height);
    }
    //Cameras

    //Markers
    ifile.read((char*)&num_markers,sizeof num_markers);
    mat_arrays.transforms_to_root_marker.resize(num_markers);

    for(size_t i=0;i<mat_arrays.transforms_to_root_marker.id.size();i++){
        ifile.read((char*)&mat_arrays.transforms_to_root_marker.id[i],sizeof mat_arrays.transforms_to_root_marker.id[i]);
        mat_arrays.transforms_to_root_marker.m[mat_arrays.transforms_to_root_marker.id[i]]=i;
    }

    ifile.read((char*)&root_marker,sizeof root_marker);
    //Markers
    ifile.read((char*)&marker_size,sizeof marker_size);
    init_marker_points_3d();
    //Frames
    ifile.read((char*)&num_frames,sizeof num_frames);
    mat_arrays.object_to_global.resize(num_frames);

    for(size_t i=0;i<mat_arrays.object_to_global.id.size();i++){
        ifile.read((char*)&mat_arrays.object_to_global.id[i],sizeof mat_arrays.object_to_global.id[i]);
        mat_arrays.object_to_global.m[mat_arrays.object_to_global.id[i]]=i;
    }
    //Frames
    Config conf;
    set_config(conf);

    io_vec.resize(num_vars);
    for(size_t i=0;i<io_vec.size();i++)
        ifile.read((char*)&io_vec(i),sizeof(double));

    eVec2Mats(io_vec,mat_arrays);
    deserialize_frame_cam_markers(ifile);
    fill_iteration_arrays();

    ifile.read((char*)&conf.optimize_cam_poses,sizeof conf.optimize_cam_poses);
    ifile.read((char*)&conf.optimize_marker_poses,sizeof conf.optimize_marker_poses);
    ifile.read((char*)&conf.optimize_object_poses,sizeof conf.optimize_object_poses);
    ifile.read((char*)&conf.optimize_cam_intrinsics,sizeof conf.optimize_cam_intrinsics);
    set_config(conf);

    //frame camera marker
    return true;
}

void MultiCamMapper::overlay_markers(cv::Mat &img, int frame_id, int camera_index){
//    auto markers=frame_cam_markers[frame_id][camera_id];
    int camera_id=mat_arrays.cam_mats.id[camera_index];

    if(img.empty())
        img=cv::Mat::zeros(image_sizes[camera_index],CV_8UC3)+cv::Scalar(255,255,255);

    if(mat_arrays.object_to_global.m.count(frame_id) == 0)
        return;

    for(auto it=mat_arrays.transforms_to_root_marker.m.begin();it!=mat_arrays.transforms_to_root_marker.m.end();it++){
        int marker_id = it->first;
        int marker_index = it->second;
        cv::Mat r, transform = mat_arrays.transforms_to_local_cam.v[camera_index]*mat_arrays.object_to_global[frame_id]*mat_arrays.transforms_to_root_marker.v[marker_index];
        cv::Rodrigues(transform(cv::Range(0,3),cv::Range(0,3)),r);
        if(transform.at<double>(2,2)<0){
            vector<cv::Point2f> points_2d;
            //project_marker(mat_arrays,frame_id,marker_id,camera_id,points_2d);

            cv::projectPoints(marker_points_3d,r,transform(cv::Range(0,3),cv::Range(3,4)),mat_arrays.cam_mats[camera_id],mat_arrays.dist_coeffs[camera_id],points_2d);
            for(int j=0;j<4;j++)
                cv::line(img,points_2d[j],points_2d[(j+1)%4],cv::Scalar(0.0,255.0,0.0),2);
        }
    }
}

void MultiCamMapper::write_text_solution_file(string text_path){
    cv::FileStorage fs(text_path,cv::FileStorage::WRITE);
    fs<<"marker_size"<<marker_size;
    fs<<"transforms_to_root_cam"<<"[";
    for(std::pair<const int,int> &p : mat_arrays.transforms_to_root_cam.m){
        fs<<"{:";
        int cam_id = p.first;
        int cam_index = p.second;
        fs<<"cam_id"<<cam_id;
        fs<<"transform"<<mat_arrays.transforms_to_root_cam.v[cam_index];
        fs<<"}";
    }
    fs<<"]";

    fs<<"transforms_to_root_marker"<<"[";
    for(std::pair<const int,int> &p : mat_arrays.transforms_to_root_marker.m){
        fs<<"{:";
        int marker_id = p.first;
        int marker_index = p.second;
        fs<<"marker_id"<<marker_id;
        fs<<"transform"<<mat_arrays.transforms_to_root_marker.v[marker_index];
        fs<<"}";
    }
    fs<<"]";

    fs<<"root_marker_to_root_cam"<<"[";
    for(std::pair<const int,int> &p : mat_arrays.object_to_global.m){
        fs<<"{:";
        int frame_id = p.first;
        int frame_index = p.second;
        fs<<"frame_id"<<frame_id;
        fs<<"transform"<<mat_arrays.object_to_global.v[frame_index];
        fs<<"}";
    }
    fs<<"]";
}


void MultiCamMapper::visualize_sequence(string path, size_t num_total_frames){

#ifdef PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_cameras(new pcl::PointCloud<pcl::PointXYZRGB>());
    visualize_cameras(*point_cloud_cameras);
    pcl::visualization::PCLVisualizer pclv;
    pclv.addPointCloud(point_cloud_cameras,"cameras");
    bool markers_cloud_added=false;

    if(num_total_frames==0)
        num_total_frames=mat_arrays.object_to_global.m.end()->first+1;
    for(int id=0;id<num_total_frames;id++){
        if(mat_arrays.object_to_global.m.count(id) != 0){
            int index = mat_arrays.object_to_global.m[id];
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_output(new pcl::PointCloud<pcl::PointXYZRGB>());
            visualize_markers(*tmp_output,mat_arrays.object_to_global.v[index]);
            if(!markers_cloud_added){
                pclv.addPointCloud(tmp_output,"markers");
                markers_cloud_added=true;
            }
            else
                pclv.updatePointCloud(tmp_output,"markers");
            pclv.spinOnce(1,true);
        }
        else if(markers_cloud_added){
            pclv.removePointCloud("markers");
            markers_cloud_added=false;
        }
        if(!path.empty()){
            stringstream s;
            s<<setfill('0')<<setw(4)<<id<<".png";
            pclv.saveScreenshot(path+"/"+s.str());
        }
    }
#else
    auto point_cloud_ptr = std::make_shared<pointcloud_t>();

    std::shared_ptr<PointCloudDrawer> pcd = std::make_shared<PointCloudDrawer>(point_cloud_ptr);
    std::shared_ptr<sgl::SglDisplay> display=sgl::SglDisplay::create(pcd,1024,768,2);

    sgl::Matrix44 cam_mat;
    cam_mat.translate(sgl::Point3(0,0,1));
    display->setViewMatrix(cam_mat);
    sgl::Scene &scene=display->getScene();

    std::thread d_thread([&display]{display->display();});

    for(auto it=mat_arrays.object_to_global.m.begin();it==mat_arrays.object_to_global.m.begin();it++){
        int frame_index=it->second;
        point_cloud_ptr->clear();
        visualize_cameras(*point_cloud_ptr);
        visualize_markers(*point_cloud_ptr,mat_arrays.object_to_global.v[frame_index]);
        pcd->draw(scene);
    }

    d_thread.join();
#endif
}

void MultiCamMapper::draw_3d_line_points(cv::Mat start, cv::Mat end, int steps, cv::Scalar start_color, pointcloud_t &point_cloud, cv::Scalar end_color){

    if(end_color==cv::Scalar(-1,-1,-1))
        end_color=start_color;
    cv::Scalar color=start_color;
    cv::Scalar color_step=(end_color-start_color)/steps;

    cv::Mat step=(end-start)/steps;
    cv::Mat point=start.clone();
    step.convertTo(step,CV_32FC1);
    point.convertTo(point,CV_32FC1);
    float &x=point.at<float>(0);
    float &y=point.at<float>(1);
    float &z=point.at<float>(2);
    for(int i=0;i<steps;i++,point+=step,color+=color_step){
        point_t p(start_color[0],start_color[1],start_color[2]);
        p.x=x;
        p.y=y;
        p.z=z;
        point_cloud.push_back(p);
    }
}

void MultiCamMapper::visualize_camera(int cam_num, const cv::Mat cam_mat, cv::Mat T_in, const cv::Size im_size,double Z, pointcloud_t &point_cloud){

    cv::Mat num_image=cv::Mat::zeros(im_size,CV_8UC1);
    cv::putText(num_image,to_string(cam_num),cv::Point(im_size.width/6,im_size.height-30),cv::FONT_HERSHEY_SIMPLEX,20,cv::Scalar(255),8);
    cv::flip(num_image,num_image,1);

    cv::Mat points(0,3,CV_32FC1);
    //camera center
    points.push_back(cv::Mat(cv::Vec3f(0,0,0)).t());
    //image corners
    points.push_back(cv::Mat(cv::Vec3f(0,0,1)).t());
    points.push_back(cv::Mat(cv::Vec3f(im_size.width-1,0,1)).t());
    points.push_back(cv::Mat(cv::Vec3f(0,im_size.height-1,1)).t());
    points.push_back(cv::Mat(cv::Vec3f(im_size.width-1,im_size.height-1,1)).t());

    //pixels to be visualized
    for(int r=0;r<num_image.rows;r++)
        for(int c=0;c<num_image.cols;c++)
            if(num_image.at<uchar>(r,c)==255)
                points.push_back(cv::Mat(cv::Vec3f(c,r,1)).t());

    cv::Mat cam_mat_32;

    cam_mat.convertTo(cam_mat_32,CV_32FC1);
    points=Z*cam_mat_32.inv()*points.t();

    cv::Mat T;
    T_in.convertTo(T,CV_32FC1);
    points.push_back(cv::Mat::ones(1,points.cols,CV_32FC1));
    points=T*points;
    cv::Scalar color(0,0,255);

    draw_3d_line_points(points.col(0),points.col(1),100,color,point_cloud,color);
    draw_3d_line_points(points.col(0),points.col(2),100,color,point_cloud,color);
    draw_3d_line_points(points.col(0),points.col(3),100,color,point_cloud,color);
    draw_3d_line_points(points.col(0),points.col(4),100,color,point_cloud,color);
    draw_3d_line_points(points.col(1),points.col(2),100,color,point_cloud,color);
    draw_3d_line_points(points.col(1),points.col(3),100,color,point_cloud,color);
    draw_3d_line_points(points.col(2),points.col(4),100,color,point_cloud,color);
    draw_3d_line_points(points.col(3),points.col(4),100,color,point_cloud,color);

    for(int c=5;c<points.cols;c++){
        point_t p(color[0],color[1],color[2]);
        p.x=points.at<float>(0,c);
        p.y=points.at<float>(1,c);
        p.z=points.at<float>(2,c);
        point_cloud.push_back(p);
    }

}

void MultiCamMapper::visualize_cameras(const std::vector<CamConfig> &cam_confs, const vector<cv::Mat> &transforms_to_root_cam, const vector<int> &index2id, float Z, pointcloud_t &point_cloud_cameras){
    for(size_t i=0;i<transforms_to_root_cam.size();i++)
        visualize_camera(i,cam_confs[index2id[i]].getCamMat(),transforms_to_root_cam[i],cam_confs[index2id[i]].getImageSize(),Z,point_cloud_cameras);
}

void MultiCamMapper::visualize_cameras(pointcloud_t &point_cloud_cameras, float Z){
    for(std::pair<int,int> id_index:mat_arrays.transforms_to_root_cam.m){
        int id=id_index.first;
        int index=id_index.second;
        visualize_camera(id,mat_arrays.cam_mats[id],mat_arrays.transforms_to_root_cam[id],image_sizes[index],Z,point_cloud_cameras);
    }
}

void MultiCamMapper::visualize_marker(int marker_id,const cv::Mat T_in,float marker_size,pointcloud_t &point_cloud){
    cv::Mat T;
    T_in.convertTo(T,CV_32FC1);
    cv::Mat points(0,2,CV_32FC1);

    points.push_back(cv::Mat(cv::Vec2f(-marker_size/2,marker_size/2)).t());
    points.push_back(cv::Mat(cv::Vec2f(marker_size/2,marker_size/2)).t());
    points.push_back(cv::Mat(cv::Vec2f(marker_size/2,-marker_size/2)).t());
    points.push_back(cv::Mat(cv::Vec2f(-marker_size/2,-marker_size/2)).t());

    cv::Mat num_image=cv::Mat::zeros(64,64,CV_8UC1);
    cv::putText(num_image,to_string(marker_id),cv::Point(0,42),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),2);

    for(int i=0;i<num_image.rows;i++){
        float y=-(i+.5-num_image.rows/2.0)/num_image.rows*marker_size;
        for(int j=0;j<num_image.cols;j++){
            float x=(j+.5-num_image.cols/2.0)/num_image.cols*marker_size;
            if(num_image.at<uchar>(i,j)==255)
                points.push_back(cv::Mat(cv::Vec2f(x,y)).t());
        }
    }

    points=points.t();
    points.push_back(cv::Mat::zeros(1,points.cols,CV_32FC1));
    points.push_back(cv::Mat::ones(1,points.cols,CV_32FC1));
    points=T*points;

    cv::Scalar color(0,255,0);
    draw_3d_line_points(points.col(0),points.col(1),100,color,point_cloud,color);
    draw_3d_line_points(points.col(1),points.col(2),100,color,point_cloud,color);
    draw_3d_line_points(points.col(2),points.col(3),100,color,point_cloud,color);
    draw_3d_line_points(points.col(3),points.col(0),100,color,point_cloud,color);

    for(int i=4;i<points.cols;i++){
        point_t point(color[0],color[1],color[2]);
        point.x=points.at<float>(0,i);
        point.y=points.at<float>(1,i);
        point.z=points.at<float>(2,i);
        point_cloud.push_back(point);
    }
}

void MultiCamMapper::visualize_markers(const vector<cv::Mat> &transforms_to_root_marker, const vector<int> index2id, float marker_size, pointcloud_t &point_cloud_markers, cv::Mat T){
    for(size_t i=0;i<transforms_to_root_marker.size();i++)
        visualize_marker(index2id[i],T*transforms_to_root_marker[i],marker_size,point_cloud_markers);
}

void MultiCamMapper::visualize_markers(pointcloud_t &point_cloud_markers, const cv::Mat T){
    MultiCamMapper::visualize_markers(mat_arrays.transforms_to_root_marker.v,mat_arrays.transforms_to_root_marker.id, marker_size, point_cloud_markers,T);
}
