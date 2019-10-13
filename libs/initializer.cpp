#include "initializer.h"
#include "aruco_serdes.h"
#include <aruco/ippe.h>
#include <iostream>
#include <queue>
#include <fstream>

using namespace std;

double Initializer::get_marker_size(){
    return marker_size;
}

void Initializer::set_transforms_to_root_cam(std::map<int,cv::Mat> &ttrc){
    transforms_to_root_cam=ttrc;
}

void Initializer::set_transforms_to_root_marker(std::map<int,cv::Mat> &ttrm){
    transforms_to_root_marker=ttrm;
}

void Initializer::set_detections(std::vector<std::vector<std::vector<aruco::Marker> > > &dts){
    detections=dts;
}

std::map<int,cv::Mat> Initializer::get_transforms_to_root_cam(){
    return transforms_to_root_cam;
}
std::map<int,cv::Mat> Initializer::get_transforms_to_root_marker(){
    return transforms_to_root_marker;
}
std::map<int,cv::Mat> Initializer::get_object_transforms(){
    return object_transforms;
}
std::map<int, std::map<int, std::vector<aruco::Marker>>> Initializer::get_frame_cam_markers(){
    return frame_cam_markers;
}
std::vector<CamConfig> Initializer::get_cam_configs(){
    return cam_configs;
}

int Initializer::get_root_cam(){
    return root_cam;
}

int Initializer::get_root_marker(){
    return root_marker;
}

set<int> Initializer::get_cam_ids()
{
    return cam_ids;
}
set<int> Initializer::get_marker_ids(){
    return marker_ids;
}

Initializer::Initializer(double marker_s, std::vector<CamConfig> &cam_cs, const std::set<int> &excluded_cs){
    excluded_cams=excluded_cs;
    marker_size=marker_s;
    cam_configs=cam_cs;
}

Initializer::Initializer(std::vector<std::vector<std::vector<aruco::Marker>>> &dts, double marker_s, std::vector<CamConfig> &cam_cs, const std::set<int> &excluded_cs){
    excluded_cams=excluded_cs;
    marker_size=marker_s;
    cam_configs=cam_cs;
    detections=dts;
    obtain_pose_estimations();
    init_transforms();
}

void Initializer::fill_transformation_set(const map<int, map<int, vector<pair<cv::Mat,double>>>> &pose_estimations, const map<int, cv::Mat>& transforms_to_root_cam, const map<int, cv::Mat>& transforms_to_root_marker, vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>> &transformation_set){
//for object pose
    for(auto marker_it=pose_estimations.begin();marker_it!=pose_estimations.end();marker_it++){//loop on marker
        int marker_id=marker_it->first;
        auto marker_cams=marker_it->second;
        for(auto cam_it=marker_cams.begin();cam_it!=marker_cams.end();cam_it++){//loop on camera
            int cam_id=cam_it->first;
            auto pose_ests = cam_it->second;

            cv::Mat T_mr,T_rm,T_cr,T_rc;
            if(transforms_to_root_marker.find(marker_id)!=transforms_to_root_marker.end()){
                T_mr=transforms_to_root_marker.at(marker_id);
                T_rm=T_mr.inv();
            }
            else{
                T_mr=cv::Mat::eye(4,4,CV_64FC1);
                T_rm=cv::Mat::eye(4,4,CV_64FC1);
            }

            if(transforms_to_root_cam.find(cam_id)!=transforms_to_root_cam.end()){
                T_cr=transforms_to_root_cam.at(cam_id);
                T_rc=T_cr.inv();
            }
            else{
                T_cr=cv::Mat::eye(4,4,CV_64FC1);
                T_rc=cv::Mat::eye(4,4,CV_64FC1);
            }

            for(size_t i=0;i<pose_ests.size();i++){
                cv::Mat T_mc;
                pose_ests[i].first.convertTo(T_mc,CV_64FC1);
                cv::Mat T_cm=T_mc.inv();
                double error=pose_ests[i].second;
                transformation_set.push_back(make_tuple(T_cr*T_mc*T_rm, T_mr*T_cm, T_rc, error));
                //T_rc*(T_cr*T_mc*T_rm)*T_mr should transform points onto themeselves
            }

        }
    }
}

void Initializer::fill_transformation_sets(transform_type tt, const map<int, map<int, vector<pair<cv::Mat,double>>>> &pose_estimations, map<int, map<int, vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>>>> &transformation_sets){
//for cameras or markers
    for(auto it=pose_estimations.begin();it!=pose_estimations.end();it++){
        auto objects=it->second;
        if(objects.size()>1)//e.g. if more than one camera has seen the marker or more than one marker is seen in the camera
            for(auto it1=objects.begin();it1!=objects.end();++it1){//for every camera or for every marker
                int id1=it1->first;//id of the first object
                auto poses_1=it1->second;//candidate poses of the first object
                for(size_t i=0;i<poses_1.size();i++){
                    double error1=poses_1[i].second;
                    for(auto it2=next(it1);it2!=objects.end();it2++){
                        int id2=it2->first;//id of the second object
                        auto poses_2=it2->second;//candidate poses of the second object
                        for(size_t j=0;j<poses_2.size();j++){
                            double error2=poses_2[j].second;
                            switch(tt){
                            case camera:
                                transformation_sets[id1][id2].push_back(make_tuple(poses_2[j].first*poses_1[i].first.inv(), poses_1[i].first, poses_2[j].first.inv(), error1*error2));
                                break;
                            case marker:
                                transformation_sets[id1][id2].push_back(make_tuple(poses_2[j].first.inv()*poses_1[i].first, poses_1[i].first.inv(), poses_2[j].first, error1*error2));
                                break;
                            }
                        }
                    }
                }
            }
    }
}

int Initializer::find_best_transformation_min(double marker_size, const vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>>& solutions, double& min_error){
    std::pair<int,double> bestMin(-1,std::numeric_limits<double>::max());
    for(size_t i=0;i< solutions.size();i++){
        double v =get<3>(solutions[i]);
        if (v<bestMin.second)
            bestMin={i,v};
    }
    min_error=bestMin.second;
    return bestMin.first;
}


int Initializer::find_best_transformation(double marker_size, const vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>>& solutions, double& weight){
    double half_msize=marker_size/2;
    cv::Mat points(4,4,CV_64FC1);
    points.at<double>(0,0)=-half_msize;
    points.at<double>(1,0)=half_msize;
    points.at<double>(2,0)=0;
    points.at<double>(3,0)=1;
    points.at<double>(0,1)=half_msize;
    points.at<double>(1,1)=half_msize;
    points.at<double>(2,1)=0;
    points.at<double>(3,1)=1;
    points.at<double>(0,2)=half_msize;
    points.at<double>(1,2)=-half_msize;
    points.at<double>(2,2)=0;
    points.at<double>(3,2)=1;
    points.at<double>(0,3)=-half_msize;
    points.at<double>(1,3)=-half_msize;
    points.at<double>(2,3)=0;
    points.at<double>(3,3)=1;

    double min_error=numeric_limits<double>::max();
    int min_index=-1;

    for(size_t i=0;i<solutions.size();i++){
        cv::Mat T=get<0>(solutions[i]);
        double curr_error=0;
        for(size_t j=0;j<solutions.size();j++){
            cv::Mat T1_inv=get<1>(solutions[j]);
            cv::Mat T2_inv=get<2>(solutions[j]);
            cv::Mat p2=T2_inv*T*T1_inv*points;
            cv::Mat diff=points-p2;
            diff=diff.rowRange(cv::Range(0,3));
            cv::Mat diff_sq=diff.mul(diff);
            cv::reduce(diff_sq,diff_sq,0,cv::REDUCE_SUM);
            cv::sqrt(diff_sq,diff_sq);
            curr_error += sum(diff_sq)[0];
        }
        if(curr_error<min_error){
            min_index=i;
            min_error=curr_error;
            weight=min_error;
        }
    }
    return min_index;
}

void Initializer::find_best_transformations(double marker_size, const map<int, map<int, vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>>>> &transformation_sets, map<int, map<int, pair<cv::Mat,double>>> &best_transformations){

    //loop on the first item
    for(auto it1=transformation_sets.begin();it1!=transformation_sets.end();it1++){
        int id1=it1->first;
        cout<<"id1: "<<id1<<endl;
        //loop on the second item
        for(auto it2=it1->second.begin();it2!=it1->second.end();it2++){
            int id2=it2->first;
            cout<<"id2: "<<id2<<endl;
            const vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>> &solutions=it2->second;

            double min_error;
            int min_index=find_best_transformation(marker_size, solutions, min_error);
            double reproj_error=min_error;
            best_transformations[id1][id2].first=get<0>(solutions[min_index]);
            best_transformations[id1][id2].second=reproj_error;

        }
    }
}

//double Initializer::get_reprojection_error(double marker_size, aruco::Marker marker, cv::Mat r, cv::Mat t, cv::Mat cam_mat, cv::Mat dist_coeffs){

//    vector<cv::Point2f> projected_points;
//    cv::projectPoints(marker.get3DPoints(marker_size), r, t, cam_mat, dist_coeffs, projected_points);

//    double error=0;
//    for(int i=0;i<4;i++){
//        error+=(projected_points[i].x-marker[i].x)*(projected_points[i].x-marker[i].x);
//        error+=(projected_points[i].y-marker[i].y)*(projected_points[i].y-marker[i].y);
//    }
//    return error;
//}

void Initializer::make_mst(int starting_node, set<int> node_ids, const map<int, map<int, pair<cv::Mat,double>>>& adjacency, map<int, set<int>> &children){

    set<Node> nodes_outside_tree;
    for(auto it = node_ids.begin(); it != node_ids.end() ; it++){//fill the set of nodes with undetermined distances
        Node n;
        n.id=*it;
        n.parent=-1;
        if(n.id==starting_node)
            n.distance=0;
        else
            n.distance=numeric_limits<double>::max();
        nodes_outside_tree.insert(n);
    }

    while(!nodes_outside_tree.empty()){
        //find the undetermined node with the smallest distance
        auto min_node_it=nodes_outside_tree.begin();
        for(auto node_it=nodes_outside_tree.begin();node_it!=nodes_outside_tree.end();node_it++)
            if(node_it->distance<min_node_it->distance)
                min_node_it=node_it;

        //update the distance for the neighbours of the node with the minimum distance
        for(set<Node>::iterator node_it=nodes_outside_tree.begin();node_it!=nodes_outside_tree.end();node_it++){

            cv::Mat transform;
            double error=numeric_limits<double>::max();

            if(min_node_it->id<node_it->id){
                if(adjacency.find(min_node_it->id)!=adjacency.end())
                    if(adjacency.at(min_node_it->id).find(node_it->id)!=adjacency.at(min_node_it->id).end()){
                        transform=adjacency.at(min_node_it->id).at(node_it->id).first;
                        error=adjacency.at(min_node_it->id).at(node_it->id).second;
                    }
            }
            else if(adjacency.find(node_it->id)!=adjacency.end())
                    if(adjacency.at(node_it->id).find(min_node_it->id)!=adjacency.at(node_it->id).end()){
                        transform=adjacency.at(node_it->id).at(min_node_it->id).first;
                        error=adjacency.at(node_it->id).at(min_node_it->id).second;
                    }

            if(!transform.empty())//if they are neighbours
                if(error /*+ min_node_it->distance*/ < node_it->distance){//if (the edge weight + distance of the min node < distance of the current node)
                    node_it->distance=error /*+ min_node_it->distance*/;
                    if(node_it->parent != -1)
                        children[node_it->parent].erase(node_it->id);
                    children[min_node_it->id].insert(node_it->id);
                    node_it->parent=min_node_it->id;
                }
        }
        //remove the memeber with the minimum distance
        nodes_outside_tree.erase(min_node_it);
    }
}

void Initializer::find_transforms_to_root(int root_node, const map<int,set<int>> &children, const map<int, map<int, pair<cv::Mat,double>>> &best_transforms, map<int, cv::Mat> &transforms_to_root){
    transforms_to_root[root_node] = cv::Mat::eye(4,4,CV_64FC1);
    queue<int> q;
    q.push(root_node);
    cout<<"finding best transforms"<<endl;
    while(!q.empty()){
        if(children.find(q.front()) != children.end())
        for(auto child_it=children.at(q.front()).begin();child_it!=children.at(q.front()).end();child_it++){
            int child_id=*child_it;
            int parent_id=q.front();

            if(child_id<parent_id)
                transforms_to_root[child_id] = best_transforms.at(child_id).at(parent_id).first.clone();
            else
                transforms_to_root[child_id] = best_transforms.at(parent_id).at(child_id).first.inv();

            if(parent_id!=root_node)
                transforms_to_root[child_id] = transforms_to_root[parent_id]*transforms_to_root[child_id];

            q.push(child_id);
        }
        q.pop();
    }
}

vector<vector<vector<aruco::Marker>>> Initializer::read_detections_file(string path,const vector<int> &subseqs){
    ifstream detections_file(path,ios_base::binary);

    if(!detections_file.is_open())
        throw runtime_error("Could not open to read the detection file at: "+path);

    std::vector<std::vector<std::vector<aruco::Marker>>> all_markers;

    size_t num_cams;
    detections_file.read((char*)&num_cams,sizeof num_cams);
    if(!(detections_file.gcount()<sizeof num_cams))
        for(int frame_num=0;;frame_num++){
            std::vector<std::vector<aruco::Marker>> frame_markers(num_cams);
            bool end_of_data=false;

            for(int cam=0;cam<num_cams;cam++){//loop over all possible cameras
                size_t num_cam_markers;
                detections_file.read((char*)&num_cam_markers,sizeof num_cam_markers);//read the number of markers for the first camera

                if(detections_file.gcount()<sizeof num_cam_markers){//if end of file is reached exit the loop
                    end_of_data=true;
                    break;
                }
                frame_markers[cam].resize(num_cam_markers);

                for(int m=0;m<num_cam_markers;m++)
                    ArucoSerdes::deserialize_marker(detections_file,frame_markers[cam][m]);

            }
            if(end_of_data)
                break;
            all_markers.push_back(frame_markers);
        }

    if(!subseqs.empty()){
        int prev_last_frame=-1;
        for(size_t i=0;i+1<subseqs.size();i+=2){
            int first_frame=subseqs[i];
            for(int f=prev_last_frame+1;f<first_frame;f++)
                for(int c=0;c<num_cams;c++)
                    all_markers[f][c].clear();
            prev_last_frame=subseqs[i+1];
        }
    }

    return all_markers;
}

void Initializer::obtain_pose_estimations(){
    frame_cam_markers.clear();
    frame_poses_cam.clear();
    frame_poses_marker.clear();
    //cout<<"Estimating camera poses from markers.."<<endl;
    for(int frame_num=0;frame_num<detections.size();frame_num++){
        map<int,map<int,vector<pair<cv::Mat,double>>>> pose_estimations_marker, pose_estimations_cam;
        //Find all of the solutions for all of markers in each camera

        //first check if the frame has more than one detections
        int num_detections=0;
        for(int cam=0;cam<detections[frame_num].size();cam++)
            if(excluded_cams.count(cam)==0)
                num_detections += detections[frame_num][cam].size();

        if(!(num_detections >= min_detections))
            continue;

        //loop over all possible cameras
        for(int cam=0;cam<detections[frame_num].size();cam++)
            if(excluded_cams.count(cam)==0){
                size_t num_cam_markers=detections[frame_num][cam].size();

                if(num_cam_markers < 1)
                    continue;

                cam_ids.insert(cam);

                auto &cam_markers=frame_cam_markers[frame_num][cam];

                for(int m=0;m<num_cam_markers;m++){
                    aruco::Marker &marker=detections[frame_num][cam][m];

                    marker_ids.insert(marker.id);

                    cam_markers.push_back(marker);

                    vector<pair<cv::Mat,double>> solutions=aruco::solvePnP_(marker_size,marker,cam_configs[cam].getCamMat(),cam_configs[cam].getDistCoeffs());

                    solutions[0].first.convertTo(solutions[0].first,CV_64FC1);
                    pose_estimations_cam[marker.id][cam].push_back(solutions[0]);
                    pose_estimations_marker[cam][marker.id].push_back(solutions[0]);


                    if(solutions[1].second/solutions[0].second < threshold){
                        solutions[1].first.convertTo(solutions[1].first,CV_64FC1);
                        pose_estimations_cam[marker.id][cam].push_back(solutions[1]);
                        pose_estimations_marker[cam][marker.id].push_back(solutions[1]);
                    }
                }
            }

        frame_poses_cam[frame_num]=pose_estimations_cam;
        frame_poses_marker[frame_num]=pose_estimations_marker;
    }
}


void Initializer::init_transforms_cam(){
    if(!config.init_cams)
        return;
    for(int frame_num=0;frame_num<detections.size();frame_num++)
        fill_transformation_sets(transform_type::camera,frame_poses_cam[frame_num],transformation_sets_cam);
    map<int, map<int, pair<cv::Mat,double>>> best_transforms_cam;
    find_best_transformations(marker_size,transformation_sets_cam,best_transforms_cam);
    map<int, set<int>> cam_tree;
    root_cam=*cam_ids.begin();
    make_mst(root_cam,cam_ids,best_transforms_cam,cam_tree);
    cout<<"Finding the transformations to the reference camera.."<<endl;
    find_transforms_to_root(root_cam,cam_tree,best_transforms_cam,transforms_to_root_cam);
}

void Initializer::init_transforms_marker(){
    if(!config.init_markers)
        return;
    for(int frame_num=0;frame_num<detections.size();frame_num++)
        fill_transformation_sets(transform_type::marker,frame_poses_marker[frame_num],transformation_sets_marker);
    map<int, map<int, pair<cv::Mat,double>>> best_transforms_marker;
    find_best_transformations(marker_size,transformation_sets_marker,best_transforms_marker);
    map<int, set<int>> marker_tree;
    root_marker=*marker_ids.begin();
    make_mst(root_marker,marker_ids,best_transforms_marker,marker_tree);

    cout<<"Finding the transformations to the reference marker.."<<endl;
    find_transforms_to_root(root_marker,marker_tree,best_transforms_marker,transforms_to_root_marker);
}

void Initializer::init_object_transforms(){
    if(!config.init_relative_poses)
        return;
    for(auto it=frame_poses_cam.begin(); it != frame_poses_cam.end(); it++){
        int frame = it->first;
        vector<tuple<cv::Mat,cv::Mat,cv::Mat,double>> transformation_set;
        fill_transformation_set(it->second,transforms_to_root_cam,transforms_to_root_marker,transformation_set);
        double min_err;
        int best_transform_index=find_best_transformation(marker_size,transformation_set,min_err);
        if(best_transform_index >= 0)
            object_transforms[frame]=get<0>(transformation_set[best_transform_index]);
    }
}

void Initializer::init_transforms(){
    cout<<"Finding the best transformations.."<<endl;
    init_transforms_cam();
    init_transforms_marker();
    init_object_transforms();
//            //debug
//            auto t_3_5=best_transforms_marker[3][5];
//            auto t_4_5=best_transforms_marker[4][5];
//            cout<<"t_3_5 weight"<<t_3_5.second<<endl;
//            cout<<"t_4_5 weight"<<t_4_5.second<<endl;
//            MultiCamMapper::visualize_marker(3,t_3_5.first,0.04,point_cloud_markers);
//            MultiCamMapper::visualize_marker(4,t_4_5.first,0.04,point_cloud_markers);
//            MultiCamMapper::visualize_marker(5,cv::Mat::eye(4,4,CV_64FC1),0.04,point_cloud_markers);
//            pcl::io::savePCDFileBinary(folder_path + "/markers_cloud.pcd",point_cloud_markers);
//            return 0;



//   cout<<"Visualizing the camera positions in the point cloud.."<<endl;
//            //debug
//            auto t_3_5=best_transforms_marker[3][5];
//            auto t_4_5=best_transforms_marker[4][5];
//            cout<<"t_3_5 weight"<<t_3_5.second<<endl;
//            cout<<"t_4_5 weight"<<t_4_5.second<<endl;
//            MultiCamMapper::visualize_marker(3,t_3_5.first,0.04,point_cloud_markers);
//            MultiCamMapper::visualize_marker(4,t_4_5.first,0.04,point_cloud_markers);
//            MultiCamMapper::visualize_marker(5,cv::Mat::eye(4,4,CV_64FC1),0.04,point_cloud_markers);
//            pcl::io::savePCDFileBinary(folder_path + "/markers_cloud.pcd",point_cloud_markers);

//            cout<<"3_5_transfomrs:"<<transformation_sets_marker[3][5].size()<<endl;
//            cout<<"4_5_transfomrs:"<<transformation_sets_marker[4][5].size()<<endl;

//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//            pcl::visualization::PCLVisualizer pclv;
//            pclv.addPointCloud(point_cloud);

//            for(auto t: transformation_sets_marker[4][5]){
//                point_cloud->clear();
//                auto t_3_5=best_transforms_marker[3][5];
//                auto t_4_5=get<0>(t);
//                MultiCamMapper::visualize_marker(3,t_3_5.first,0.04,*point_cloud);
//                MultiCamMapper::visualize_marker(4,t_4_5,0.04,*point_cloud);
//                MultiCamMapper::visualize_marker(5,cv::Mat::eye(4,4,CV_64FC1),0.04,*point_cloud);
//                pclv.updatePointCloud(point_cloud);
//                pclv.spin();
//            }
//            //debug

}
