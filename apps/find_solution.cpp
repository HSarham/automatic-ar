#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/calib3d.hpp>
#include <aruco/markerdetector.h>
#include <aruco/ippe.h>
#include <aruco/cvdrawingutils.h>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <iterator>
#include <chrono>
#include "initializer.h"
#include "multicam_mapper.h"

using namespace std;

int print_usage(char* argv0){
    cout<<"Usage: "<<argv0<<" <folder_path> <marker_size> [-subseqs] [ -exclude-cam [<cam_1> [<cam2> [...] ] ]  ] ["<<endl;
    cout<<"\toptions:"<<endl
        <<"\t         -subseqs: evaluates the results only in the intervals of the subsequences in subseqs.txt."<<endl
        <<"\t         -exclude-cam: specifies a list of cameras that should be excluded from initialization and optimization."<<endl
        <<"\t         "<<endl;
    return -1;
}

int main(int argc, char*argv[])
{
    Eigen::initParallel();
    double threshold = 2.0;
    if(argc<3)
        return print_usage(argv[0]);

        set<int> excluded_cams;
        double marker_size=stod(argv[2]);

        enum ArgFlag { NONE, ExcludeCams, Threshold};
        ArgFlag arg_flag=NONE;
        string folder_path=argv[1];
        bool use_subseqs=false;
        bool tracking_only=false;
        bool with_huber=false;
        bool set_threshold=false;
        string detections_file_path=folder_path+"/aruco.detections";
        string tracking_input_solution_path=folder_path+"/../fixed/subseqs.solution";
        for(int i=4;i<argc;i++)
            if(string(argv[i])=="-subseqs"){
                use_subseqs=true;
            }
            else if(string(argv[i])=="-exclude-cams"){
                arg_flag=ExcludeCams;
            }
            else if(string(argv[i])=="-tracking-only"){
                tracking_only=true;
                arg_flag=NONE;
            }
            else if(string(argv[i])=="-with-huber"){
                with_huber=true;
                arg_flag=NONE;
            }
            else if(string(argv[i])=="-thresh"){
                set_threshold=true;
                arg_flag=Threshold;
            }
            else if(arg_flag==ExcludeCams){
                excluded_cams.insert(stoi(argv[i]));
            }
            else if(arg_flag==Threshold){
                threshold=stod(argv[i]);
                arg_flag=NONE;
            }

        string solution_file_name="";

        if(tracking_only)
            solution_file_name+="_tracking_only";

        if(use_subseqs)
            solution_file_name+="_subseqs";

        if(with_huber)
            solution_file_name+="_with_huber";

        if(excluded_cams.size()>0){
            solution_file_name+="_excluded_cams";
            for(int cam_id:excluded_cams)
                solution_file_name+="_"+to_string(cam_id);
        }

        if(set_threshold){
            char d[5];
            sprintf(d,"%.1f",threshold);
            solution_file_name+="_thresh_"+string(d);
        }

        solution_file_name+=".solution";

        string initial_solution_file_path = folder_path+"/initial"+solution_file_name;
        string final_solution_file_path = folder_path+"/final"+solution_file_name;

        vector<CamConfig> cam_configs=CamConfig::read_cam_configs(folder_path);

        vector<vector<vector<aruco::Marker>>> detections;

        if(use_subseqs){
            auto subseqs=MultiCamMapper::read_subseqs(folder_path+"/subseqs.txt");
            detections=Initializer::read_detections_file(detections_file_path,subseqs);
        }
        else
        detections=Initializer::read_detections_file(detections_file_path);

        Initializer initializer(detections,marker_size,cam_configs,excluded_cams);

        auto start=chrono::system_clock::now();

#ifdef PCL
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud_cameras,point_cloud_markers;
#endif

        const set<int> marker_ids, cam_ids;

        int num_cams=cam_configs.size();
        size_t num_markers=marker_ids.size();

        cout<<"marker_ids: ";
        for(auto it=marker_ids.begin();it!=marker_ids.end();it++)
            cout<<*it<<" ";
        cout<<endl;

        cout<<"cam ids: ";
        for(auto it=cam_ids.begin();it!=cam_ids.end();it++)
            cout<<*it<<" ";
        cout<<endl;


        //local optimization
        MultiCamMapper mcm(initializer);

        mcm.set_optmize_flag_cam_intrinsics(false);
        if(with_huber)
            mcm.set_with_huber(true);

        std::chrono::duration<double> d=chrono::system_clock::now()-start;

        mcm.write_solution_file(initial_solution_file_path);
        mcm.write_text_solution_file(initial_solution_file_path+".yaml");

#ifdef PCL
        mcm.visualize_sequence();
        point_cloud_markers.clear();
        point_cloud_cameras.clear();
        mcm.visualize_markers(point_cloud_markers);
        pcl::io::savePCDFileBinary(initial_solution_file_path+".markers.pcd",point_cloud_markers);
        mcm.visualize_cameras(point_cloud_cameras);
        pcl::io::savePCDFileBinary(initial_solution_file_path+".cameras.pcd",point_cloud_cameras);
#endif
        start=chrono::system_clock::now();
        mcm.solve();
        d += chrono::system_clock::now()-start;

        mcm.write_solution_file(final_solution_file_path);
        mcm.write_text_solution_file(final_solution_file_path+".yaml");
#ifdef PCL
        mcm.visualize_sequence();
        point_cloud_markers.clear();
        point_cloud_cameras.clear();
        mcm.visualize_markers(point_cloud_markers);
        pcl::io::savePCDFileBinary(final_solution_file_path+".markers.pcd",point_cloud_markers);
        mcm.visualize_cameras(point_cloud_cameras);
        pcl::io::savePCDFileBinary(final_solution_file_path+".cameras.pcd",point_cloud_cameras);
#endif


        int minutes=d.count()/60;
        int seconds=lround(d.count()-minutes*60);
        cout<<"The algorithm took: "<<minutes<<" minutes "<<seconds<<" seconds"<<endl;


    return 0;
}

