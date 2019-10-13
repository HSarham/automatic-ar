#include <iostream>
#include <fstream>
#include <aruco/aruco.h>
#include <algorithm>
#include <set>
#include <opencv2/highgui.hpp>
#include <cctype>
#include <algorithm>
#include <cstdio>
#include "multicam_mapper.h"
#include "dataset.h"
#include <aruco_serdes.h>

using namespace std;

int print_usage(char* argv0){
    cout<<"Usage: "<<argv0<<" <path_to_data_folder> [-d <dictionary>]"<<endl
        <<"The default values:"<<endl
        <<"\t<dictionary>: ARUCO_MIP_36h12"<<endl;
    return -1;
}

int main(int argc, char* argv[]){
    if(argc<2)
        return print_usage(argv[0]);

    string dictionary_name="ARUCO_MIP_36h12";

    if(argc>2){
        vector<string> optional_params;
        for(int i=3;i<argc;i++)
            optional_params.push_back(argv[i]);

        auto it=find(optional_params.begin(),optional_params.end(),"-d");
        if(it!=optional_params.end())
            dictionary_name=*(++it);
    }

    try{
        string folder_path = argv[1];
        Dataset dataset(folder_path);
        size_t num_cams=dataset.get_num_cams();
        string output_file_name=folder_path+"/aruco.detections";

        vector<aruco::MarkerDetector> detector(num_cams);
        for(size_t i=0;i<num_cams;i++){
            detector[i].getParameters().setDetectionMode(aruco::DetectionMode::DM_VIDEO_FAST,0);
            detector[i].setDictionary(dictionary_name);
            detector[i].getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
        }

        ofstream output_file(output_file_name,ios_base::binary);
        if(!output_file.is_open())
            throw runtime_error("Could not open a file to write output at: "+output_file_name);

        output_file.write((char*)&num_cams,sizeof(num_cams));

        const int min_detections_per_marker=1;

        std::set<size_t> frame_nums;
        dataset.get_frame_nums(frame_nums);

        std::set<int> marker_ids;
        vector<cv::Mat> frames;
        for( size_t frame_num : frame_nums){
            cout<<"frame num:"<<frame_num<<endl;
            auto start=chrono::system_clock::now();
            dataset.get_frame(frame_num,frames);
            map<int,int> markers_count;
            vector<vector<aruco::Marker>> cam_markers(num_cams);
            for(size_t cam=0;cam<num_cams;cam++){
                cv::Mat tmp_img=frames[cam];
                detector[cam].detect(tmp_img,cam_markers[cam]);
                for(size_t m=0;m<cam_markers[cam].size();m++)
                    if(markers_count.find(cam_markers[cam][m].id)!=markers_count.end())
                        markers_count[cam_markers[cam][m].id]++;
                    else
                        markers_count[cam_markers[cam][m].id]=1;
            }
            auto end=chrono::system_clock::now();
            std::chrono::duration<double> d=end-start;

            vector<vector<aruco::Marker>> new_markers(num_cams);

            for(size_t cam=0;cam<num_cams;cam++){
                auto &markers=cam_markers[cam];

                for(size_t m=0;m<markers.size();m++){
                    if(markers_count[markers[m].id]>=min_detections_per_marker)//if there is more that one instance of that marker detected in the cameras
                        new_markers[cam].push_back(markers[m]);
                }
            }

            for(size_t cam=0;cam<num_cams;cam++){
                cv::Mat tmp_img=frames[cam];
                auto new_markers_size=new_markers[cam].size();
                output_file.write((char*)&new_markers_size,sizeof(new_markers_size));

                for(size_t m=0;m<new_markers_size;m++){
                    marker_ids.insert(new_markers[cam][m].id);
                    ArucoSerdes::serialize_marker(new_markers[cam][m],output_file);
                    new_markers[cam][m].draw(tmp_img);
                }
                imshow("cam_"+to_string(cam),tmp_img);
                cv::waitKey(1);
            }

        }
        cout<<"Detected marker ids: ";
        for(auto it=marker_ids.begin();it!=marker_ids.end();it++)
            cout<<*it<<" ";
        cout<<endl;
    }
    catch(cv::Exception e){
        cout<<e.err<<endl;
    }

}
