#include "initializer.h"
#include "multicam_mapper.h"
#include "dataset.h"
#include <opencv2/highgui.hpp>

using namespace std;

void print_usage(){
    cout<<"Usage: <path_to_data_folder> <solution_file_name> [-save-video]"<<endl;
    cout<<"Options:"<<endl;
    cout<<"\t -save-video: saves videos for each camera in the data folder"<<endl;
}

int main(int argc, char *argv[]){
    if(argc < 3){
        print_usage();
        return -1;
    }

    bool save_video=false;

    if(argc > 3){
        string option=argv[3];
        if(option=="-save-video")
            save_video=true;
    }

    string folder_path = argv[1];
    Dataset dataset(folder_path);
    string solution_file_name = argv[2];
    string solution_file_path = folder_path + "/" + solution_file_name;

    MultiCamMapper mcm;
    mcm.read_solution_file(solution_file_path);

    set<size_t> frame_nums;
    dataset.get_frame_nums(frame_nums);
    size_t num_cams=dataset.get_num_cams();

    vector<cv::Mat> frames;
    size_t frame_index=0;
    vector<cv::VideoWriter> vws(num_cams);
    auto image_sizes=mcm.get_image_sizes();
    if(save_video)
        for(size_t i=0;i<vws.size();i++)
            vws[i].open(folder_path+"/overlayed_cam_"+to_string(i)+".avi",cv::VideoWriter::fourcc('H','F','Y','U'),24.0,image_sizes[i]);

    auto detections = Initializer::read_detections_file(folder_path+"/aruco.detections");

    for(size_t frame_num : frame_nums){
        dataset.get_frame(frame_num,frames);
        vector<vector<aruco::Marker>> &markers=detections[frame_index];
        for(size_t cam=0;cam<frames.size();cam++){
            mcm.overlay_markers(frames[cam],frame_index,cam);
            for(aruco::Marker &marker : markers[cam])
                marker.draw(frames[cam]);
            cv::imshow(to_string(cam),frames[cam]);
            vws[cam]<<frames[cam];
            cv::waitKey(1);
        }
        frame_index++;
    }

    return 0;
}
