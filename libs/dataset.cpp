#include "dataset.h"
#include "filesystem.h"
#include <opencv2/highgui.hpp>

using namespace std;
using namespace filesystem;

Dataset::Dataset(std::string folder)
{
    folder_path=folder;
    num_cams=obtain_num_cams();
    get_frame_nums();
}

void Dataset::get_frame_nums(vector<size_t> &fns){
    for(size_t frame_num:all_frames)
        fns.push_back(frame_num);
}

void Dataset::get_frame_nums(std::set<size_t> &fn){
    fn=all_frames;
}

size_t Dataset::get_num_cams(){
    return num_cams;
}

size_t Dataset::obtain_num_cams(){

    vector<string> dirs_list=get_dirs_list(folder_path);

    int num_cams=-1;
    for(size_t i=0;i<dirs_list.size();i++){
        if(string_is_uint(dirs_list[i])){
            int dir_num=stoi(dirs_list[i]);
            if(dir_num>num_cams)
                num_cams=dir_num;
        }
    }

    num_cams++;
    return num_cams;
}

bool Dataset::string_is_uint(string s){
    if(s.size()==0)
        return false;
    for(char c: s)
        if (!isdigit(c))
            return false;
    return true;
}

void Dataset::get_frame_nums(){
    frame_nums.resize(num_cams);

    for(auto cam_num=0;cam_num<num_cams;cam_num++){
        string cam_dir_path=folder_path+"/"+to_string(cam_num);
        vector<string> files_list=get_files_list(cam_dir_path);
        sort(files_list.begin(),files_list.end());
        for(string file_name : files_list){
            size_t frame_num=0;
            if(sscanf(file_name.c_str(),"%lu.png",&frame_num)==1){
                frame_nums[cam_num].insert(frame_num);
                all_frames.insert(frame_num);
            }
        }
    }
}

void Dataset::get_frame(size_t frame_num, vector<cv::Mat> &frames){
    size_t num_cams=frame_nums.size();
    frames.resize(num_cams);

    for(size_t cam=0;cam<num_cams;cam++){
        string cam_dir_path=folder_path+"/"+to_string(cam);

        if(frame_nums[cam].count(frame_num)>0)//the cameras has that frame
            frames[cam]=cv::imread(cam_dir_path+"/"+to_string(frame_num)+".png");
    }
}
