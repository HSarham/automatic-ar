#include "dataset.h"
#include "multicam_mapper.h"
#include "initializer.h"
#include "image_array_detector.h"
#include <chrono>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <omp.h>

using namespace std;

void print_usage(){
    cout<<"Usage: <data_folder_path> <path_to_solution_file>"<<endl;
}

cv::Mat make_mosaic(vector<cv::Mat> images, int mosaic_width){
    cv::Mat mosaic;
    if(images.size()==0)
        return mosaic;

    double image_h_over_w = double(images[0].rows)/images[0].cols;

    int num_images=images.size();

    int side_images=ceil(sqrt(num_images));

    int image_width=mosaic_width/side_images;
    int image_height=image_width*image_h_over_w;

    int width_images=side_images;
    int height_images=ceil(double(num_images)/side_images);

    mosaic=cv::Mat::zeros(image_height*height_images,image_width*width_images,images[0].type());
    for(int i=0;i<num_images;i++){
        int r = i/width_images;
        int c = i%width_images;
        cv::Mat img_resized;
        cv::resize(images[i],img_resized,cv::Size(image_width,image_height));
        img_resized.copyTo(mosaic(cv::Range(r*image_height,(r+1)*image_height),cv::Range(c*image_width,(c+1)*image_width)));
    }
    return mosaic;
}

int main(int argc, char *argv[]){
    omp_set_num_threads(5);
    if(argc<3){
        print_usage();
        return -1;
    }
//    const int min_detections=2;
    string folder_path(argv[1]);
    folder_path += '/';
    string solution_file_path(argv[2]);

    Dataset dataset(folder_path);
    vector<CamConfig> cam_configs;

    cam_configs=CamConfig::read_cam_configs(folder_path);

    size_t num_cams;
    vector<size_t> frame_nums;

    num_cams=dataset.get_num_cams();
    dataset.get_frame_nums(frame_nums);

    ImageArrayDetector iad(num_cams);

    MultiCamMapper mcm;
    mcm.read_solution_file(solution_file_path);

    map<int,cv::Mat> transforms_to_root_cam,transforms_to_root_marker;

    MultiCamMapper::MatArrays ma=mcm.get_mat_arrays();
    auto mcm_ttrc=ma.transforms_to_root_cam;
    auto mcm_ttrm=ma.transforms_to_root_marker;
    for(auto &cam_id_index: mcm_ttrc.m){
        int cam_id=cam_id_index.first;
        int cam_index=cam_id_index.second;
        transforms_to_root_cam[cam_id]=mcm_ttrc.v[cam_index];
    }
    for(auto &marker_id_index: mcm_ttrm.m){
        int marker_id=marker_id_index.first;
        int marker_index=marker_id_index.second;
        transforms_to_root_marker[marker_id]=mcm_ttrm.v[marker_index];
    }

    Initializer initializer(mcm.get_marker_size(),cam_configs);
    initializer.set_transforms_to_root_cam(transforms_to_root_cam);
    initializer.set_transforms_to_root_marker(transforms_to_root_marker);

    vector<vector<vector<aruco::Marker>>> frame_detections(1);

    mcm.set_optmize_flag_cam_poses(false);
    mcm.set_optmize_flag_marker_poses(false);
    mcm.set_optmize_flag_object_poses(true);

    size_t num_wi_frames=0;
    size_t num_frames=0;
    double sum_detections_duration=0;
    double sum_inf_duration=0;
    double sum_duration=0;
    for(size_t frame_index=0;;frame_index++){
        vector<cv::Mat> frames;

        if(frame_index<frame_nums.size()){
            //retrieve the frame
            dataset.get_frame(frame_nums[frame_index],frames);
        }
        else
            break;

        num_frames++;
        //detect the markers
        auto start=chrono::system_clock::now();
        frame_detections[0]=iad.detect_markers(frames);
        //initiate the parameters
        initializer.set_detections(frame_detections);

        int num_detections=0;
        for(size_t i=0;i<frame_detections[0].size();i++)
            num_detections+=frame_detections[0][i].size();

        sum_detections_duration+=chrono::duration<double>(chrono::system_clock::now()-start).count();

        auto inf_start=chrono::system_clock::now();
        if(num_detections>0){

            num_wi_frames++;

            initializer.obtain_pose_estimations();
            initializer.init_object_transforms();
            mcm.init(initializer.get_object_transforms(),initializer.get_frame_cam_markers());
            mcm.track();

            sum_inf_duration+=chrono::duration<double>(chrono::system_clock::now()-inf_start).count();
        }
//        else{
//            num_woi_frames++;
//        }

        chrono::duration<double> duration=chrono::system_clock::now()-start;
        double d=duration.count();
        sum_duration+=d;


        for(size_t i=0;i<frame_detections[0].size();i++){
            if(num_detections==0)
                cv::putText(frames[i],"no reliable detections",cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,1.5,cv::Scalar(0,0,255),3);
            else
                mcm.overlay_markers(frames[i],0,i);
        }

        cv::Mat mosaic=make_mosaic(frames,1536);
        cv::imshow("Tracking",mosaic);
        cv::waitKey(1);
    }
    cout<<"Averge per frame time : "<<sum_duration/num_frames<<" seconds in "<<num_frames<<" frames."<<endl;
    cout<<"Averge inference fps : "<<sum_inf_duration/num_wi_frames<<" seconds in "<<num_wi_frames<<" frames."<<endl;
    cout<<"Average detections time: "<<sum_detections_duration/num_frames<<endl;

    return 0;
}
