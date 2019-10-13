#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <string>
#include <set>
#include <opencv2/core.hpp>

class Dataset
{
    std::string folder_path;
    size_t num_cams;
    size_t obtain_num_cams();

    std::vector<std::set<size_t>> frame_nums;
    std::set<size_t> all_frames;

    void get_frame_nums();

public:

    Dataset(std::string folder_path);
    static bool string_is_uint(std::string s);
    void get_frame(size_t frame_num, std::vector<cv::Mat> &frames);
    size_t get_num_cams();
    void get_frame_nums(std::vector<size_t> &fns);
    void get_frame_nums(std::set<size_t> &frame_nums);

};

#endif // DATASET_H
