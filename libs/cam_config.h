#ifndef CAMCONFIG_H
#define CAMCONFIG_H

#include <opencv2/core.hpp>
#include <vector>
#include "filesystem.h"

class CamConfig
{
    cv::Mat cam_mat;
    cv::Mat dist_coeffs;
    cv::Size image_size;
public:
    CamConfig();
    CamConfig(cv::Mat c_mat ,cv::Mat d_coeffs, cv::Size im_size);
    void setCamMat(cv::Mat);
    void setDistCoeffs(cv::Mat);
    void setImageSize(cv::Size);
    const cv::Mat& getCamMat() const;
    const cv::Mat& getDistCoeffs() const;
    const cv::Size& getImageSize() const;
    bool read_from_file(std::string path);
    static std::vector<CamConfig> read_cam_configs(std::string folder_path);
};



#endif // CAMCONFIG_H
