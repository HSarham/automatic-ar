#include "cam_config.h"
#include "filesystem.h"
#include <iostream>

using namespace filesystem;
using namespace std;

CamConfig::CamConfig()
{

}

CamConfig::CamConfig(cv::Mat c_mat,cv::Mat d_coeffs, cv::Size im_size){
    setCamMat(c_mat);
    setDistCoeffs(d_coeffs);
    setImageSize(im_size);
}

void CamConfig::setCamMat(cv::Mat c_mat){
    if(c_mat.rows != 3 || c_mat.cols != 3)
        throw std::runtime_error("The size of the input camera matrix must be 4x4!");
    c_mat.convertTo(cam_mat,CV_64FC1);
}

void CamConfig::setDistCoeffs(cv::Mat d_coeffs){
    if(d_coeffs.rows != 1 && d_coeffs.cols != 1)
        throw std::runtime_error("The size of the input matrix for distorsion coefficients must be 1xN nor Nx1!");

    dist_coeffs=cv::Mat::zeros(5,1,CV_64FC1);

    d_coeffs.convertTo(d_coeffs,CV_64FC1);
    for(int i=0;i<d_coeffs.total() && i < 5;i++)
        dist_coeffs.at<double>(i)=d_coeffs.at<double>(i);
}

void CamConfig::setImageSize(cv::Size im_size){
    image_size=im_size;
}

const cv::Mat& CamConfig::getCamMat() const{
    return cam_mat;
}

const cv::Mat& CamConfig::getDistCoeffs() const{
    return dist_coeffs;
}

const cv::Size& CamConfig::getImageSize() const{
    return image_size;
}

bool CamConfig::read_from_file(std::string path){
    cv::FileStorage file(path,cv::FileStorage::READ);
    if(!file.isOpened())
        return false;

    if(file["image_height"].isNone())
        return false;
    else
        file["image_height"]>>image_size.height;

    if(file["image_width"].isNone())
        return false;
    else
        file["image_width"]>>image_size.width;

    if(file["camera_matrix"].isNone())
        return false;
    else
        file["camera_matrix"]>>cam_mat;

    if(file["distortion_coefficients"].isNone())
        return false;
    else
        file["distortion_coefficients"]>>dist_coeffs;

    return true;
}

std::vector<CamConfig> CamConfig::read_cam_configs(std::string folder_path){
    auto dirs_list=get_dirs_list(folder_path);
    std::vector<CamConfig> result;

    std::vector<std::string> possible_extensions={"xml","yml","yaml"};

    for(std::string dir_name: dirs_list){
        for(std::string ext: possible_extensions){
            std::string file_path=folder_path+"/"+dir_name+"/"+"calib."+ext;
            CamConfig cc;
            if(cc.read_from_file(file_path))
                result.push_back(cc);
        }
    }
    return result;
}
