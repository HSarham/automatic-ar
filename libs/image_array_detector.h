#ifndef IMAGEARRAYDETECTOR_H
#define IMAGEARRAYDETECTOR_H

#include<aruco/aruco.h>
//#include<memory>

class ImageArrayDetector
{
    std::vector<std::unique_ptr<aruco::MarkerDetector>> detectors;
    size_t num_cams;
public:
    ImageArrayDetector(size_t num_cams, std::string dictionary_name="ARUCO_MIP_36h12");
    std::vector<std::vector<aruco::Marker>> detect_markers(std::vector<cv::Mat> image_array, size_t min_detections=2);
};

#endif // IMAGEARRAYDETECTOR_H
