#include "image_array_detector.h"
#include <set>

using namespace std;

ImageArrayDetector::ImageArrayDetector(size_t nc, string dictionary_name)
{
    num_cams=nc;
    detectors.resize(num_cams);
#pragma omp parallel for
    for(size_t i=0;i<num_cams;i++){
        detectors[i].reset(new aruco::MarkerDetector);
        detectors[i]->getParameters().setDetectionMode(aruco::DetectionMode::DM_VIDEO_FAST,0);
        detectors[i]->setDictionary(dictionary_name);
        detectors[i]->getParameters().setCornerRefinementMethod(aruco::CornerRefinementMethod::CORNER_LINES);
    }
}

vector<vector<aruco::Marker>> ImageArrayDetector::detect_markers(vector<cv::Mat> image_array, size_t min_detections){
    vector<vector<aruco::Marker>> cam_markers(num_cams);

#pragma omp parallel for
    for(size_t cam=0;cam<num_cams;cam++)
        detectors[cam]->detect(image_array[cam],cam_markers[cam]);

    multiset<int> marker_ids;
    for(size_t cam=0;cam<num_cams;cam++)
        for(auto &marker : cam_markers[cam])
            marker_ids.insert(marker.id);

    for(size_t cam=0;cam<num_cams;cam++){//remove the elements that are not detected in enough number of cameras assuming there is only one marker for each id
        for(auto it=cam_markers[cam].begin();it!=cam_markers[cam].end();){
            if(marker_ids.count(it->id) < min_detections)
                it=cam_markers[cam].erase(it);
            else
                it++;
        }
    }

    return cam_markers;
}
