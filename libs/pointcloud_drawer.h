#ifndef POINTCLOUDDRAWER_H
#define POINTCLOUDDRAWER_H

#include "sgldisplay.h"

#ifdef PCL

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#define point_t pcl::PointXYZRGB
#define pointcloud_t pcl::PointCloud<pcl::PointXYZRGB>

#else

#define point_t Point3D
#define pointcloud_t std::vector<Point3D>

struct Point3D{
    Point3D(unsigned char R,unsigned char G,unsigned char B){
        r=R;
        g=G;
        b=B;
    }
    float x=0;
    float y=0;
    float z=0;
    unsigned char r=0;
    unsigned char g=0;
    unsigned char b=0;
};

#endif

class PointCloudDrawer:public sgl::SceneDrawer
{
    std::shared_ptr<pointcloud_t> point_cloud;
public:
    PointCloudDrawer(std::shared_ptr<pointcloud_t>);
    void draw(sgl::Scene &scene);
};



#endif // POINTCLOUDDRAWER_H
