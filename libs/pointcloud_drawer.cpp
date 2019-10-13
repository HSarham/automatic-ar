#include "pointcloud_drawer.h"

using namespace std;

PointCloudDrawer::PointCloudDrawer(shared_ptr<pointcloud_t> pc)
{
    point_cloud=pc;
}

void PointCloudDrawer::draw(sgl::Scene &scene){
    scene.clear(sgl::Color(0,0,0));

    for(point_t &p: *point_cloud){
        sgl::Point3 p3(p.x,p.y,p.z);
        sgl::Color c(p.r,p.g,p.b);
        scene.drawPoint(p3,c);
    }

}

