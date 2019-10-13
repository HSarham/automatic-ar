

#include "sgldisplay.h"
#ifdef SGL_OPENCV
#include "sgldisplay_cv.h"
#endif
#ifdef SGL_QT
#include "sgldisplay_qt.h"
#endif
namespace sgl {
std::shared_ptr<SglDisplay> SglDisplay::create(std::shared_ptr<sgl::SceneDrawer> drawer,int w,int h,float f){
  std::shared_ptr<SglDisplay> disp;
#ifdef SGL_QT
  disp=std::make_shared<SglDisplay_QT>(drawer,w,h,f);
  disp->_sglDrawer=drawer;
  drawer->setUpdateCallBack(std::bind(&SglDisplay::redraw, disp.get() ));
  return disp;
#endif
#ifdef SGL_OPENCV
  disp=std::make_shared<SglDisplay_CV>(drawer,w,h,f);
  disp->_sglDrawer=drawer;
  drawer->setUpdateCallBack(std::bind(&SglDisplay::redraw, disp.get() ));
  return disp;
#endif

}


void SglDisplay::zoom(float value){
    _scn.zoom(value);
}
void SglDisplay::rotate(float x,float z){
    _scn.rotate(x,z);
}
void SglDisplay::translate(float x,float y){
    _scn.translate(x,y);
}

void SglDisplay::setViewMatrix(sgl::Matrix44 vm){
    _scn.setViewMatrix(vm);
}

void SceneDrawer::sceneUpdated(){
    for(auto &f:v_callbckFs)f();
}

}

