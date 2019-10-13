#ifndef _SGLDRAWER_H
#define _SGLDRAWER_H
#include "sglviewer_exports.h"
#include <opencv2/core.hpp>
class APP_SGLVIEWER_TOOLS_API SglDrawer{

public:
    cv::Size _size;
    float _f;


    void setImageSize(cv::Size s){_size=s;}
    void setFocalLenght(float f){_f=f;}



    virtual  void draw(cv::Mat &image)=0;
    virtual void zoom(float value)=0;
    virtual void rotate(float x,float z)=0;
    virtual void translate(float x,float y)=0;

};

#endif
