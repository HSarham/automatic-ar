

#ifndef _SglDisplay_H
#define _SglDisplay_H

#include <memory>
#include "sgl_exports.h"
#include "sgl.h"
namespace sgl{


class SceneDrawer;
class APP_SGLVIEWER_TOOLS_API SglDisplay
{
protected:
    std::shared_ptr<SceneDrawer> _sglDrawer;
    Scene _scn;
public:

    static std::shared_ptr<SglDisplay> create(std::shared_ptr<SceneDrawer> drawer,int w,int h,float f);
    virtual int display( bool blocking=false,bool needFullGUIInitialization=true)=0;
    virtual void redraw()=0;
    void zoom(float value);
    void rotate(float x,float z);
    void translate(float x,float y);
    void setViewMatrix(sgl::Matrix44 vm);

    Scene & getScene(){return _scn;}

};



class APP_SGLVIEWER_TOOLS_API SceneDrawer{
    friend class SglDisplay;
public:
    virtual void draw(Scene &scn)=0;

    //call whenever the scene contain has changed and need to redraw it
    void sceneUpdated();

    //do not use
private:

    typedef std::function<void(void)> callbf;
    std::vector<callbf > v_callbckFs;

    void setUpdateCallBack(callbf f) {v_callbckFs.push_back(f);}
};
}
#endif

