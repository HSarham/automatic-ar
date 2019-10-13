#include <QApplication>
#include <QWidget>
#include <QLabel>
#include "sgldisplay_qt.h"
#include "sgldisplay_qt_internal.h"
namespace sgl {
bool SglDisplay_QT::IsInited=false;

SglDisplay_QT::SglDisplay_QT(std::shared_ptr<sgl::SceneDrawer> drawer,int w,int h,float f){

    _sglDrawer=drawer;
    _scn.setCameraParams(f,w,h,3);
}
int SglDisplay_QT::runApp(){
    int argc=1;
    char **argv=new char*[1];
    argv[0]=new char[100];
    argv[0][1]='a';
    argv[0][1]=0;
    QApplication QApp(argc,argv);
    _window_display=new SglDisplay_QT_Internal();
    _window_display->setParams(_sglDrawer,&_scn);
    _window_display->show();
    SglDisplay_QT::IsInited=true;
    return QApp.exec();
}

int SglDisplay_QT::display(bool blocking,bool needFullGUIInitialization){


    if (needFullGUIInitialization && !SglDisplay_QT::IsInited){


            if(!blocking)
                _windowThread=std::thread([&](){ runApp();});
            else
                runApp();
    }
    else {
        _window_display=new SglDisplay_QT_Internal();
        _window_display->setParams(_sglDrawer,&_scn);
        _window_display->show();
    }
    return 1;
}

void SglDisplay_QT::redraw(){
        _window_display->redraw();
}

}
