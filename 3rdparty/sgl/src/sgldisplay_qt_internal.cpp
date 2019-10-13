#include <QApplication>

#include <QtGui>
#include <QBoxLayout>
#include "sgldisplay_qt_internal.h"
#include <iostream>
namespace sgl{

SglDisplay_QT_Internal::SglDisplay_QT_Internal(QWidget *parent){
    setScaledContents(true);
    setMinimumSize(QSize(640,480));
}

void SglDisplay_QT_Internal::setParams(std::shared_ptr<SceneDrawer> sgldrawer , Scene *scn){
    _sglDrawer=sgldrawer;
    _scene=scn;
    redraw();

}
void SglDisplay_QT_Internal::redraw(){
    if(!_sglDrawer) return;

    _sglDrawer->draw(*_scene);
    QImage _qimgR ( ( const uchar * ) ( _scene->getBuffer() ),_scene->getWidth(),_scene->getHeight(), QImage::Format_RGB888 ) ;
    setPixmap ( QPixmap::fromImage ( _qimgR.rgbSwapped() ) );
}

void	SglDisplay_QT_Internal::mouseMoveEvent(QMouseEvent *event){
    if(!_sglDrawer) return;
    bool needUpdate=false;
    if (mouseAction==MA_TRANSLATE){
        float xdif=event->windowPos().x() - prevMousePos.x();
        float ydif=event->windowPos().y() - prevMousePos.y();
        _scene->translate(xdif*0.01*strengthFactor,ydif*0.01*strengthFactor);
        needUpdate=true;
    }
    else if(mouseAction==MA_ROTATE){
        float xdif=event->windowPos().x() - prevMousePos.x();
        float ydif=event->windowPos().y() - prevMousePos.y();
        _scene->rotate(ydif*0.01*strengthFactor,xdif*0.01*strengthFactor);
        needUpdate=true;
    }
    else{

    }


    if (needUpdate){
        prevMousePos=event->windowPos();
        redraw();
    }
}
void	SglDisplay_QT_Internal::keyPressEvent(QKeyEvent *k){
    switch ( k->key() )
        {
            case Qt::Key_Escape:
        close();
        break;
    };
}

void	SglDisplay_QT_Internal::mousePressEvent(QMouseEvent *event){
    if(!_sglDrawer) return;
    if (event->button()== Qt::LeftButton){
        mouseAction=MA_ROTATE;
        prevMousePos=event->windowPos();
    }
    else if( event->button()== Qt::RightButton){
        mouseAction=MA_TRANSLATE;
        prevMousePos=event->windowPos();
    }
}

void	SglDisplay_QT_Internal::mouseReleaseEvent(QMouseEvent *event){
    mouseAction=MA_NONE;
}

void	SglDisplay_QT_Internal::wheelEvent(QWheelEvent *event){
    if(!_sglDrawer) return;
    QPoint numDegrees = event->angleDelta() / 8;
    _scene->zoom(-numDegrees .ry()*0.025*strengthFactor);
    redraw();

}


}



