#include <QtGui>
#include <QBoxLayout>
#include "sglviewer.h"
#include <iostream>
SglViewer::SglViewer(QWidget *parent){
    setScaledContents(true);
    setPixmap(QPixmap ( QString:: fromUtf8 ( ":/images/cityoflove.jpg" ) ));
    setMinimumSize(QSize(640,480));
}

void SglViewer::setDrawer(std::shared_ptr<SglDrawer> sglrawer ){
    _sglDrawer=sglrawer;
    _sglDrawer->setImageSize(cv::Size(1280,960));
    _sglDrawer->setFocalLenght(1.5);

    redraw();

}
void SglViewer::redraw(){
    if(!_sglDrawer) return;

    _image.create(_sglDrawer->_size,CV_8UC3);
    _sglDrawer->draw(_image);
    QImage _qimgR ( ( const uchar * ) ( _image.ptr<uchar> ( 0 ) ),
                    _image.cols,_image.rows, QImage::Format_RGB888 ) ;

    setPixmap ( QPixmap::fromImage ( _qimgR.rgbSwapped() ) );
}

void	SglViewer::mouseMoveEvent(QMouseEvent *event){
    if(!_sglDrawer) return;
    bool needUpdate=false;
    if (mouseAction==MA_TRANSLATE){
        float xdif=event->windowPos().x() - prevMousePos.x();
        float ydif=event->windowPos().y() - prevMousePos.y();
        _sglDrawer->translate(xdif*0.01*strengthFactor,ydif*0.01*strengthFactor);
        needUpdate=true;
    }
    else if(mouseAction==MA_ROTATE){
        float xdif=event->windowPos().x() - prevMousePos.x();
        float ydif=event->windowPos().y() - prevMousePos.y();
        _sglDrawer->rotate(ydif*0.01*strengthFactor,xdif*0.01*strengthFactor);
        needUpdate=true;
    }
    else{

    }


    if (needUpdate){
        prevMousePos=event->windowPos();
        redraw();
    }
}
void	SglViewer::mousePressEvent(QMouseEvent *event){
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

void	SglViewer::mouseReleaseEvent(QMouseEvent *event){
    mouseAction=MA_NONE;
}

void	SglViewer::wheelEvent(QWheelEvent *event){
    if(!_sglDrawer) return;
    QPoint numDegrees = event->angleDelta() / 8;
    _sglDrawer->zoom(-numDegrees .ry()*0.025*strengthFactor);
    redraw();

}





