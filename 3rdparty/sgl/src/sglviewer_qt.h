

#ifndef _SGLVIEWER_H
#define _SGLVIEWER_H

#include <QWidget>
#include <QLabel>
#include <memory>
#include "sglviewer_exports.h"
#include "sgldrawer.h"

class APP_SGLVIEWER_TOOLS_API SglViewer : public QLabel
{
    Q_OBJECT
public:
    SglViewer(QWidget *parent  = nullptr);

    void setDrawer(std::shared_ptr<SglDrawer> sglrawer );
public slots:
    void redraw();

private:

     cv::Mat _image;


    std::shared_ptr<SglDrawer> _sglDrawer;
    void	mouseMoveEvent(QMouseEvent *event);
    void	mousePressEvent(QMouseEvent *event);
    void	mouseReleaseEvent(QMouseEvent *event);
    void	wheelEvent(QWheelEvent *event);


    QPointF prevMousePos;
    enum MouseActions: int{MA_NONE,MA_TRANSLATE,MA_ROTATE,MA_ZOOM};
    int mouseAction=MA_NONE;

    float strengthFactor=1;

};
#endif

