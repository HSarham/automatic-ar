

#ifndef _SglDisplay_QT_Internal_H
#define _SglDisplay_QT_Internal_H

#include <QWidget>
#include <QLabel>
#include <memory>
#include "sgldisplay.h"
namespace sgl{
class  SglDisplay_QT_Internal : public QLabel
{
    Q_OBJECT
public:
    SglDisplay_QT_Internal(QWidget *parent  = nullptr);

    void setParams(std::shared_ptr< SceneDrawer> sgldrawer , Scene *_scn);
public slots:
    void redraw();

private:

    std::shared_ptr<SceneDrawer> _sglDrawer;
sgl::Scene *_scene;
    void	mouseMoveEvent(QMouseEvent *event);
    void	mousePressEvent(QMouseEvent *event);
    void	mouseReleaseEvent(QMouseEvent *event);
    void	wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *ev);


    QPointF prevMousePos;
    enum MouseActions: int{MA_NONE,MA_TRANSLATE,MA_ROTATE,MA_ZOOM};
    int mouseAction=MA_NONE;

    float strengthFactor=1;

};
}
#endif
