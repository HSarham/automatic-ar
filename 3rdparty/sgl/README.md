SGL
=====
SGL (Simplest Graphics Library) is simple graphic library for rendering 3D points and lines with CPU into a memory buffer.
The main component of the libary is the sgl.h header, that implements the whole rendering engine. It is a single file that can be
easily integrated in your project. Additionally, we provide classes for easily visualizing and interacting with the 3D scenes rendered.


## 
## Main features:
        * CPU only
        * A header file only



##
## Main classes:

    * SGLScene:  main class for rendering the scene. It is implemented in the sgl.h file and is highly portable to any project
    * SGLDrawer : abstract class for drawing scenes. Useful for implementing SglDisplays
    * SglDisplay: represents a surface where presenting the rendered scene and to rotate, translate and this kind of stuff. We currently provide two
        cross-platform methods. One based on OpenCV library and another one based on Qt5.
    * SglDisplay_CV: inherits from SglDisplay and allows to show and interact with the generated scene using a window created with the OpenCv library
    * SglDisplay_QT: inherits from SglDisplay and allows to show and interact with the generated  scene using a window created with the Qt5 library (not yet)


##
## Examples:
 In utils there is an example showing how to implements a visualizer of poinclouds. In particular, the program is able to read binary PCD files and show them.
# sgl

