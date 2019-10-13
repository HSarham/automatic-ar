#include <iostream>
#include <fstream>
#include <sgldisplay.h>
#include <memory>
#include <chrono>
#include <thread>
using namespace std;


//this is the class that draws the scene
class myDrawer:public sgl::SceneDrawer
{
public:
    float sz=0.4;

    void increaseSize(){

        sz*=1.1;
        sceneUpdated();
    }
    void decreaseSize(){
        sz*=0.9;
        sceneUpdated();
    }




    //the function you must reimplement that do the drawing
    void draw(sgl::Scene &scn){
            scn.clear(sgl::Color(255,255,255));
            scn.setLineSize(3);
            scn.drawLine(sgl::Point3(0,0,0),sgl::Point3(sz,0,0), sgl::Color(0,0,255));
            scn.drawLine(sgl::Point3(sz,0,0),sgl::Point3(sz,sz,0), sgl::Color(0,0,255));
            scn.drawLine(sgl::Point3(sz,sz,0),sgl::Point3(0,sz,0), sgl::Color(0,255,0));
            scn.drawLine(sgl::Point3(0,sz,0),sgl::Point3(0,0,0), sgl::Color(255,0,0));

            scn.drawLine(sgl::Point3(0,0,sz),sgl::Point3(sz,0,sz), sgl::Color(0,0,255));
            scn.drawLine(sgl::Point3(sz,0,sz),sgl::Point3(sz,sz,sz), sgl::Color(0,0,255));
            scn.drawLine(sgl::Point3(sz,sz,sz),sgl::Point3(0,sz,sz), sgl::Color(0,255,0));
            scn.drawLine(sgl::Point3(0,sz,sz),sgl::Point3(0,0,sz), sgl::Color(255,0,0));



    }

};

class CmdLineParser{int argc; char **argv; public:
CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
};
int main(int argc,char **argv){

    try{
    CmdLineParser cml(argc,argv);
    if (argc!=1|| cml["-h"])
        throw std::runtime_error("Usage:  ");

    //Creates the drawer
    auto drawer=std::make_shared<myDrawer>();
    //reads the inputfile in pcd binary format

    //creates the display
    auto sglDisplay =sgl::SglDisplay::create(drawer,1280,800,1.25);

    //put the view matrix poiting to the center of the scene
    sgl::Matrix44 cam;
    cam.translate({0,4,0});
    cam.rotateX(3.1415/2.);
    sglDisplay ->setViewMatrix(cam);

    //show in a non-blocking mode

    sglDisplay->display(false);
    int n=0;
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (n<5) drawer->increaseSize();
        else drawer->decreaseSize();
        n=(n+1) % 10;
    }
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
