#include <iostream>
#include <fstream>
#include <sgldisplay.h>
#include <memory>
#include <chrono>
using namespace std;

struct Point4f{
    sgl::Point3 pos;
    sgl::Color color;
};


void readPCD(std::string path,std::vector<Point4f>  &pointcloud);


//this is the class that draws the scene
class myDrawer:public sgl::SceneDrawer
{
    std::vector<Point4f> pointCloud;

public:

    std::vector<Point4f> & getPointCloud(){return pointCloud;}

    void setPointCloud(const std::vector<Point4f>  &pc){pointCloud=pc;}


    //the function you must reimplement that do the drawing
    void draw(sgl::Scene &scn){
            auto begin= std::chrono::high_resolution_clock::now();

            scn.clear(sgl::Color(255,255,255));
            scn.setPointSize(2);
            for(auto &p:pointCloud)
                scn.drawPoint(p.pos,p.color);
            //draw 3d axis
            scn.setLineSize(3);
            scn.drawLine(sgl::Point3(0,0,0),sgl::Point3(1,0,0), sgl::Color(0,0,255));
            scn.drawLine(sgl::Point3(0,0,0),sgl::Point3(0,1,0), sgl::Color(0,255,0));
            scn.drawLine(sgl::Point3(0,0,0),sgl::Point3(0,0,1), sgl::Color(255,0,0));

            auto end= std::chrono::high_resolution_clock::now();
            double fps=1e9/double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count());
            std::cout   <<"fps="<<fps<<std::endl; ;

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
    if (argc!=2 || cml["-h"])
        throw std::runtime_error("Usage: file.pcd");

    //Creates the drawer
    auto drawer=std::make_shared<myDrawer>();
    //reads the inputfile in pcd binary format
    readPCD(argv[1],drawer->getPointCloud());

    //creates the display
    auto sglDisplay =sgl::SglDisplay::create(drawer,1280,800,1.25);

    //put the view matrix poiting to the center of the scene
    sgl::Matrix44 cam;
    cam.translate({0,4,0});
    cam.rotateX(3.1415/2.);
    sglDisplay ->setViewMatrix(cam);

    //show and stays there until ESC pressed
    sglDisplay->display();
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}


void readPCD(std::string path,std::vector<Point4f>  &pointcloud){

    std::ifstream file(path,ios::binary);
    if (!file)throw std::runtime_error("Could not open file:" +path);

       int w=-1,h=-1;
       bool isPCDSigFound=false;
       //find .PCD line indicating the start
       string sline ;
       while ( !isPCDSigFound&& !file.eof() )
       {
           std::getline(file,sline);
           if ( sline.find ( ".PCD" ) !=string::npos )
               isPCDSigFound=true;
       }

       if ( !isPCDSigFound ) throw std::runtime_error (  "File is not of .PCD type" );
       bool beginData=false;
       while ( !beginData )
       {
           std::getline(file,sline);

           if ( sline.find ( "WIDTH" ) !=string::npos )
           {
               sscanf ( sline.c_str(),"WIDTH %d",&w );
           }

           else if ( sline.find ( "HEIGHT" ) !=string::npos )
           {
               sscanf ( sline.c_str(),"HEIGHT %d",&h );
           }
           else if ( sline.find ( "DATA" ) !=string::npos )
           {
               beginData=true;
               if ( sline.find ( "binary" ) ==string::npos )
                   throw std::runtime_error ( "File is not of binary type. Not supported yet");
           }
       }
       pointcloud.resize ( w*h );
       file.read ((char*)& pointcloud[0] ,w*h*sizeof(Point4f) );
}
